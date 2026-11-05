#!/usr/bin/env python3
# DART 雷达目标检测节点
#
# 面向飞镖场景：在雷达系的一个方向扇区内同时检测两类目标
#   1) 固定目标：距离约 20~30m 的地面/基地目标，输出距离与方位；
#   2) 空中目标：高于雷达若干米（默认 4m 以上）的飞行物，输出位置、速度与轨迹。
#
# 扇区中心可配置 0°（雷达正前方）或 180°（反装时），半宽默认 ±30°。
# 坐标均为雷达系（x 前、y 左、z 上），与 /cloud_registered_body 一致。
#
# 输出话题（均可配）：
#   /lidar/fixed_target  Float32MultiArray [distance, az_deg, el_deg, x, y, z, points]
#   /lidar/air_target    Float32MultiArray [x, y, z, vx, vy, vz, points, age_s]
#   /lidar/air_path      nav_msgs/Path      （空中目标最近 N 帧轨迹）
# 没有目标时对应话题发布空数组 / 空 Path。

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


class DartLidarTargets(Node):
    def __init__(self):
        super().__init__('dart_lidar_targets')

        self.declare_parameter('cloud_topic', '/cloud_registered_body')
        self.declare_parameter('world_cloud_topic', '/cloud_registered')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('fixed_topic', '/lidar/fixed_target')
        self.declare_parameter('air_topic', '/lidar/air_target')
        self.declare_parameter('path_topic', '/lidar/air_path')

        # 背景扣除：世界系体素累积静态背景，只有新出现的点才算动态目标
        self.declare_parameter('use_background', True)
        self.declare_parameter('bg_learn_s', 5.0)      # 启动后学习背景的时长
        self.declare_parameter('bg_voxel_m', 0.25)
        self.declare_parameter('bg_max_voxels', 3000000)
        self.declare_parameter('dynamic_hits', 2)      # 连续出现多少帧才确认为动态
        self.declare_parameter('dynamic_ttl_s', 2.0)   # 动态体素记忆时长

        # 扇区：中心 0=正前方，180=反装；半宽默认 ±30°
        self.declare_parameter('az_center_deg', 0.0)
        self.declare_parameter('az_half_width_deg', 30.0)
        self.declare_parameter('el_min_deg', -7.0)
        self.declare_parameter('el_max_deg', 52.0)

        # 固定目标：约 25m 左右
        self.declare_parameter('fixed_range_min_m', 18.0)
        self.declare_parameter('fixed_range_max_m', 32.0)
        self.declare_parameter('fixed_voxel_m', 0.4)
        self.declare_parameter('fixed_min_points', 8)

        # 空中目标：高于雷达约 6m
        self.declare_parameter('air_min_height_m', 4.0)
        self.declare_parameter('air_max_range_m', 100.0)
        self.declare_parameter('air_voxel_m', 0.3)
        self.declare_parameter('air_min_points', 2)
        self.declare_parameter('air_assoc_gate_m', 3.0)
        self.declare_parameter('air_max_age_s', 1.5)
        self.declare_parameter('air_path_len', 60)

        # 落点预测：用轨迹拟合抛体运动，预测与地面目标高度平面的交点
        self.declare_parameter('impact_topic', '/lidar/impact_point')
        self.declare_parameter('impact_z_target_m', 0.0)   # 地面目标高度（雷达系 z）
        self.declare_parameter('impact_fit_points', 12)    # 参与拟合的最近轨迹点数
        self.declare_parameter('impact_min_points', 4)     # 最少点数才预测
        self.declare_parameter('gravity', 9.81)

        self.fixed_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter('fixed_topic').value, 10)
        self.air_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter('air_topic').value, 10)
        self.path_pub = self.create_publisher(
            Path, self.get_parameter('path_topic').value, 10)
        self.impact_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter('impact_topic').value, 10)

        self.sub = self.create_subscription(
            PointCloud2, self.get_parameter('cloud_topic').value,
            self.callback, qos_profile_sensor_data)
        self.world_sub = self.create_subscription(
            PointCloud2, self.get_parameter('world_cloud_topic').value,
            self.callback_world, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self.callback_odom, qos_profile_sensor_data)

        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.latest_odom = None
        self.bg = set()              # 静态背景体素 (ix, iy, iz)
        self.dyn_hits = {}           # 动态候选体素 -> 连续命中次数
        self.dyn_last = {}           # 动态候选体素 -> 最后出现时间

        self.prev_air = None   # (x, y, z, t)
        self.air_vel = np.zeros(3)
        self.track_age = 0.0
        self.track = []        # [(x, y, z)] body 系

        self.get_logger().info(
            f"雷达目标检测: 订阅 {self.get_parameter('cloud_topic').value}, "
            f"扇区中心 {self.get_parameter('az_center_deg').value}°")

    # ---------------- 点云处理 ----------------
    def callback(self, msg):
        try:
            pts = point_cloud2.read_points_numpy(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception as e:
            self.get_logger().warn(f'点云解析失败: {e}', throttle_duration_sec=5.0)
            return
        if pts is None or len(pts) == 0:
            self.publish_empty(msg.header)
            return

        x = pts[:, 0].astype(np.float64)
        y = pts[:, 1].astype(np.float64)
        z = pts[:, 2].astype(np.float64)
        r = np.sqrt(x * x + y * y + z * z)
        az = np.degrees(np.arctan2(y, x))
        el = np.degrees(np.arctan2(z, np.hypot(x, y)))

        center = float(self.get_parameter('az_center_deg').value)
        half = float(self.get_parameter('az_half_width_deg').value)
        d_az = np.abs(wrap_deg(az - center))
        sector = (
            (d_az <= half)
            & (el >= self.get_parameter('el_min_deg').value)
            & (el <= self.get_parameter('el_max_deg').value)
        )

        self.detect_fixed(x, y, z, r, az, el, sector)
        # 开背景扣除时，空中目标由世界系回调负责（排除静态结构）
        if not self.get_parameter('use_background').value:
            self.detect_air(x, y, z, r, sector, msg.header)

    # ---------------- 背景扣除（世界系） ----------------
    def callback_odom(self, msg):
        self.latest_odom = msg

    @staticmethod
    def world_to_body(pts_world, odom):
        """世界系点 -> 雷达 body 系。FAST-LIO 的 /Odometry 是 body->world。"""
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        t = np.array([p.x, p.y, p.z])
        # R^T (p - t)  ==  (p - t) @ R
        return (pts_world - t) @ R

    def callback_world(self, msg):
        if not self.get_parameter('use_background').value:
            return
        if self.latest_odom is None:
            return
        try:
            pts = point_cloud2.read_points_numpy(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception:
            return
        if pts is None or len(pts) == 0:
            return

        wx = pts[:, 0].astype(np.float64)
        wy = pts[:, 1].astype(np.float64)
        wz = pts[:, 2].astype(np.float64)

        voxel = float(self.get_parameter('bg_voxel_m').value)
        vox = np.floor(np.stack([wx, wy, wz], axis=1) / max(voxel, 1e-3)).astype(np.int64)
        keys = list(map(tuple, vox.tolist()))

        now = self.get_clock().now().nanoseconds * 1e-9
        learn_s = float(self.get_parameter('bg_learn_s').value)
        if now - self.start_time < learn_s:
            # 学习期：把当前场景全部记为静态背景
            if len(self.bg) < int(self.get_parameter('bg_max_voxels').value):
                self.bg.update(keys)
            return

        hits_req = int(self.get_parameter('dynamic_hits').value)
        ttl = float(self.get_parameter('dynamic_ttl_s').value)

        # 过期动态候选清理
        for k in [k for k, t in self.dyn_last.items() if now - t > ttl]:
            self.dyn_last.pop(k, None)
            self.dyn_hits.pop(k, None)

        seen = set()
        dyn_idx = []
        for i, k in enumerate(keys):
            if k in self.bg:
                continue
            seen.add(k)
            self.dyn_hits[k] = self.dyn_hits.get(k, 0) + 1
            self.dyn_last[k] = now
            if self.dyn_hits[k] >= hits_req:
                dyn_idx.append(i)

        # 未出现的候选逐渐衰减，避免一次噪声就长期有效
        for k in [k for k in self.dyn_hits if k not in seen]:
            self.dyn_hits[k] -= 1
            if self.dyn_hits[k] <= 0:
                self.dyn_hits.pop(k, None)

        if not dyn_idx:
            self.air_pub.publish(Float32MultiArray())
            self._publish_path(msg.header)
            return

        pts_world = np.stack([wx[dyn_idx], wy[dyn_idx], wz[dyn_idx]], axis=1)
        body = self.world_to_body(pts_world, self.latest_odom)
        bx, by, bz = body[:, 0], body[:, 1], body[:, 2]

        r = np.sqrt(bx * bx + by * by + bz * bz)
        az = np.degrees(np.arctan2(by, bx))
        el = np.degrees(np.arctan2(bz, np.hypot(bx, by)))

        center = float(self.get_parameter('az_center_deg').value)
        half = float(self.get_parameter('az_half_width_deg').value)
        d_az = np.abs(wrap_deg(az - center))
        sector = (
            (d_az <= half)
            & (el >= self.get_parameter('el_min_deg').value)
            & (el <= self.get_parameter('el_max_deg').value)
        )
        hmin = float(self.get_parameter('air_min_height_m').value)
        rmax = float(self.get_parameter('air_max_range_m').value)
        mask = sector & (bz >= hmin) & (r <= rmax)
        if not np.any(mask):
            self._decay_air(now)
            self.air_pub.publish(Float32MultiArray())
            self._publish_path(msg.header)
            return

        voxel = float(self.get_parameter('air_voxel_m').value)
        min_pts = int(self.get_parameter('air_min_points').value)
        cx, cy, cz = self._largest_cluster(
            bx[mask], by[mask], bz[mask], voxel, min_pts)
        if cx is None:
            self._decay_air(now)
            self.air_pub.publish(Float32MultiArray())
            self._publish_path(msg.header)
            return

        self._update_air_track(cx, cy, cz, now, msg.header)

    # ---------------- 固定目标 ----------------
    def detect_fixed(self, x, y, z, r, az, el, sector):
        rmin = float(self.get_parameter('fixed_range_min_m').value)
        rmax = float(self.get_parameter('fixed_range_max_m').value)
        mask = sector & (r >= rmin) & (r <= rmax)
        if not np.any(mask):
            self.fixed_pub.publish(Float32MultiArray())
            return

        voxel = float(self.get_parameter('fixed_voxel_m').value)
        min_pts = int(self.get_parameter('fixed_min_points').value)
        cx, cy, cz = self._largest_cluster(x[mask], y[mask], z[mask], voxel, min_pts)
        if cx is None:
            self.fixed_pub.publish(Float32MultiArray())
            return

        dist = float(math.sqrt(cx * cx + cy * cy + cz * cz))
        azimuth = float(math.degrees(math.atan2(cy, cx)))
        elevation = float(math.degrees(math.atan2(cz, math.hypot(cx, cy))))
        self.fixed_pub.publish(Float32MultiArray(data=[
            dist, azimuth, elevation, float(cx), float(cy), float(cz),
            float(self._last_cluster_points),
        ]))

    # ---------------- 空中目标 ----------------
    def detect_air(self, x, y, z, r, sector, header):
        hmin = float(self.get_parameter('air_min_height_m').value)
        rmax = float(self.get_parameter('air_max_range_m').value)
        mask = sector & (z >= hmin) & (r <= rmax)
        now = self.get_clock().now().nanoseconds * 1e-9

        if not np.any(mask):
            self._decay_air(now)
            self.air_pub.publish(Float32MultiArray())
            self._publish_path(header)
            return

        voxel = float(self.get_parameter('air_voxel_m').value)
        min_pts = int(self.get_parameter('air_min_points').value)
        cx, cy, cz = self._largest_cluster(x[mask], y[mask], z[mask], voxel, min_pts)
        if cx is None:
            self._decay_air(now)
            self.air_pub.publish(Float32MultiArray())
            self._publish_path(header)
            return

        pos = np.array([cx, cy, cz])
        self._update_air_track(pos[0], pos[1], pos[2], now, header)

    def _update_air_track(self, cx, cy, cz, now, header):
        """帧间关联 + 速度估计 + 轨迹发布（body 系输入）。"""
        pos = np.array([cx, cy, cz])
        gate = float(self.get_parameter('air_assoc_gate_m').value)

        if self.prev_air is not None:
            px, py, pz, pt = self.prev_air
            dt = max(now - pt, 1e-3)
            if np.linalg.norm(pos - np.array([px, py, pz])) <= gate:
                v = (pos - np.array([px, py, pz])) / dt
                # 一阶低通，抑制点云抖动
                self.air_vel = 0.6 * self.air_vel + 0.4 * v
                self.track_age += dt
            else:
                # 关联失败：视为新目标
                self.air_vel = np.zeros(3)
                self.track_age = 0.0
                self.track.clear()
        else:
            self.track_age = 0.0

        self.prev_air = (cx, cy, cz, now)
        self.track.append((cx, cy, cz, now))
        max_len = int(self.get_parameter('air_path_len').value)
        if len(self.track) > max_len:
            self.track = self.track[-max_len:]

        self.air_pub.publish(Float32MultiArray(data=[
            float(cx), float(cy), float(cz),
            float(self.air_vel[0]), float(self.air_vel[1]), float(self.air_vel[2]),
            float(self._last_cluster_points), float(self.track_age),
        ]))
        self._publish_path(header)
        self._predict_impact(now)

    def _decay_air(self, now):
        if self.prev_air is not None:
            _, _, _, pt = self.prev_air
            if now - pt > float(self.get_parameter('air_max_age_s').value):
                self.prev_air = None
                self.air_vel = np.zeros(3)
                self.track_age = 0.0
                self.track.clear()

    def _publish_path(self, header):
        path = Path()
        path.header = header
        for (px, py, pz, _) in self.track:
            ps = PoseStamped()
            ps.header = header
            ps.pose.position.x = px
            ps.pose.position.y = py
            ps.pose.position.z = pz
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    # ---------------- 落点预测 ----------------
    def _predict_impact(self, now):
        """用最近轨迹点拟合抛体运动，预测与 z=impact_z_target 平面的交点。
        输出 [x, y, z, 距落点剩余时间 s, 拟合残差 m, 参与点数]。"""
        n_fit = int(self.get_parameter('impact_fit_points').value)
        n_min = int(self.get_parameter('impact_min_points').value)
        pts = self.track[-max(n_fit, n_min):]
        if len(pts) < max(n_min, 3):
            self.impact_pub.publish(Float32MultiArray())
            return

        t0 = pts[0][3]
        ts = np.array([p[3] - t0 for p in pts])
        xs = np.array([p[0] for p in pts])
        ys = np.array([p[1] for p in pts])
        zs = np.array([p[2] for p in pts])
        if ts[-1] - ts[0] < 1e-3:
            return

        g = float(self.get_parameter('gravity').value)

        # 水平：匀速直线 x = x0 + vx*t
        A = np.vstack([np.ones_like(ts), ts]).T
        (x0, vx), *_ = np.linalg.lstsq(A, xs, rcond=None)
        (y0, vy), *_ = np.linalg.lstsq(A, ys, rcond=None)

        # 垂直：z = z0 + vz*t - 0.5*g*t^2（g 已知）
        (z0, vz), *_ = np.linalg.lstsq(A, zs + 0.5 * g * ts * ts, rcond=None)

        z_target = float(self.get_parameter('impact_z_target_m').value)
        disc = vz * vz + 2.0 * g * (z0 - z_target)
        if disc < 0.0:
            self.impact_pub.publish(Float32MultiArray())
            return
        t_impact = (vz + math.sqrt(disc)) / g
        x_imp = x0 + vx * t_impact
        y_imp = y0 + vy * t_impact

        # 拟合残差（水平 + 垂直的 RMS）
        pred_x = x0 + vx * ts
        pred_y = y0 + vy * ts
        pred_z = z0 + vz * ts - 0.5 * g * ts * ts
        resid = float(np.sqrt(np.mean(
            (xs - pred_x) ** 2 + (ys - pred_y) ** 2 + (zs - pred_z) ** 2)))

        # 剩余时间：从当前时刻到落点（ts 以 t0 为零点）
        remaining = max(0.0, t_impact - (now - t0))
        self.impact_pub.publish(Float32MultiArray(data=[
            float(x_imp), float(y_imp), float(z_target),
            float(remaining), resid, float(len(pts)),
        ]))

    # ---------------- 工具 ----------------
    def _largest_cluster(self, x, y, z, voxel, min_pts):
        """体素聚类，返回点数最多的簇的质心（点数不足返回 None）。"""
        self._last_cluster_points = 0
        if len(x) == 0:
            return None, None, None
        vox = np.floor(np.stack([x, y, z], axis=1) / max(voxel, 1e-3)).astype(np.int64)
        uniq, inverse, counts = np.unique(
            vox, axis=0, return_inverse=True, return_counts=True)
        dense = counts >= max(min_pts, 1)
        if not np.any(dense):
            return None, None, None
        best = int(np.argmax(np.where(dense, counts, -1)))
        sel = inverse == best
        self._last_cluster_points = int(np.sum(sel))
        return (float(np.mean(x[sel])), float(np.mean(y[sel])),
                float(np.mean(z[sel])))

    def publish_empty(self, header):
        self.fixed_pub.publish(Float32MultiArray())
        self.air_pub.publish(Float32MultiArray())
        self._publish_path(header)


def main(args=None):
    rclpy.init(args=args)
    node = DartLidarTargets()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
