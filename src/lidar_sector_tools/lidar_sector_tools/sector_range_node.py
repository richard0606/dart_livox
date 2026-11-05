#!/usr/bin/env python3
# 扇区测距节点
#
# 在雷达坐标系的一个方向扇区内，找最近（或最稠密）的目标并发布距离。
# 输入默认用 FAST-LIO 的 /cloud_registered_body（已运动补偿，传感器系），
# 也可换成任何 sensor_msgs/PointCloud2 话题。
#
# 输出 std_msgs/Float32MultiArray（输出话题可配）:
#   [0] distance_m       目标簇最近点距离
#   [1] azimuth_deg      方位角 atan2(y, x)
#   [2] elevation_deg    仰角 atan2(z, sqrt(x^2+y^2))
#   [3] x, [4] y, [5] z  目标簇质心（雷达系）
#   [6] point_count      簇内点数
#   [7] cluster_size_m   簇包围盒对角线长度（粗略尺寸）
# 未找到目标时发布空 data。

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray


class SectorRangeNode(Node):
    def __init__(self):
        super().__init__('sector_range')

        self.declare_parameter('cloud_topic', '/cloud_registered_body')
        self.declare_parameter('output_topic', '/lidar/sector_target')
        self.declare_parameter('scan_topic', '/lidar/sector_scan')
        # >1 时按方位均分扇区，发布每个扇区的最近距离到 scan_topic
        self.declare_parameter('sector_count', 12)
        self.declare_parameter('az_min_deg', -30.0)
        self.declare_parameter('az_max_deg', 30.0)
        self.declare_parameter('el_min_deg', -7.0)
        self.declare_parameter('el_max_deg', 45.0)
        self.declare_parameter('range_min_m', 0.5)
        self.declare_parameter('range_max_m', 40.0)
        self.declare_parameter('min_height_m', -2.0)   # 目标最低 z（雷达系，向上为正）
        self.declare_parameter('voxel_size_m', 0.15)
        self.declare_parameter('min_points', 5)
        # nearest: 最近的稠密体素；densest: 点数最多的体素（适合找远处大目标）
        self.declare_parameter('select', 'densest')

        cloud_topic = self.get_parameter('cloud_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.sub = self.create_subscription(
            PointCloud2, cloud_topic, self.callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(Float32MultiArray, output_topic, 10)
        self.scan_pub = self.create_publisher(
            Float32MultiArray, self.get_parameter('scan_topic').value, 10)
        self.get_logger().info(f'扇区测距: 订阅 {cloud_topic} -> 发布 {output_topic}')

    # 每次回调都重新读参数，方便运行期用 ros2 param set 调扇区
    def _params(self):
        return dict(
            az_min=self.get_parameter('az_min_deg').value,
            az_max=self.get_parameter('az_max_deg').value,
            el_min=self.get_parameter('el_min_deg').value,
            el_max=self.get_parameter('el_max_deg').value,
            r_min=self.get_parameter('range_min_m').value,
            r_max=self.get_parameter('range_max_m').value,
            z_min=self.get_parameter('min_height_m').value,
            voxel=self.get_parameter('voxel_size_m').value,
            min_pts=self.get_parameter('min_points').value,
            select=self.get_parameter('select').value,
            sector_count=int(self.get_parameter('sector_count').value),
        )

    def callback(self, msg):
        p = self._params()
        try:
            pts = point_cloud2.read_points_numpy(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception as e:  # 点云格式异常时不崩溃
            self.get_logger().warn(f'点云解析失败: {e}', throttle_duration_sec=5.0)
            self.publish_empty()
            return

        if pts is None or len(pts) == 0:
            self.publish_empty()
            return

        # read_points_numpy 返回 (N, len(field_names)) 的普通数组
        x = pts[:, 0].astype(np.float64)
        y = pts[:, 1].astype(np.float64)
        z = pts[:, 2].astype(np.float64)

        horiz = np.hypot(x, y)
        r = np.sqrt(x * x + y * y + z * z)
        az = np.degrees(np.arctan2(y, x))
        el = np.degrees(np.arctan2(z, horiz))

        mask = (
            (az >= p['az_min']) & (az <= p['az_max'])
            & (el >= p['el_min']) & (el <= p['el_max'])
            & (r >= p['r_min']) & (r <= p['r_max'])
            & (z >= p['z_min'])
        )
        if not np.any(mask):
            self.publish_empty()
            return

        x, y, z, r = x[mask], y[mask], z[mask], r[mask]

        # 分扇区扫描：每个方位扇区独立取最近稠密体素的距离
        if p['sector_count'] > 1:
            self.publish_scan(x, y, z, r, p['sector_count'], p['min_pts'], p['voxel'])

        # 体素聚类：统计每个体素的点数，选最近或最稠密的
        voxel = np.floor(np.stack([x, y, z], axis=1) / max(p['voxel'], 1e-3)).astype(np.int64)
        uniq, inverse, counts = np.unique(
            voxel, axis=0, return_inverse=True, return_counts=True)
        dense = counts >= p['min_pts']
        if not np.any(dense):
            self.publish_empty()
            return

        centers = (uniq + 0.5) * p['voxel']
        center_r = np.linalg.norm(centers, axis=1)

        if p['select'] == 'nearest':
            score = np.where(dense, center_r, np.inf)
            best = int(np.argmin(score))
        else:
            score = np.where(dense, counts, -1)
            best = int(np.argmax(score))

        sel = inverse == best
        cx, cy, cz = x[sel], y[sel], z[sel]

        nearest_idx = int(np.argmin(r[sel]))
        dist = float(r[sel][nearest_idx])
        azimuth = float(math.degrees(math.atan2(cy.mean(), cx.mean())))
        elevation = float(math.degrees(math.atan2(
            cz.mean(), math.hypot(cx.mean(), cy.mean()))))

        size = float(np.linalg.norm(
            [cx.max() - cx.min(), cy.max() - cy.min(), cz.max() - cz.min()]))

        out = Float32MultiArray()
        out.data = [
            dist, azimuth, elevation,
            float(cx.mean()), float(cy.mean()), float(cz.mean()),
            float(len(cx)), size,
        ]
        self.pub.publish(out)

    def publish_scan(self, x, y, z, r, n_sectors, min_pts, voxel):
        """按方位均分扇区，每区发布最近稠密体素的距离（无目标填 -1）。
        数组下标 i 对应方位区间 [-180 + i*360/N, -180 + (i+1)*360/N)。"""
        az = np.degrees(np.arctan2(y, x))
        edges = np.linspace(-180.0, 180.0, n_sectors + 1)
        data = []
        for i in range(n_sectors):
            m = (az >= edges[i]) & (az < edges[i + 1])
            if not np.any(m):
                data.append(-1.0)
                continue
            vx, vy, vz, vr = x[m], y[m], z[m], r[m]
            vox = np.floor(np.stack([vx, vy, vz], axis=1) / max(voxel, 1e-3)).astype(np.int64)
            uniq, inverse, counts = np.unique(
                vox, axis=0, return_inverse=True, return_counts=True)
            dense = counts >= min_pts
            if not np.any(dense):
                data.append(-1.0)
                continue
            centers = (uniq + 0.5) * voxel
            center_r = np.linalg.norm(centers, axis=1)
            best = int(np.argmin(np.where(dense, center_r, np.inf)))
            data.append(float(vr[inverse == best].min()))
        self.scan_pub.publish(Float32MultiArray(data=data))

    def publish_empty(self):
        self.pub.publish(Float32MultiArray())


def main(args=None):
    rclpy.init(args=args)
    node = SectorRangeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
