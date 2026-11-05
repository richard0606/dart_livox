# ws_livox — DART 激光雷达建图与目标检测

Livox MID-360 + FAST-LIO 的建图/测距/空中目标检测工作区。本仓库只包含自研部分：
`lidar_sector_tools` 包、启动脚本、systemd 服务与调试工具；第三方驱动与 FAST-LIO 需按下文单独获取。

## 功能

| 节点 | 说明 |
|---|---|
| `sector_range_node` | 扇区测距：全向 12 扇区最近距离 + 指定扇区目标详情 |
| `dart_lidar_targets` | 面向飞镖场景：固定目标（18–32m）、空中目标（背景扣除 + 跟踪）、抛体落点预测 |

### 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/lidar/sector_scan` | `Float32MultiArray` | 12 个方位扇区的最近距离（-1=无目标） |
| `/lidar/sector_target` | `Float32MultiArray` | `[距离, 方位, 仰角, x, y, z, 点数, 簇尺寸]` |
| `/lidar/fixed_target` | `Float32MultiArray` | `[距离, 方位, 仰角, x, y, z, 点数]` |
| `/lidar/air_target` | `Float32MultiArray` | `[x, y, z, vx, vy, vz, 点数, 跟踪时长]` |
| `/lidar/air_path` | `nav_msgs/Path` | 空中目标最近 60 帧轨迹 |
| `/lidar/impact_point` | `Float32MultiArray` | `[落点 x, y, z, 剩余时间, 拟合残差, 点数]` |

## 依赖

- ROS 2 Humble、PCL、Eigen
- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)（编译安装到 `/usr/local`）
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)（放到 `src/livox_ros_driver2`）
- [FAST_LIO_ROS2](https://github.com/）对应 ROS2 分支（放到 `src/FAST_LIO_ROS2`）

```bash
# Livox-SDK2
cd external/Livox-SDK2 && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j && sudo make install && sudo ldconfig

# 驱动与 FAST-LIO
cd src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
# FAST_LIO_ROS2 按上游 README 获取（注意包含 ikd-Tree 子模块）
```

**编译注意**：`livox_ros_driver2` 需要显式传 `DISTRO_ROS`，否则 CMake 走错分支报
`LIVOX_INTERFACES_INCLUDE_DIRECTORIES NOTFOUND`：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DDISTRO_ROS=humble
```

## 运行

```bash
# 一键启动：驱动 + FAST-LIO(rviz=false) + 目标检测
./run_lidar_stack.sh

# 或单独启动
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch fast_lio mapping.launch.py rviz:=false
ros2 launch lidar_sector_tools dart_lidar_targets.launch.py
```

开机自启（systemd）：

```bash
sudo install -m 644 dart_lidar.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now dart_lidar.service
# 日志：~/ws_livox/lidar_logs/（每次重启归档为 *.log.1）
```

`run_lidar_stack.sh` 内置看门狗：每 5 秒检查 `livox_lidar_publisher`、`laser_mapping`、
`dart_lidar_targets` 三个节点，连续两次缺失自动重启整套（实测约 30 秒恢复）。

## 网络

雷达默认 IP `192.168.1.112`，主机 `192.168.1.50`（配置见
`src/livox_ros_driver2/config/MID360_config.json`）。`net_mid360.sh` 用于配置主机网口与探测雷达 IP。

## 关键参数

`src/lidar_sector_tools/config/dart_lidar_params.yaml`：

| 参数 | 默认 | 说明 |
|---|---|---|
| `az_center_deg` | 0 | 扇区中心，0=正装 / 180=反装 |
| `az_half_width_deg` | 30 | 扇区半宽 |
| `fixed_range_min_m/max_m` | 18/32 | 固定目标距离窗口 |
| `air_min_height_m` | 4 | 空中目标最低高度（雷达系 z） |
| `bg_learn_s` | 5 | 启动后静态背景学习时长 |
| `bg_update_hits` | 30 | 体素连续出现多少帧后并入背景（雷达移动后自适应；0=关闭） |
| `dynamic_hits` | 2 | 连续多少帧确认动态目标 |
| `impact_z_target_m` | 0 | 落点预测的地面目标高度 |
| `impact_max_residual_m` | 1.0 | 落点拟合残差超过此值不发布（0=不过滤） |
