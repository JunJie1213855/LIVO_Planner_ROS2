# Real2Sim — 真机 FAST-LIO2 地图转入 SCAN-Planner 仿真器

本文介绍如何将 **FAST-LIO2 + 真实 LiDAR/IMU 采集的建图数据** 转为 **SCAN-Planner 仿真器的地图输入**，从而在已知真实室内场景中离线 / 在线运行完整导航链路。

- **适用场景**：室内、90°×360° LiDAR（如 Livox Mid-360） + 外部 IMU
- **LIO 算法**：[FAST-LIO2](https://github.com/hku-mars/FAST_LIO)，提前编译在 `~/rosws/fast_lio2_ros2`
- 核心思路：FAST-LIO2 建图 → 稠密点云（PCD 或 live 话题）→ 仿真器消费 → SCAN‑Planner 规划 + 控制
- 参考文档：[仿真器说明](scan_simulator.md) · [真机模式](example.md#7-真机--外部里程计模式可选) · [编译](scan_planner_compiled.md)

---

## 1. 总体流程

### 参数速查

| 阶段 | 动作 | 关键主题 / 帧 |
|---|---|---|
| **LIO 建图** | `fast_lio` 在线建图 | 输出 `/cloud_registered`（world 帧）、`/Odometry` |
| **离线回放** | 保存 .pcd → `map_pub` 循环发布 → `pcl_render` 渲染 | `/map_generator/global_cloud`（world 帧） |
| **直接真机** | SCAN‑Planner 直连 LIO 话题 | 消费 `/cloud_registered` + `/Odometry` |

### 数据流

```
真机雷达+IMU  ──► FAST-LIO2 ──► /Odometry, /cloud_registered (world)
                                      │
                    ┌─────────────────┼─────────────────┐
                    ▼                 ▼                  ▼
              保存 PCD (离线)     ros2 launch          ros2 launch
              │                  is_real_world:=true    is_real_world:=true
              ▼                  (直连模式)            (直连模式)
          map_pub 循环发布        规划器+控制器        规划器+控制器
              │                  /cmd_vel→机器狗        /cmd_vel→机器狗
              ▼
    pcl_render_node 渲染 → grid_map → 规划器
```

---

## 2. 前置条件

### 2.1 传感器（室内场景）
- **LiDAR**：Livox Mid-360（90° 垂直 / 360° × 90° 覆盖）或 RoboSense Airy 等，正常广播点云话题
- **IMU**：外部/内置 IMU，≥ 100 Hz 更佳
- **FAST-LIO2 输出**：`/cloud_registered`（world 帧稠密点云）+ `/Odometry`（位姿）

### 2.2 工作区
```bash
# 均已提前编译
cd ~/rosws/fast_lio2_ros2 && colcon build   # FAST-LIO2（fast_lio + fast_lio_robosense）
cd ~/rosws/planner_ws && colcon build         # SCAN-Planner
```

若 Livox 驱动独立安装 → 先 `source <livox_ws>/install/setup.bash`。

---

## 3. 离线回放（推荐）：录制 PCD → 仿真器新地图

### 3.1 启动 FAST-LIO2 在线建图（Livox Mid-360）

```bash
cd ~/rosws/fast_lio2_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
# 可选：source <livox_ws>/install/setup.bash

# 终端 1：FAST-LIO2 建图 + RViz（加载 fastlio.rviz）
ros2 launch fast_lio mapping.launch.py \
  config_path:=<path_to>/mid360.yaml \
  rviz:=true

# 终端 2：Livox Mid-360 驱动
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

> 若使用 **RoboSense Airy**：改为 `ros2 launch fast_lio_robosense mapping_robosense_airy.launch.py`。

**验证输出正常**：
```bash
ros2 topic hz /Odometry             # 应 ≥ 10 Hz
ros2 topic hz /cloud_registered     # 应 ≥ 5 Hz，frame_id 为 "world"
```

> **室内注意**：FAST-LIO2 依赖 IMU 激励。如果初次启动漂移，先手持设备走动几秒钟让滤波器收敛，然后开始记录地图。

### 3.2 录制稠密点云为 .pcd

#### 方法 A：使用 pcl_ros 录制
```bash
ros2 run pcl_ros pointcloud_to_pcd input:=/cloud_registered
# 走动覆盖全场景后 Ctrl+C 停止，会在当前目录生成 .pcd 文件
```

> 建议采集 **1-2 分钟**，尤其是室内走廊、房间边界等重要区域。

#### 方法 B：使用 FAST-LIO2 内置保存（RoboSense 仅）
```bash
ros2 service call /map_save std_srvs/srv/Trigger
# 需要启动时加上 map_file_path:=/path/save.pcd
```

### 3.3 下采样 & 裁剪

```bash
# 体素滤波到 0.1 m（大幅加速仿真器渲染）
pcl_voxel_grid indoor_map.pcd indoor_down.pcd --leaf 0.1,0.1,0.1

# 可选：裁剪到有效区域
# pcl_crop_box indoor_down.pcd indoor_crop.pcd minX maxX minY maxY minZ maxZ
```

### 3.4 启动仿真 + PCD 地图

```bash
cd ~/rosws/planner_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch scan_planner run.launch.py \
  use_pcd_map:=true \
  pcd_map_file:=/absolute/path/to/indoor_down.pcd
```

此时 `map_pub` 会把这张 PCD 循环发布到 `/map_generator/global_cloud`，`pcl_render_node` 按机器人位姿渲染，grid_map 构建占用地图，规划器正常避障。

### 3.5 匹配机器人初始位姿

PCD 以 LIO 起始点为世界原点保存。设置机器人起点以匹配：
```bash
# 例如：若地图大体在 x ∈ [-8,2], y ∈ [-3,5]，
# 可把机器人放在地图通行区域：
ros2 launch scan_planner run.launch.py \
  use_pcd_map:=true pcd_map_file:=./indoor_down.pcd \
  init_x:=-5.0 init_y:=0.0 init_z:=0.3
```

---

## 4. 在线实时适配（需开发）

若希望 LIO 建图过程中**同时更新仿真世界**：
- 订阅 `/cloud_registered`（world 帧）及 `/Odometry`
- 内存内拼接 sweeps，定时下采样发布到 `/map_generator/global_cloud`
- 启动仿真 `use_pcd_map:=false`，替换 `map_pub` / `mockamap` 的数据源

> 此适配器需要单独编写，推荐先用离线模式（§3）验证有效性。

---

## 5. 实时真机模式（`is_real_world:=true`）

### 5.1 新增参数：覆写 LIO 话题名

`run.launch.py` 已暴露三个参数，直接对齐 FAST-LIO2 的输出话题，**无需 topic relay**：

| 参数 | 默认（`is_real_world:=true` 时） | FAST-LIO2 应设为 |
|---|---|---|
| `lio_odom_topic` | `/LIO/odom_vehicle` | `/Odometry` |
| `lio_cloud_topic` | `/LIO/clouds_lidar` | `/cloud_registered` |
| `lio_imu_topic` | `/LIO/odom_imu` | `/Odometry` |

### 5.2 完整导航（规划 + 控制 → 输出 `/cmd_vel`）

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  controller_mode:=closed_loop \
  lio_odom_topic:=/Odometry \
  lio_cloud_topic:=/cloud_registered \
  lio_imu_topic:=/Odometry \
  rviz:=true
```

### 5.3 仅可视化（禁用控制器，不输出 `/cmd_vel`）

```bash
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  controller_mode:=none \
  lio_odom_topic:=/Odometry \
  lio_cloud_topic:=/cloud_registered \
  lio_imu_topic:=/Odometry \
  rviz:=true
```

> `controller_mode:=none` 跳过 `closed_loop_controller` / `open_loop_controller` / `go2_kinematic_sim`。规划器照建地图，发目标后 A* / B‑样条轨迹正常输出，但不→ `/cmd_vel`。

### 5.4 快速可视化 RViz 配置

仓库提供了一个合并了 FAST-LIO2 与 SCAN‑Planner 显示项的 `.rviz` 文件：

```bash
rviz2 -d ~/rosws/planner_ws/src/SCAN-Planner/rviz/fastlio_scan.rviz
```

预载显示：LIO 点云 `/cloud_registered` + 占用地图 `/grid_map/occupancy_inflate` + 优化轨迹 `/optimal_list` + A* 路径 `/a_star_list` + 目标点 `/goal_point` + LIO 里程计 `/Odometry` + 运动路径 `/path`；Fixed Frame = `camera_init`，2D Goal Pose 话题 = `/move_base_simple/goal`。

### 5.5 地面滤波（去除地面点，避免污染占用地图）

FAST-LIO2 不对点云做地面分割，地面点进入 `grid_map` 的 raycast 概率融合后**地面会被标记为占用障碍**，挤占滑动地图容量、干扰 A* 与 B‑样条规划。

仓库自带了 Python 地面过滤节点 `src/ground_filter.py`，利用 Odometry 将点云转到 body 帧后裁掉低于阈值的点。

#### 启动过滤节点

```bash
source /opt/ros/humble/setup.bash
# 必须用系统 Python 并指定 ROS 2 路径（避开 conda Python 3.13 的不兼容）
export PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages

/usr/bin/python3 ~/rosws/planner_ws/src/ground_filter.py \
  --ros-args \
  -p ground_z_threshold:=-0.35 \
  -p input_cloud:=/cloud_registered \
  -p output_cloud:=/cloud_registered_filtered \
  -p input_odom:=/Odometry
```

#### 关键参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `ground_z_threshold` | -0.35 | body 帧 Z 裁剪值（m），低于此高度丢弃 |
| `input_cloud` | `/cloud_registered` | 输入点云话题 |
| `output_cloud` | `/cloud_registered_filtered` | 过滤后输出话题 |
| `input_odom` | `/Odometry` | 里程计话题（用于 world→body 变换） |

> **阈值选取**：Go2 站立时 body 中心离地 ~0.3‑0.4 m，地面点在 body 帧约 z=-0.35~-0.45。设 -0.35 保留机器狗腿高度的障碍（桌腿、椅腿），滤掉平坦地面。可改用 -0.50 更激进，或 -0.20 更保守。

#### 搭配 SCAN-Planner

```bash
# ※ 过滤节点已启动后，SCAN-Planner 订阅过滤后的点云
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  controller_mode:=none \
  lio_odom_topic:=/Odometry \
  lio_cloud_topic:=/cloud_registered_filtered \
  lio_imu_topic:=/Odometry \
  rviz:=false
```

> 这样 `grid_map` 收到的点云就已去除地面，占用地图中地板不再是障碍。

---

## 6. 坐标系 / 时间轴注意事项

### 6.1 FAST-LIO2 的世界坐标系

FAST-LIO2 输出 `/cloud_registered` 的 `frame_id` 为 **`camera_init`**，`/Odometry` 的 `child_frame_id` 为 **`body`**。实际运行时需加一条静态 TF：
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world camera_init
```
这样 `grid_map` 可透过 TF 树 `world → camera_init → body` 获得机器人位姿变换。

在 `is_real_world:=true` 模式下确认 frame 存在：
```bash
ros2 run tf2_ros tf2_echo world body
```

### 6.2 时间戳

LIO 话题应有正常时间戳（不能为 nan/0）。grid_map 点云已修复（`cloud_msg.header.stamp = node_->now()`）。

### 6.3 室内注意

- FAST-LIO2 在室内长走廊等特征稀少环境可能出现微小漂移，建图后**尽量保存全场景 PCD**，让仿真器拿到完整地图
- 窗口、玻璃等 LiDAR 无法感知的区域，PCD 中表现为空白，规划器看不到障碍

---

## 7. 验证清单

| 检查项 | 命令 / 预期 |
|---|---|
| FAST-LIO2 odom 正常 | `ros2 topic hz /Odometry` > 10 Hz |
| FAST-LIO2 点云正常 | `ros2 topic hz /cloud_registered` ≥ 5 Hz，frame = world |
| TF 存在（真机模式） | `ros2 run tf2_ros tf2_echo world body` 有输出 |
| PCD 导入成功 (离线回放) | `ros2 topic echo --once --no-arr /map_generator/global_cloud` 非空 |
| 渲染点云存在 (离线回放) | `ros2 topic hz /pcl_render_node/cloud` ~10 Hz |
| grid_map 工作 | `ros2 topic hz /grid_map/occupancy_inflate` 有发布 |
| FSM 正常 | `ros2 node list` 包含 `scan_planner_node` |
| 规划出轨迹 | `ros2 topic hz /planning/bspline` 有输出 |
| 话题 relay 已启动（真机模式） | `ros2 topic info /LIO/clouds_lidar` 显示 Publisher count ≥ 1 |

### 常见报错

| 现象 | 可能原因 | 解决 |
|---|---|---|
| 离线回放时点云不渲染 | PCD 文件过大 / 帧不对 | 下采样到 0.1 m 再导入 |
| `/LIO/clouds_lidar` 无数据 (真机) | SCAN‑Planner 直接连接，不启动仿真器 | 确认 `is_real_world:=true` |
| 真实机器人模式 grid_map 无更新 | relay 未启动 | 用 `ros2 run topic_tools relay` 桥接 |
| 地图瞬移 / 机器人来回跑 | 多套仿真残留 | `pkill -f pcl_render_node`, 只保留一个实例 |

---

## 8. 综合实战脚本

### 8.1 离线回放完整示例（室内 Mid-360）
```bash
# === 1) 启动 FAST-LIO2 + Livox 驱动 ===
cd ~/rosws/fast_lio2_ros2
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_path:=<path>/mid360.yaml &
# 另一终端启动 Livox： ros2 launch livox_ros_driver2 msg_MID360_launch.py

# === 2) 记录 PCD（覆盖室内全场景） ===
ros2 run pcl_ros pointcloud_to_pcd input:=/cloud_registered
# 全场景走一遍后 Ctrl+C

# === 3) 下采样 ===
pcl_voxel_grid *.pcd indoor.pcd --leaf 0.1,0.1,0.1

# === 4) 仿真 + 室内地图 ===
cd ~/rosws/planner_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch scan_planner run.launch.py \
  use_pcd_map:=true pcd_map_file:=./indoor.pcd \
  init_x:=0.0 init_y:=0.0 init_z:=0.3

# === 5) 下发目标，机器狗运动 ===
# RViz "2D Goal Pose" → 选目标点 → 规划 + 控制器驱动
```

### 8.2 实时真机模式完整示例（FAST-LIO2 + 仅可视化）
```bash
# === 终端 1: FAST-LIO2 建图（已有）===
# 已启动 /laser_mapping 节点，输出 /Odometry + /cloud_registered

# === 终端 2: SCAN-Planner（仅可视化，不输出 /cmd_vel）===
cd ~/rosws/planner_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

# 静态 TF（仅需首次运行一次）
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world camera_init &

# 启动规划器（viz-only）
ros2 launch scan_planner run.launch.py \
  is_real_world:=true \
  controller_mode:=none \
  lio_odom_topic:=/Odometry \
  lio_cloud_topic:=/cloud_registered \
  lio_imu_topic:=/Odometry \
  rviz:=false

# === 终端 3: RViz 可视化 ===
rviz2 -d ~/rosws/planner_ws/src/SCAN-Planner/rviz/fastlio_scan.rviz
# 使用 "2D Goal Pose" 工具点击目标 → 显示占用地图、A*/B-spline 轨迹
```

### 8.3 关闭命令

```bash
# 注意：以下命令可能被复制到终端环境中执行，请确认无误后再运行

# 【仅关闭 SCAN-Planner（保留 FAST-LIO2 和 RViz）】
pkill -f scan_planner_node
pkill -f closed_loop_controller
pkill -f static_transform_publisher

# 【关闭所有（FAST-LIO2 + SCAN-Planner + RViz）】
pkill -f scan_planner_node
pkill -f closed_loop_controller
pkill -f static_transform_publisher
pkill -f laser_mapping
pkill -f rviz2

# 【仅关闭 RViz】
pkill -f rviz2
```

> `pkill -f <pattern>` 按进程命令行匹配，比 `kill <pid>` 方便。如需更精确只杀某个节点，先用 `ros2 node list` 查看节点名再用 `pkill`。

---

相关文档：[仿真器说明](scan_simulator.md) · [运行/排查](example.md) · [编译](scan_planner_compiled.md) · [ROS1→ROS2 迁移](ros2_migration_guide.md)。
