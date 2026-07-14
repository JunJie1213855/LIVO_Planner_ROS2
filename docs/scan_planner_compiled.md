# SCAN-Planner (ROS 2 Humble) 编译文档

> 本文档说明如何在 **ROS 2 Humble** 下编译 **SCAN-Planner**（`src/SCAN-Planner`）。
> 项目已从原 ROS 1 / catkin 迁移到 **ROS 2 Humble / ament_cmake / colcon**（四足机器人 Unitree Go2 局部规划器 + MARSIM 仿真器）。
> 迁移细节见 `docs/ros2_migration_guide.md`，运行方法见 `docs/example.md`。

## 1. 环境要求

| 项目 | 要求 |
|------|------|
| 操作系统 | **Ubuntu 22.04**（ROS 2 Humble 官方目标平台） |
| ROS 版本 | **ROS 2 Humble** |
| 构建工具 | **colcon** + **ament_cmake**（非 catkin_make） |
| 编译器 | GCC ≥ 11（Ubuntu 22.04 自带） |
| CMake | ≥ 3.8 |
| C++ 标准 | **C++17** |

## 2. 依赖

> 本机（当前环境）经预检**全部已安装，无需额外 `apt install`**：
> ROS Humble 全套 + PCL 1.12 + OpenCV 4.5.4 + Eigen3 + Armadillo 10.8.2 均就绪。

若在**全新的 Humble 机器**上搭建，一次性安装：

```bash
sudo apt update
sudo apt install \
  ros-humble-desktop python3-colcon-common-extensions \
  ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-cv-bridge \
  ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs ros-humble-tf2-eigen \
  ros-humble-message-filters ros-humble-rosidl-default-generators \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-xacro \
  libarmadillo-dev libeigen3-dev libpcl-dev libopencv-dev
```

| 依赖 | 用途 |
|------|------|
| `ros-humble-desktop` | rclcpp / tf2 / rviz2 / sensor_msgs / nav_msgs / visualization_msgs 等 |
| `ros-humble-pcl-ros` / `pcl-conversions` | 点云与 ROS 消息互转 |
| `ros-humble-cv-bridge` | 深度图 ↔ OpenCV |
| `ros-humble-tf2*` | 坐标变换（替代 ROS 1 的 `tf`） |
| `ros-humble-message-filters` | grid_map 的多话题时间同步 |
| `ros-humble-rosidl-default-generators` | 自定义消息 `scan_planner_msgs`（Bspline / DataDisp）生成 |
| `robot-state-publisher` / `joint-state-publisher` / `xacro` | Go2 URDF 可视化（运行时） |
| **libarmadillo-dev** | `pose_utils` / `odom_visualization` 线性代数（仿真器必需） |
| libeigen3-dev / libpcl-dev / libopencv-dev | Eigen3 / PCL(≥1.7，实测 1.12) / OpenCV(4.x) |

### 可选：GPU / OpenGL 渲染后端（默认关闭，未随迁移启用）
`local_sensing_node` 提供 CPU 版 `pcl_render_node`（**默认**）与 GPU 版 `opengl_render_node`。GPU 版在本次 ROS 2 迁移中**暂未启用**（`USE_GPU=OFF`）。如需自行启用，先装：
```bash
sudo apt install libglew-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
```
> glm / glad / ikd-Tree / glfw 已随仓库捆绑在 `src/simulator/local_sensing/` 内。

## 3. 编译步骤

工作区根：`/home/ros/rosws/planner_ws`

### 3.1 完整编译（含仿真器，跑 demo 用这个）
```bash
cd /home/ros/rosws/planner_ws
source /opt/ros/humble/setup.bash
colcon build
```

### 3.2 只编核心规划器（不含仿真器）
```bash
cd /home/ros/rosws/planner_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to scan_planner
```

### 3.3 干净重编（排查问题时）
```bash
cd /home/ros/rosws/planner_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build
```

> 说明：完整 `colcon build` 会连带编译工作区内相邻的 `ego_planner` 包（非本项目），属正常现象。

编译成功后 `source install/setup.bash` 即可使用（运行见 `docs/example.md`）。

## 4. 工作区目录注意事项

本工作区的 ROS 包被多套了一层：
```
planner_ws/src/SCAN-Planner/src/{planner,simulator}   ← ROS 2 包在此
```
`colcon build` 会从工作区 `src/` 递归查找 `package.xml`，因此**能正常识别并编译**这些包，无需调整目录。

## 5. 包含的 ROS 2 包（共 13 个，均 ament_cmake）

**核心规划器**（`src/planner/`）：
`plan_env` → `path_searching` → `bspline_opt` → `traj_utils` → `scan_planner_msgs`(rosidl 自定义消息) → `scan_planner`(=plan_manage)

**仿真器**（`src/simulator/`）：
`pose_utils`、`go2_description`、`waypoint_generator`、`mockamap`、`map_generator`、`odom_visualization`、`local_sensing_node`

> 依赖构建顺序由 colcon 自动解析；核心链按上面箭头顺序，仿真器各包依赖 `pose_utils`/PCL 等。

## 6. 验证编译结果

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon list                                   # 应列出 13 个包（+ 相邻 ego_planner）
ros2 pkg executables scan_planner             # scan_planner_node + 4 个控制器/仿真可执行
ros2 interface list | grep scan_planner_msgs  # Bspline / DataDisp
```

实测：全量 `colcon build` **14 个包 0 失败**（13 个本项目包 + 1 个相邻 `ego_planner`），仅编译告警。
