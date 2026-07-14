# SCAN-Planner (ROS 2 Humble) 运行示例

本文给出**编译**、**启动 MARSIM 自带仿真**、**下发导航目标**的完整命令，均为实测可用。

- 环境：Ubuntu + ROS 2 **Humble**（已 `source /opt/ros/humble/setup.bash`）
- 工作区根：`/home/ros/rosws/planner_ws`
- 依赖：随系统已装（rclcpp / pcl_ros / cv_bridge / tf2 / Armadillo / PCL / OpenCV / Eigen3），无需额外 `apt install`

---

## 1. 编译

### 1.1 完整编译（含仿真器，跑 demo 用这个）
```bash
cd /home/ros/rosws/planner_ws
source /opt/ros/humble/setup.bash
colcon build
```

完整编译参考 ("compile")[scan_planner_compiled.md].

### 1.2 只编核心规划器（不含仿真器）
```bash
cd /home/ros/rosws/planner_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to scan_planner
```

### 1.3 干净重编（排查问题时）
```bash
cd /home/ros/rosws/planner_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build
```

> 说明：完整 `colcon build` 会连带编译工作区内相邻的 `ego_planner` 包（非本项目），属正常现象。

---

## 2. 启动仿真示例（MARSIM 迷宫）

**终端 1** —— 一条命令启动整套仿真 + 规划器 + RViz2：
```bash
cd /home/ros/rosws/planner_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch scan_planner run.launch.py
```

会启动 8 个仿真/规划节点，并**自动开启 RViz2**（加载 `example.rviz`，详见 §3；加 `rviz:=false` 可关闭）：
- `scan_planner_node` —— 规划器主节点（前端 A* + 后端 B 样条优化 + FSM）
- `mockamap_node` —— 程序化生成迷宫地图 → `/map_generator/global_cloud`
- `pcl_render_node` —— 从地图 + 机器人位姿渲染传感器点云 → `/pcl_render_node/cloud`
- `go2_kinematic_sim` —— 运动学机器人（收 `/cmd_vel`，发里程计 `/quad_0/body_pose`）
- `closed_loop_controller` —— 轨迹跟踪，发 `/cmd_vel`
- `go2_gait_publisher` / `odom_visualization` / `go2_robot_state_publisher` —— 步态/可视化/URDF
- `rviz2` —— 可视化（自动加载 `example.rviz`，Fixed Frame=`world`，显示项已就位）

### 常用 launch 参数（`参数:=值`）
| 参数 | 默认 | 说明 |
|---|---|---|
| `is_real_world` | `false` | `true`=真机话题(`/LIO/*`)，`false`=仿真 |
| `navi_mode` | `1` | `1`=2D 目标点，`2`=多层关键点，`3`=参考路径跟踪 |
| `sensor_type` | `lidar` | `lidar` / `depth` |
| `controller_mode` | `closed_loop` | `closed_loop` / `open_loop` |
| `max_vel` / `max_acc` | `0.75` / `0.5` | 速度 / 加速度上限 |
| `planning_horizon` | `7.5` | 局部规划视距 (m) |
| `init_x`/`init_y`/`init_z` | `-19` / `1` / `0.3` | 机器人初始位姿 |
| `map_size_x/y/z` | `40` / `40` / `5` | 地图尺寸 (m)，大致范围 x,y ∈ [-20, 20] |
| `use_gpu` | `false` | GPU 渲染（`opengl_render_node` 未迁移，保持 `false`） |
| `rviz` | `true` | 是否随启动自动开启 RViz2 并加载 `example.rviz` |

例：
```bash
ros2 launch scan_planner run.launch.py max_vel:=1.0 init_x:=-15.0
```

---

## 3. 可视化（RViz2）

**`run.launch.py` 默认会自动启动 RViz2 并加载**预配置好全部显示项的 `example.rviz`（源文件在 `src/SCAN-Planner/rviz/`，编译后装到包的 share 目录）——**无需再开终端、无需手动 Add**。启动时加 `rviz:=false` 可关闭。

如需单独/手动启动（例如用了 `rviz:=false`，或想开第二个 RViz）：
```bash
source /opt/ros/humble/setup.bash
source /home/ros/rosws/planner_ws/install/setup.bash
rviz2 -d $(ros2 pkg prefix scan_planner)/share/scan_planner/rviz/example.rviz
```

该配置已包含（Fixed Frame = `world`）：

| 话题 | 类型 | 显示 |
|---|---|---|
| `/pcl_render_node/cloud` | PointCloud2 | 传感器渲染点云（按 Z 高度着色，Best Effort） |
| `/grid_map/occupancy_inflate` | PointCloud2 | 膨胀障碍图（橙色 Boxes，Best Effort） |
| `/optimal_list` | Marker | 优化后的 B 样条轨迹 |
| `/a_star_list` | Marker | A* 前端路径 |
| `/goal_point` | Marker | 当前目标点 |
| `/self_inflation` | Marker | 机器人自膨胀（Transient Local） |
| `/quad_0/body_pose` | Odometry | 机器人位姿（绿色箭头） |
| `/robot_description` | RobotModel | Go2 模型（Transient Local） |

此外，配置里的 **“2D Goal Pose” 工具已预设发布到 `/move_base_simple/goal`**，可直接在图上点选目标（见 §4.1 方法 B）。

---

## 4. 下发导航目标

### 4.1 navi_mode=1（默认，2D 目标点）
FSM 订阅 **`/move_base_simple/goal`**（`geometry_msgs/PoseStamped`，`frame_id=world`，`z≈0.3`，x/y 在地图 ±20 范围内）。

**方法 A —— 命令行**（终端 3）：
```bash
source /opt/ros/humble/setup.bash
source /home/ros/rosws/planner_ws/install/setup.bash
ros2 topic pub -1 /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "world"}, pose: {position: {x: -8.0, y: 1.0, z: 0.3}, orientation: {w: 1.0}}}'
```

**方法 B —— RViz2 工具**：
用工具栏的 **“2D Goal Pose”** 在地图上点选目标。
> ✅ 使用本仓库的 `src/SCAN-Planner/rviz/example.rviz` 时，该工具已预设发布到 `/move_base_simple/goal`，可直接点选，无需额外设置。
> 若换用其他 rviz 配置：rviz2 的 “2D Goal Pose” 默认发到 `/goal_pose`，需在 **Tool Properties** 面板把其 Topic 改为 `/move_base_simple/goal`（否则规划器收不到目标）。

发目标后：FSM 走 `WAIT_TARGET → EXEC_TRAJ`，`/optimal_list` 出现轨迹，`/cmd_vel` 驱动机器人朝目标移动，到达后回到 `WAIT_TARGET`。

### 4.2 navi_mode=3（参考路径跟踪）
启动时加 `navi_mode:=3`，向 **`/initial_path`**（`nav_msgs/Path`，`frame_id=world`）发布参考路径，规划器局部避障跟踪。

### 4.3 navi_mode=2（多层关键点）
当前仍用 ROS1 的 `$(rospack find)` + `rosparam load` 机制，**尚未适配 ROS2**（遗留项，暂不可用）。

---

## 5. 检查运行状态
```bash
source /opt/ros/humble/setup.bash && source /home/ros/rosws/planner_ws/install/setup.bash
ros2 node list                            # 8 个仿真/规划节点 + rviz2
ros2 topic hz /pcl_render_node/cloud      # ~10 Hz  → 传感器渲染正常
ros2 topic hz /quad_0/body_pose           # ~100 Hz → 里程计正常
ros2 topic hz /cmd_vel                    # 发目标后有输出 → 控制器在驱动
ros2 topic echo --once /quad_0/body_pose | grep -A3 position   # 看机器人当前位置
```

---

## 6. 停止
在**终端 1**（`ros2 launch` 所在终端）按 `Ctrl+C`，会关闭本次启动的所有节点。

---

## 7. 真机 / 外部里程计模式（可选）
若用真机或外部 LIO（如 FAST-LIO2 / Elevator-LIO）提供 `/LIO/odom_vehicle`、`/LIO/odom_imu`、`/LIO/clouds_lidar`：
```bash
ros2 launch scan_planner run.launch.py is_real_world:=true
```
此模式不启动 MARSIM 仿真器，规划器直接消费 `/LIO/*` 话题，并向 `/cmd_vel` 输出控制指令。

---

## 8. 常见问题 / 排查

### 占用地图（occupancy）在 RViz 里导航后消失
- **现象**：初始化时能看到 `/grid_map/occupancy_inflate` 局部占用地图，机器人导航一会儿后地图从 RViz 消失（但用 `ros2 topic echo` 看话题其实仍在持续发布点云）。
- **根因**：`plan_env/grid_map.cpp` 的 `publishMap` / `publishMapInflate` / `publishUnknown` 发布点云时**未设置 `header.stamp`**（`pcl::toROSMsg` 拷来的 PCL 时间戳恒为 0）。当 ROS 域内存在 gazebo 等发布的 `/clock`、RViz 的 TF 缓冲区推进到真实时间后，其 tf2 MessageFilter 会把时间戳为 0 的点云判为“过旧”而丢弃 → 于是初始化能显示、跑一会儿就消失。（`publishDepthCloud` / `publishSlidingMapFrame` / `publishSlidingMapBBox` 迁移时已有戳，唯独这三个漏了。）
- **修复（已修复）**：三处发布前补 `cloud_msg.header.stamp = node_->now();`，重编 `plan_env` + `scan_planner`（静态库需重链）。占用地图现在导航全程持续显示。验证：
  ```bash
  # 时间戳应为真实时间（非 sec: 0）
  ros2 topic echo --once --no-arr /grid_map/occupancy_inflate | grep -A3 stamp
  ```

### 规划日志出现 `a star error, force return!`
规划器在个别 replan 子过程中 A* 未找到路径时的正常回退提示，会继续 replan 并到达目标，不影响整体导航。

### 发了目标机器人不动 / RViz 点选无反应
见 §4.1：RViz 的 “2D Goal Pose” 默认发布到 `/goal_pose`，而本项目订阅 `/move_base_simple/goal`。用本仓库的 `example.rviz` 已预设正确话题；否则改工具话题或改用命令行方法 A。

### 完整 `colcon build` 里 4 个仿真包报错（历史）
若看到 `pose_utils` / `mockamap` 等报 `catkin` 相关错误，说明用的是**未迁移**的旧代码；当前仓库这些包均已迁移到 ament_cmake，正常 `colcon build` 应 0 失败（14 个包，含相邻 `ego_planner`）。

---

相关文档：`docs/scan_planner_compiled.md`（ROS 2 编译文档 / 依赖 / 环境）、`docs/ros2_migration_guide.md`（ROS1→ROS2 迁移说明）、`docs/scan_sim.png`（仿真截图）、`src/SCAN-Planner/rviz/example.rviz`（RViz2 预配置）。
