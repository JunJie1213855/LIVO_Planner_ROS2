# SCAN-Planner 仿真器说明（组件 · 启动流程 · 运行流程）

本文说明 SCAN-Planner 自带的 **MARSIM 风格仿真器**：它由哪些组件构成、怎么启动、运行时数据如何闭环流动。
目标：无需真机 / 真实 SLAM，即可在程序化生成的迷宫里跑通"感知 → 建图 → 规划 → 控制 → 机器人运动"整条链路。

- 一键启动：`ros2 launch scan_planner run.launch.py`（`is_real_world:=false` 时自动带起仿真器）
- 坐标系：全局统一为 **`world`**
- 相关文档：[运行/排查](example.md) · [编译](scan_planner_compiled.md)

---

## 1. 组件清单

仿真器 = "世界 + 传感器 + 机器人动力学"的模拟，让规划器像面对真机一样工作。

| 组件（ROS 2 包 / 可执行） | 角色 | 关键输入 | 关键输出 |
|---|---|---|---|
| **mockamap** (`mockamap_node`) | 程序化生成随机迷宫地图（3D 点云） | 参数（seed/尺寸/障碍数…） | `/map_generator/global_cloud`（全局静态地图，world 帧） |
| **map_generator** (`map_pub`) | 备选：从 `.pcd` 文件加载地图（`use_pcd_map:=true` 时用它替代 mockamap） | PCD 文件路径 | `/map_generator/global_cloud` |
| **local_sensing_node** (`pcl_render_node`) | **传感器仿真**：按机器人位姿对全局地图做 FOV/遮挡渲染（ikd-Tree + FOV_Checker） | 全局地图 + 机器人位姿 | `/pcl_render_node/cloud`（机器人视角局部点云,~10Hz）、`/pcl_render_node/depth`、传感器位姿 |
| **go2_kinematic_sim** | **机器人动力学仿真**：收速度指令，积分出位姿（运动学模型，非物理引擎） | `/cmd_vel`、`init_x/y/z` | `/quad_0/body_pose`（里程计,~100Hz）、`/quad_0/lidar_pose`、`/quad_0/camera_pose` |
| **go2_gait_publisher** | 腿部步态动画（关节角） | 机器人位姿 | `/joint_states`（供 rviz 显示腿部运动） |
| **odom_visualization** | 里程计 → 机器人 Marker（rviz 显示） | `/quad_0/body_pose` | 机器人网格 Marker + TF |
| **go2_description** | Go2 机器人 URDF/网格（`robot_state_publisher` 用） | — | `/robot_description` + 关节 TF |

> **消费仿真的"上层"（属规划器，不属仿真器，但同在 `run.launch.py` 里）：**
> `scan_planner_node`（把渲染点云建成滑动占用地图 + 规划）、`closed_loop_controller`（B样条轨迹 → `/cmd_vel`）。

**可选后端**：`local_sensing_node` 还有 GPU 版 `opengl_render_node`（`use_gpu:=true`），本次 ROS 2 迁移未启用，默认走 CPU 的 `pcl_render_node`。

---

## 2. 数据流架构（闭环）

```
        ┌─────────────────────────── 仿真器 ───────────────────────────┐
        │                                                              │
  mockamap ──/map_generator/global_cloud──►  pcl_render_node           │
 (生成迷宫)        (world 帧全局地图)         (按位姿渲染传感器视角)      │
        │                                        │                     │
        │                     /pcl_render_node/cloud (局部点云 ~10Hz)   │
        │                                        ▼                     │
        │                              ┌──────────────────┐            │
        │                              │  scan_planner_node │           │
        │        /quad_0/body_pose ───►│  grid_map: 点云→   │           │
        │        (里程计 ~100Hz)  ┌───►│  滑动占用地图      │           │
        │                        │     │  FSM: A*+B样条规划 │           │
        │                        │     └──────────────────┘            │
        │                        │              │ /planning/bspline    │
        │                        │              ▼                      │
        │                        │     closed_loop_controller           │
        │                        │       (轨迹跟踪)                      │
        │                        │              │ /cmd_vel              │
        │                        │              ▼                      │
        │                  go2_kinematic_sim ◄──┘                       │
        │                  (收 cmd_vel，积分出新位姿) ──┐                │
        │                        ▲                     │                │
        │                        └── 新 /quad_0/body_pose 回到渲染&规划 ─┘
        └──────────────────────────────────────────────────────────────┘
   目标输入: RViz “2D Goal Pose” → /move_base_simple/goal  (见 example.md §4)
   可视化:   odom_visualization + go2_gait_publisher + robot_state_publisher → RViz2
```

**闭环本质**：规划器出轨迹 → 控制器发 `/cmd_vel` → 运动学机器人更新位姿 → 传感器按新位姿重新渲染 → 占用地图更新 → 规划器再规划。里程计在纯仿真下**自足**（`go2_kinematic_sim` ⇄ `/cmd_vel`），无需外部 LIO。

---

## 3. 启动流程

`ros2 launch scan_planner run.launch.py` 内部分两层：

**① `run.launch.py`（顶层）** 启动：
1. `scan_planner_node` —— 规划器主节点（含 grid_map + FSM）
2. `closed_loop_controller` —— 轨迹跟踪（`controller_mode:=closed_loop` 时）
3. `go2_kinematic_sim` —— 运动学机器人（仅 `is_real_world:=false`），接收 `init_x/y/z` 作初始位姿
4. `go2_gait_publisher` —— 步态
5. `go2_robot_state_publisher` —— 加载 Go2 URDF
6. `rviz2` —— 加载 `example.rviz`（`rviz:=true` 默认）
7. **include `simulator.launch.py`**（仅 `is_real_world:=false`）

**② `simulator.launch.py`（被包含）** 启动：
1. `mockamap_node` —— 当 `use_pcd_map:=false`（默认），生成随机迷宫，`/mock_map` 重映射到 `/map_generator/global_cloud`
2. `map_pub` —— 当 `use_pcd_map:=true`，改为加载 PCD 地图
3. `odom_visualization`
4. `pcl_render_node` —— CPU 传感器渲染（`use_gpu:=false` 默认）；`~global_map` 重映射到 `/map_generator/global_cloud`

启动后各节点按话题自动连通，无固定先后依赖（DDS 发现）。

---

## 4. 运行流程（一次导航）

1. **建世界**：`mockamap_node` 生成迷宫点云，发布到 `/map_generator/global_cloud`（world 帧，静态/低频刷新）。
2. **初始位姿**：`go2_kinematic_sim` 从 `init_x/y/z`（默认 `-19,1,0.3`）起，持续发布 `/quad_0/body_pose`。
3. **传感器渲染**：`pcl_render_node` 用「全局地图 + 当前位姿」渲染出机器人视角的局部点云 `/pcl_render_node/cloud`（~10Hz，含 FOV 与遮挡）。
4. **建占用地图**：`scan_planner_node` 的 `grid_map` 订阅局部点云 + 位姿，raycast + 概率融合 + 膨胀，得到机器人中心**滑动占用地图** `/grid_map/occupancy_inflate`。
5. **下发目标**：RViz 用 “2D Goal Pose” 点选（→ `/move_base_simple/goal`），FSM 进入 `EXEC_TRAJ`。
6. **规划**：FSM 前端动态 A* 搜索 + 后端 B 样条优化，发布轨迹 `/planning/bspline`。
7. **跟踪**：`closed_loop_controller` 收轨迹 + 里程计，算出 `/cmd_vel`（~120Hz）。
8. **运动 + 闭环**：`go2_kinematic_sim` 收 `/cmd_vel` 积分出新位姿 → 回到第 3 步（重新渲染、重新规划），直到到达目标，FSM 回 `WAIT_TARGET`。

---

## 5. 关键话题与坐标系

| 话题 | 类型 | 发布者 | 说明 |
|---|---|---|---|
| `/map_generator/global_cloud` | PointCloud2 | mockamap / map_pub | 全局静态地图（迷宫） |
| `/pcl_render_node/cloud` | PointCloud2 | pcl_render_node | 机器人视角渲染点云（传感器仿真输出） |
| `/quad_0/body_pose` | Odometry | go2_kinematic_sim | 机器人里程计（纯仿真下自足） |
| `/quad_0/lidar_pose`、`/quad_0/camera_pose` | Odometry | pcl_render_node/sim | 传感器位姿 |
| `/grid_map/occupancy_inflate` | PointCloud2 | scan_planner_node | 膨胀占用地图（规划器所见障碍） |
| `/planning/bspline` | scan_planner_msgs/Bspline | scan_planner_node | 规划轨迹 |
| `/cmd_vel` | Twist | closed_loop_controller | 速度指令 |
| `/move_base_simple/goal` | PoseStamped | RViz | 导航目标输入 |

坐标系统一为 **`world`**；`grid_map` 另发布一个滑动地图子帧（TF `world → sliding_map`）。

---

## 6. 可配置项（常用）

| 参数 | 默认 | 说明 |
|---|---|---|
| `is_real_world` | false | true=接真机/LIO（不启动仿真器，消费 `/LIO/*`）；false=仿真 |
| `use_pcd_map` | false | false=mockamap 随机迷宫；true=map_generator 加载 `.pcd` |
| `sensor_type` | lidar | `lidar`（MID360 类）/ `depth`（RealSense 类），影响渲染与传感器位姿话题 |
| `use_gpu` | false | true=OpenGL `opengl_render_node`（未迁移，保持 false） |
| `init_x/y/z` | -19/1/0.3 | 机器人初始位姿 |
| `map_size_x/y/z` | 40/40/5 | 地图尺寸 (m) |

mockamap 迷宫的形态由 `simulator.launch.py` 里的 mockamap 参数决定（`seed`、`type`、`obstacle_number`、`width/height_min/max` 等）。

---

## 8. 使用真机点云替代 mockamap 地图

有两种方式把真机雷达/LIO 点云传入仿真器作为"新世界"：

### 8.1 离线回放 （推荐，无需改码） 

**思路**：真机录一帧 .pcd（或积累多帧融合的稠密点云）→ `map_pub` 逐帧发布 → `pcl_render_node` 以此渲染感官 → 规划器工作。

1. **把 LIO `/LIO/clouds_lidar` 积累点云录成 .pcd**
   ```bash
   # 方法 A：pcl_ros
   ros2 run pcl_ros pointcloud_to_pcd input:=/LIO/clouds_lidar
   # 方法 B：用 ros2 topic echo 采集若干帧，程序写入 PCD（需一点脚本）。
   ```
2. **下采样 / 裁剪**（可选，控制仿真地图分辨率）
   ```bash
   pcl_voxel_grid input.pcd output.pcd --leaf 0.1,0.1,0.1
   ```
3. **放到 `resources/`，启动仿真并声明 PCD 路径**
   ```bash
   ros2 launch scan_planner run.launch.py use_pcd_map:=true pcd_map_file:=/absolute/path/to/your_map.pcd
   ```
   启动后 `map_pub` 将该 .pcd 作为 `/map_generator/global_cloud` 发布，`pcl_render_node` 立刻渲染→规划器工作。

> 注意：`use_pcd_map:=true` 会跳过 `mockamap_node`，但 `map_pub` 会按 `publish_rate` 参数重复发布（默认可 0.2 Hz），这对静态 .pcd 足够。

### 8.2 在线实时 — Live Adapter（需要开发）

若你希望 **LIO 建图 → 实时更新仿真世界**（比如增量拼接 LiDAR sweeps 到一个不断更新的全局地图），可以写一个在线适配节点：

- 订阅 `/LIO/clouds_lidar`（world 帧稠密点云，已去畸配准）以及 `/LIO/odom_vehicle`（位姿）
- 在内存内用点云拼接算法（或简单 frame 积累）合成全局地图
- 周期性下采样并发布到 `/map_generator/global_cloud`
- 替代 `map_pub` / `mockamap_node`，直接启动仿真执行 `use_pcd_map:=false`，但**替换数据源**，保持仿真闭环。

> 此方案需要编写新的 ROS 2 节点，不属于现成功能。推荐先试 8.1 方案验证适用性，再决定是否投入在线适配开发。

### 8.3 真机 LIO 作为实时仿真输入（`is_real_world:=true` 旁路）

若直接把真机 LIO 当仿真数据源跑规划器，**不需要仿真器生成地图**，直接启动：
```bash
# 真机 LIO 发布 /LIO/clouds_lidar, /LIO/odom_vehicle 等
ros2 launch scan_planner run.launch.py is_real_world:=true
```
此模式`scan_planner_node`**直接消费** LIO 活数据建占用地图并规划，连 `pcl_render_node` 也被旁路（无仿真器）。本质上等于"真机部署模式"。详见`示例` 示例中的 §7。

> 附注：三种方式适用不同场景：
> - 离线回放（8.1）: 回放已测地图，离线调试
> - 在线实时（8.2）: 始终用最新 LIO 数据建仿真世界
> - 直接真机（8.3）: 旁路整个仿真器，直接跑规划

---

## 9. 常见问题

- **地图 / 机器狗"来回瞬移"**：多半是**同时跑了多套仿真**（多个 `go2_kinematic_sim`/`mockamap`/`pcl_render_node` 抢发同名话题、多个 TF 广播者）。先清残留再只起一套：
  ```bash
  pkill -f scan_planner_node; pkill -f go2_kinematic_sim; pkill -f pcl_render_node; pkill -f mockamap_node
  ```
  并强烈建议给仿真单开一个 `ROS_DOMAIN_ID`（如 `export ROS_DOMAIN_ID=42`，launch 与 rviz 终端都设），避开机器上其它 ROS 栈（gazebo/nav2/其它 sim）的同名话题。
- **占用地图导航后从 RViz 消失**：`grid_map` 点云曾缺 `header.stamp`，已修复。详见 [example.md §8](example.md)。
- **`a star error, force return!`**：规划器个别 replan 子过程的正常回退提示，不影响到达目标。

---

相关文档：[编译](scan_planner_compiled.md) · [运行/发目标/排查](example.md) · [ROS1→ROS2 迁移](ros2_migration_guide.md)。
