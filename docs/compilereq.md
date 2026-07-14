# SCAN-Planner 编译环境要求

> 本文档说明编译 **SCAN-Planner**（`src/SCAN-Planner`）所需的系统环境与依赖。
> 该项目是基于 **ROS 1 / catkin** 的四足机器人（Unitree Go2）局部规划器 + 仿真器。

## 1. 官方推荐环境

| 项目 | 要求 |
|------|------|
| 操作系统 | **Ubuntu 20.04**（README "Tested on" 标注） |
| ROS 版本 | **ROS Noetic**（ROS 1，非 ROS 2） |
| 构建工具 | `catkin_make`（catkin，非 colcon/ament） |
| 编译器 | GCC ≥ 9（Ubuntu 20.04 自带即可） |
| CMake | ≥ 2.8.3 |
| C++ 标准 | C++11 / C++14 |

> ⚠️ 本项目是 catkin/roscpp 代码，**不能直接在 ROS 2 下编译**。
> Ubuntu 18.04 + ROS Melodic 理论可行（仅需 C++14），但未经作者测试，PCL/OpenCV 版本差异可能需微调。

## 2. 依赖列表

| 依赖 | 用途 | 安装方式 |
|------|------|----------|
| ROS Noetic (desktop-full) | 提供 roscpp / tf / pcl_ros / cv_bridge / rviz | 官方源安装 |
| **Armadillo** | 仿真器必需（pose_utils / odom_visualization） | `sudo apt-get install libarmadillo-dev` |
| Eigen3 | 矩阵运算 | 系统自带或 `libeigen3-dev` |
| PCL ≥ 1.7 | 点云处理 | 随 desktop-full 安装 |
| OpenCV + cv_bridge | 深度图渲染 | 随 desktop-full 安装 |
| Boost (filesystem/iostreams/program_options/system/serialization) | 仿真器 | 系统自带 |

### 可选：GPU / OpenGL 渲染后端

`local_sensing` 提供 CPU 版（`pcl_render_node`，**默认**）和 GPU 版（`opengl_render_node`）。
仅在需要 GPU 渲染时安装以下依赖：

```bash
sudo apt-get install libglew-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
```

> glm / glad / ikd-Tree / glfw 已随仓库捆绑在 `src/simulator/local_sensing/` 内。

## 3. 编译步骤

```bash
# 步骤 1：安装仿真器必需依赖
sudo apt-get install libarmadillo-dev

# 步骤 2：进入工作区并编译
cd ~/rosws/planner_ws
catkin_make                    # 默认 CPU 版

# 或启用 GPU 渲染后端
catkin_make -DUSE_GPU=ON
```

`simulator.xml` 中的 `use_gpu` 选项决定启动哪个渲染节点。

## 4. 运行

```bash
# 终端 1：启动 RViz
source devel/setup.bash && roslaunch scan_planner rviz.launch

# 终端 2：启动算法
source devel/setup.bash && roslaunch scan_planner run.launch
```

## 5. 当前工作区注意事项

本工作区目录多套了一层：

```
planner_ws/src/SCAN-Planner/src/{planner,simulator}   ← ROS 包在此
```

`catkin_make` 会从工作区 `src/` 递归查找 `package.xml`，因此**能正常识别并编译**这些包。
若遇到路径相关问题，可将 `SCAN-Planner/src/` 下的 `planner`、`simulator` 目录直接移动到 `planner_ws/src/` 下。

## 6. 包含的 ROS 包（共 12 个）

**规划核心**（`src/planner/`）：`plan_env`、`path_searching`、`bspline_opt`、`traj_utils`、`plan_manage`(scan_planner)

**仿真器**（`src/simulator/`）：`local_sensing`、`map_generator`、`mockamap`、`go2_description`、`odom_visualization`、`pose_utils`、`waypoint_generator`
