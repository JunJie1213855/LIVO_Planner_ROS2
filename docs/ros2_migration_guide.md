# SCAN-Planner ROS 1 → ROS 2 Humble 迁移手册

本手册是协调 agent 与执行 agent 共享的迁移规范。目标：让核心规划器 5 个包在 ROS 2 Humble 下 `colcon build` 通过。

工作区根：`/home/ros/rosws/planner_ws`
包位置：`src/SCAN-Planner/src/planner/{plan_env,path_searching,bspline_opt,traj_utils,plan_manage}`
构建：`colcon build --packages-select <pkg>`（每次先 `source /opt/ros/humble/setup.bash`，若 `install/` 存在再 `source install/setup.bash`）

## 依赖构建顺序
`plan_env` → `path_searching` → `bspline_opt` → `traj_utils` → `scan_planner_msgs` → `plan_manage(scan_planner)`

---

## 1. package.xml
- `<package format="2">` → `<package format="3">`
- `<buildtool_depend>catkin</buildtool_depend>` → `<buildtool_depend>ament_cmake</buildtool_depend>`
- 结尾 `<export>` 内加：`<build_type>ament_cmake</build_type>`
- 依赖名替换：`roscpp`→`rclcpp`；`tf`→`tf2` `tf2_ros` `tf2_geometry_msgs` `tf2_eigen`；`rospy` 删除
- `message_generation`/`message_runtime` → 仅消息包用 `rosidl_default_generators`(build)/`rosidl_default_runtime`(exec) + `<member_of_group>rosidl_interface_packages</member_of_group>`
- 保留/新增：`std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs cv_bridge pcl_ros pcl_conversions message_filters`
- 内部包依赖用 `<depend>plan_env</depend>` 等

## 2. CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.8)
project(<pkg>)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)   # Humble 要求 C++17
endif()
add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# ... 每个依赖单独 find_package ...
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(<pkg> src/xxx.cpp ...)
ament_target_dependencies(<pkg> rclcpp std_msgs ... plan_env)   # ROS 依赖走这里
target_link_libraries(<pkg> ${PCL_LIBRARIES})                    # 非 ament 库走这里

install(TARGETS <pkg> EXPORT export_<pkg>
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(<pkg>)
ament_export_dependencies(rclcpp std_msgs ... plan_env Eigen3 PCL)
ament_package()
```
- 删除所有 `${catkin_INCLUDE_DIRS}` `${catkin_LIBRARIES}` `catkin_package(...)` `find_package(catkin ...)`
- 可执行文件：`add_executable` + `ament_target_dependencies` + `install(TARGETS <node> DESTINATION lib/${PROJECT_NAME})`

## 3. C++ 源码 API 对照
| ROS 1 | ROS 2 Humble |
|---|---|
| `#include <ros/ros.h>` | `#include <rclcpp/rclcpp.hpp>` |
| `#include <nav_msgs/Odometry.h>` | `#include <nav_msgs/msg/odometry.hpp>`（驼峰转下划线, 加 `/msg/`, `.h`→`.hpp`） |
| `nav_msgs::Odometry` | `nav_msgs::msg::Odometry` |
| `T::ConstPtr` / `T::Ptr` | `T::ConstSharedPtr` / `T::SharedPtr` |
| `ros::NodeHandle nh` | 类继承 `rclcpp::Node` 或持有 `rclcpp::Node::SharedPtr node` |
| `nh.advertise<T>("t",10)` | `node->create_publisher<T>("t",10)`，类型 `rclcpp::Publisher<T>::SharedPtr` |
| `nh.subscribe("t",10,&C::cb,this)` | `node->create_subscription<T>("t",10,std::bind(&C::cb,this,_1))` |
| 回调 `const T::ConstPtr& m` | `const T::ConstSharedPtr m` |
| `nh.createTimer(ros::Duration(d),&C::cb,this)` | `node->create_wall_timer(Xms,std::bind(&C::cb,this))`（回调无 TimerEvent 参数） |
| `ROS_INFO(...)` | `RCLCPP_INFO(node->get_logger(),...)`（STREAM/WARN/ERROR 同理） |
| `nh.param("n",v,def)` | `node->declare_parameter("n",def); node->get_parameter("n",v);` |
| `ros::Time::now()` | `node->now()` 或 `rclcpp::Clock().now()` |
| `.toSec()` | `.seconds()` |
| `ros::Duration(x)` | `rclcpp::Duration::from_seconds(x)` |
| `ros::Rate r(hz)` | `rclcpp::Rate r(hz)` |
| `ros::init(argc,argv,"n")` | `rclcpp::init(argc,argv)` |
| `ros::spin()` | `rclcpp::spin(node)` |
| `ros::ok()` / `ros::shutdown()` | `rclcpp::ok()` / `rclcpp::shutdown()` |
| `tf::TransformListener` | `tf2_ros::Buffer` + `tf2_ros::TransformListener` |
| `pcl_conversions` | `#include <pcl_conversions/pcl_conversions.h>`（fromROSMsg/toROSMsg 头变化） |

## 4. main() 模板
```cpp
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<XxxClass>();   // 或 rclcpp::Node::make_shared("name")
  node->init();                               // 若初始化需 shared_from_this()，放在构造后单独调用
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```
注意：构造函数里不能用 `shared_from_this()`；需要节点共享指针的初始化拆到 `init()` 里，main 中构造后调用。

## 5. 消息包 scan_planner_msgs
```
scan_planner_msgs/
  package.xml   (format 3, build_type ament_cmake, rosidl_default_generators/runtime, member_of_group rosidl_interface_packages, depend std_msgs geometry_msgs)
  CMakeLists.txt(find rosidl_default_generators; rosidl_generate_interfaces(${PROJECT_NAME} "msg/Bspline.msg" "msg/DataDisp.msg" DEPENDENCIES std_msgs geometry_msgs))
  msg/Bspline.msg  msg/DataDisp.msg
```
下游用法：`find_package(scan_planner_msgs REQUIRED)`；C++ 端 `rosidl_get_typesupport_target` 或 `ament_target_dependencies(<t> scan_planner_msgs)`；include 改为 `scan_planner_msgs/msg/bspline.hpp`，类型 `scan_planner_msgs::msg::Bspline`。

## 6. 执行循环协议（两 agent）
- 执行 agent 每次只做**一个包**：改 package.xml + CMakeLists + 全部源码 → `colcon build --packages-select <pkg>` → 回报**完整 colcon 输出**（成功或全部报错）→ 停止等待。
- 缺依赖：**不要 sudo**，回报精确的 `ros-humble-*` apt 包名给协调 agent。
- 协调 agent：记录报错、决定修复或推进、向用户申请 sudo 安装、最终验收。
