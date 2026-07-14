#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

rclcpp::Node::SharedPtr g_node;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoints_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_vis_pub;
nav_msgs::msg::Odometry odom;
bool is_odom_ready = false;

void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  is_odom_ready = true;
  odom = *msg;
}

void publishWaypoints(const geometry_msgs::msg::PoseStamped& goal) {
  nav_msgs::msg::Path waypoints;
  waypoints.header.frame_id = "world";
  waypoints.header.stamp = g_node->now();
  waypoints.poses.push_back(goal);
  waypoints_pub->publish(waypoints);

  geometry_msgs::msg::PoseArray waypoints_vis;
  waypoints_vis.header = waypoints.header;
  if (is_odom_ready) {
    waypoints_vis.poses.push_back(odom.pose.pose);
  }
  waypoints_vis.poses.push_back(goal.pose);
  waypoints_vis_pub->publish(waypoints_vis);
}

void goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  if (msg->pose.position.z <= -0.1) {
    RCLCPP_WARN(g_node->get_logger(), "[waypoint_generator] invalid goal.");
    return;
  }

  publishWaypoints(*msg);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("waypoint_generator");

  std::string body_pose_topic;
  if (!g_node->has_parameter("body_pose_topic"))
    g_node->declare_parameter("body_pose_topic", std::string("/quad_0/body_pose"));
  g_node->get_parameter("body_pose_topic", body_pose_topic);

  auto odom_sub = g_node->create_subscription<nav_msgs::msg::Odometry>(body_pose_topic, 10, odomCallback);
  auto goal_sub = g_node->create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10, goalCallback);
  waypoints_pub = g_node->create_publisher<nav_msgs::msg::Path>("waypoints", 50);
  waypoints_vis_pub = g_node->create_publisher<geometry_msgs::msg::PoseArray>("waypoints_vis", 10);

  rclcpp::spin(g_node);
  rclcpp::shutdown();
  return 0;
}
