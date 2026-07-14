#include "bspline_opt/uniform_bspline.h"
#include "scan_planner_msgs/msg/bspline.hpp"

#include <Eigen/Eigen>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

using scan_planner::UniformBspline;

namespace
{
rclcpp::Node::SharedPtr g_node;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

bool receive_traj = false;
std::vector<UniformBspline> traj;
double traj_duration = 0.0;
rclcpp::Time start_time;
int traj_id = 0;

std::string frame_id;
std::string child_frame_id;
double last_yaw = 0.0;
double yaw_min_speed = 0.05;
bool hold_final_position = true;
Eigen::Vector3d current_pos = Eigen::Vector3d::Zero();
Eigen::Vector3d current_vel = Eigen::Vector3d::Zero();

void publishState(const rclcpp::Time& stamp, const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
                  double yaw, double yaw_rate)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = frame_id;
  odom.child_frame_id = child_frame_id;

  odom.pose.pose.position.x = pos.x();
  odom.pose.pose.position.y = pos.y();
  odom.pose.pose.position.z = pos.z();
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  odom.pose.pose.orientation = tf2::toMsg(q);

  odom.twist.twist.linear.x = vel.x();
  odom.twist.twist.linear.y = vel.y();
  odom.twist.twist.linear.z = vel.z();
  odom.twist.twist.angular.z = yaw_rate;

  odom_pub->publish(odom);
}

bool parseBspline(const scan_planner_msgs::msg::Bspline::ConstSharedPtr& msg, UniformBspline& pos_traj)
{
  if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
  {
    RCLCPP_WARN(g_node->get_logger(), "[open_loop_controller] Ignore invalid bspline.");
    return false;
  }

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
    knots(i) = msg->knots[i];

  pos_traj = UniformBspline(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);
  return true;
}

void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
{
  UniformBspline pos_traj;
  if (!parseBspline(msg, pos_traj))
    return;

  traj.clear();
  traj.push_back(pos_traj);
  traj.push_back(traj[0].getDerivative());
  traj.push_back(traj[1].getDerivative());

  start_time = rclcpp::Time(msg->start_time);
  traj_id = msg->traj_id;
  traj_duration = traj[0].getTimeSum();
  receive_traj = true;

  RCLCPP_INFO(g_node->get_logger(), "[open_loop_controller] Receive traj %d, duration %.3fs.", traj_id, traj_duration);
}

void publishOdom()
{
  const rclcpp::Time now = g_node->now();

  if (!receive_traj)
  {
    publishState(now, current_pos, current_vel, last_yaw, 0.0);
    return;
  }

  const double elapsed = (now - start_time).seconds();

  if (elapsed < 0.0)
  {
    publishState(now, current_pos, current_vel, last_yaw, 0.0);
    return;
  }
  if (!hold_final_position && elapsed > traj_duration)
    return;

  const double t = std::min(std::max(elapsed, 0.0), traj_duration);
  Eigen::Vector3d pos = traj[0].evaluateDeBoorT(t);
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();

  if (elapsed <= traj_duration)
  {
    vel = traj[1].evaluateDeBoorT(t);
    acc = traj[2].evaluateDeBoorT(t);
  }

  const double horizontal_speed = std::hypot(vel.x(), vel.y());
  if (horizontal_speed > yaw_min_speed)
    last_yaw = std::atan2(vel.y(), vel.x());

  const double yaw_rate = horizontal_speed > yaw_min_speed
                              ? (vel.x() * acc.y() - vel.y() * acc.x()) /
                                    std::max(horizontal_speed * horizontal_speed, 1e-6)
                              : 0.0;

  current_pos = pos;
  current_vel = vel;
  publishState(now, pos, vel, last_yaw, yaw_rate);
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("open_loop_controller");

  std::string bspline_topic;
  std::string body_pose_topic;
  double publish_rate;
  double init_x;
  double init_y;
  double init_z;

  auto get_param = [&](const std::string& name, auto& val, auto def) {
    if (!g_node->has_parameter(name))
      g_node->declare_parameter(name, def);
    g_node->get_parameter(name, val);
  };

  get_param("bspline_topic", bspline_topic, std::string("/planning/bspline"));
  get_param("body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
  get_param("frame_id", frame_id, std::string("world"));
  get_param("child_frame_id", child_frame_id, std::string("quadruped"));
  get_param("publish_rate", publish_rate, 100.0);
  get_param("yaw_min_speed", yaw_min_speed, 0.05);
  get_param("hold_final_position", hold_final_position, true);
  get_param("init_yaw", last_yaw, 0.0);
  get_param("init_x", init_x, 0.0);
  get_param("init_y", init_y, 0.0);
  get_param("init_z", init_z, 0.3);
  current_pos = Eigen::Vector3d(init_x, init_y, init_z);

  odom_pub = g_node->create_publisher<nav_msgs::msg::Odometry>(body_pose_topic, 20);
  auto bspline_sub =
      g_node->create_subscription<scan_planner_msgs::msg::Bspline>(bspline_topic, 10, bsplineCallback);

  auto timer = g_node->create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate), publishOdom);

  RCLCPP_WARN(g_node->get_logger(), "[open_loop_controller] ready. bspline: %s, odom: %s", bspline_topic.c_str(),
              body_pose_topic.c_str());

  rclcpp::spin(g_node);
  rclcpp::shutdown();
  return 0;
}
