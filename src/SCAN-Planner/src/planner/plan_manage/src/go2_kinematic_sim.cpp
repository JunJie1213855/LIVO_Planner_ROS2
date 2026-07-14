#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace
{
constexpr double kMaxVYawLimit = 1.0;

rclcpp::Node::SharedPtr g_node;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
rclcpp::TimerBase::SharedPtr sim_timer;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

double x = 0.0;
double y = 0.0;
double z = 0.0;
double yaw = 0.0;

double vx_cmd = 0.0;
double vy_cmd = 0.0;
double vyaw_cmd = 0.0;
double vx_world = 0.0;
double vy_world = 0.0;

double max_vx = 0.8;
double max_vy = 0.5;
double max_vyaw = kMaxVYawLimit;
double cmd_timeout = 0.3;
double sim_rate = 100.0;
bool publish_tf = false;
std::string frame_id = "world";
std::string child_frame_id = "base";
std::string body_pose_topic = "/quad_0/body_pose";

rclcpp::Time last_cmd_time;
rclcpp::Time last_sim_time;

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

void loadParamWithFallback(const rclcpp::Node::SharedPtr &node, const std::string &private_name,
                           const std::string & /*fallback_name*/, double &value, double default_value)
{
  if (!node->has_parameter(private_name))
    node->declare_parameter(private_name, default_value);
  node->get_parameter(private_name, value);
}

void cmdCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  vx_cmd = clamp(msg->linear.x, -max_vx, max_vx);
  vy_cmd = clamp(msg->linear.y, -max_vy, max_vy);
  vyaw_cmd = clamp(msg->angular.z, -max_vyaw, max_vyaw);
  last_cmd_time = g_node->now();
}

void publishOdom(const rclcpp::Time &stamp)
{
  tf2::Quaternion tf_q;
  tf_q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion q = tf2::toMsg(tf_q);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = frame_id;
  odom.child_frame_id = child_frame_id;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = z;
  odom.pose.pose.orientation = q;
  odom.twist.twist.linear.x = vx_world;
  odom.twist.twist.linear.y = vy_world;
  odom.twist.twist.angular.z = vyaw_cmd;
  odom_pub->publish(odom);

  if (!publish_tf || tf_broadcaster == nullptr)
    return;

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = frame_id;
  tf_msg.child_frame_id = child_frame_id;
  tf_msg.transform.translation.x = x;
  tf_msg.transform.translation.y = y;
  tf_msg.transform.translation.z = z;
  tf_msg.transform.rotation = q;
  tf_broadcaster->sendTransform(tf_msg);
}

void simCallback()
{
  const rclcpp::Time now = g_node->now();
  double dt = (now - last_sim_time).seconds();
  last_sim_time = now;
  if (dt < 0.0 || dt > 0.2)
    dt = 0.0;

  double vx = vx_cmd;
  double vy = vy_cmd;
  double wz = vyaw_cmd;
  if ((now - last_cmd_time).seconds() > cmd_timeout)
  {
    vx = 0.0;
    vy = 0.0;
    wz = 0.0;
  }

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  vx_world = c * vx - s * vy;
  vy_world = s * vx + c * vy;

  x += vx_world * dt;
  y += vy_world * dt;
  yaw = normalizeAngle(yaw + wz * dt);

  publishOdom(now);
}
} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("go2_kinematic_sim");

  auto get_param = [&](const std::string &name, auto &val, auto def) {
    if (!g_node->has_parameter(name))
      g_node->declare_parameter(name, def);
    g_node->get_parameter(name, val);
  };

  get_param("body_pose_topic", body_pose_topic, std::string("/quad_0/body_pose"));
  get_param("init_x", x, 0.0);
  get_param("init_y", y, 0.0);
  get_param("init_z", z, 0.3);
  get_param("init_yaw", yaw, 0.0);
  loadParamWithFallback(g_node, "max_vx", "/closed_loop_controller/max_vx", max_vx, 0.8);
  loadParamWithFallback(g_node, "max_vy", "/closed_loop_controller/max_vy", max_vy, 0.5);
  loadParamWithFallback(g_node, "max_vyaw", "/closed_loop_controller/max_vyaw", max_vyaw, kMaxVYawLimit);
  if (max_vyaw > kMaxVYawLimit)
  {
    RCLCPP_WARN(g_node->get_logger(), "[Go2 kinematic sim] cap max_vyaw %.3f to %.3f rad/s.", max_vyaw, kMaxVYawLimit);
    max_vyaw = kMaxVYawLimit;
  }
  get_param("cmd_timeout", cmd_timeout, 0.3);
  get_param("sim_rate", sim_rate, 100.0);
  get_param("publish_tf", publish_tf, false);
  get_param("frame_id", frame_id, std::string("world"));
  get_param("child_frame_id", child_frame_id, std::string("base"));

  tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(g_node);

  odom_pub = g_node->create_publisher<nav_msgs::msg::Odometry>(body_pose_topic, 100);
  cmd_sub = g_node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 20, cmdCallback);

  last_cmd_time = g_node->now();
  last_sim_time = g_node->now();
  sim_timer = g_node->create_wall_timer(std::chrono::duration<double>(1.0 / sim_rate), simCallback);

  RCLCPP_WARN(g_node->get_logger(), "[Go2 kinematic sim] ready.");

  rclcpp::spin(g_node);
  rclcpp::shutdown();
  return 0;
}
