#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("map_pub");

  std::string file_name;
  if (argc > 1) {
    file_name = argv[1];
  }

  auto get_param = [&](const std::string& name, auto& val, auto def) {
    if (!node->has_parameter(name))
      node->declare_parameter(name, def);
    node->get_parameter(name, val);
  };

  get_param("file_name", file_name, file_name);

  std::string frame_id;
  std::string cloud_topic;
  double publish_rate;
  double downsample_res;
  double map_offset_x;
  double map_offset_y;
  double map_offset_z;
  get_param("frame_id", frame_id, std::string("world"));
  get_param("cloud_topic", cloud_topic, std::string("/map_generator/global_cloud"));
  get_param("publish_rate", publish_rate, 3.0);
  get_param("downsample_res", downsample_res, 0.0);
  get_param("map_offset_x", map_offset_x, 0.0);
  get_param("map_offset_y", map_offset_y, 0.0);
  get_param("map_offset_z", map_offset_z, 0.0);

  if (file_name.empty()) {
    RCLCPP_ERROR(node->get_logger(), "[map_pub] No PCD file specified. Pass it as an arg or set file_name.");
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, cloud) < 0) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "[map_pub] Failed to read PCD file: " << file_name);
    return 1;
  }

  std::vector<int> finite_indices;
  pcl::removeNaNFromPointCloud(cloud, cloud, finite_indices);

  if (downsample_res > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_sampler;
    voxel_sampler.setInputCloud(cloud.makeShared());
    voxel_sampler.setLeafSize(downsample_res, downsample_res, downsample_res);
    voxel_sampler.filter(cloud);
  }

  if (map_offset_x != 0.0 || map_offset_y != 0.0 || map_offset_z != 0.0) {
    for (auto& point : cloud.points) {
      point.x += map_offset_x;
      point.y += map_offset_y;
      point.z += map_offset_z;
    }
  }

  if (cloud.empty()) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "[map_pub] PCD file has no valid XYZ points: " << file_name);
    return 1;
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame_id;

  auto cloud_pub =
      node->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::QoS(10).transient_local());

  RCLCPP_INFO_STREAM(node->get_logger(), "[map_pub] Loaded " << cloud.points.size() << " points from " << file_name
                                          << ", publishing " << cloud_topic << " in frame "
                                          << frame_id);

  const double rate_hz = std::max(0.1, publish_rate);
  auto period = std::chrono::duration<double>(1.0 / rate_hz);
  auto timer = node->create_wall_timer(period, [&msg, cloud_pub, node]() {
    msg.header.stamp = node->now();
    cloud_pub->publish(msg);
  });

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
