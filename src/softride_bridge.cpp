// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include "ros1_bridge/bridge.hpp"

rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub;

void DiagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr & ros1_msg)
{
  diagnostic_msgs::msg::DiagnosticArray ros2_msg;
  for (const auto& field : ros1_msg->status)
  {
    diagnostic_msgs::msg::DiagnosticStatus ros2_status_msg;
    ros2_status_msg.level = field.level;
    ros2_status_msg.name = field.name;
    ros2_status_msg.message = field.message;
    ros2_status_msg.hardware_id = field.hardware_id;
    ros2_msg.status.push_back(ros2_status_msg);
  }
  status_pub->publish(ros2_msg);
}

int main(int argc, char * argv[])
{
  // ROS 1 node
  ros::init(argc, argv, "ros2_bridge");
  ros::NodeHandle ros1_node;

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("ros1_bridge");

  // bridge to raw pointcloud2 data
  std::string ros1_points_topic_name = "/fused_points";
  std::string ros2_points_topic_name = "/raw_pointcloud2_data";
  std::string ros1_points_type_name = "sensor_msgs/PointCloud2";
  std::string ros2_points_type_name = "sensor_msgs/msg/PointCloud2";
  size_t points_queue_size = 1;

  auto points_handles = ros1_bridge::create_bridge_from_1_to_2(
    ros1_node, ros2_node, ros1_points_type_name, ros1_points_topic_name, points_queue_size, 
    ros2_points_type_name, ros2_points_topic_name, points_queue_size);

  status_pub = ros2_node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/status");
  ros::Subscriber daignostics_subscriber = ros1_node.subscribe("/diagnostics", 6000, DiagnosticsCallback);

  //bridge to nav_sat_fix data
  std::string ros1_nav_sat_fix_topic_name = "/exiguo/nav_sat_fix";
  std::string ros2_nav_sat_fix_topic_name = "/nav_sat_fix";
  std::string ros1_nav_sat_fix_type_name = "sensor_msgs/NavSatFix";
  std::string ros2_nav_sat_fix_type_name = "sensor_msgs/msg/NavSatFix";
  size_t nav_sat_fix_queue_size = 10;

  auto nav_sat_fix_handles = ros1_bridge::create_bridge_from_1_to_2(
    ros1_node, ros2_node, ros1_nav_sat_fix_type_name, ros1_nav_sat_fix_topic_name, nav_sat_fix_queue_size, 
    ros2_nav_sat_fix_type_name, ros2_nav_sat_fix_topic_name, nav_sat_fix_queue_size);

  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // ROS 2 spinning loop
  rclcpp::executors::SingleThreadedExecutor executor;
  while (ros1_node.ok() && rclcpp::ok()) {
    executor.spin_node_once(ros2_node, std::chrono::milliseconds(1000));
  }

  return 0;
}
