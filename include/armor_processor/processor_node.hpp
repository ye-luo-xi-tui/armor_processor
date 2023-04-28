// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <visualization_msgs/MarkerArray.h>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_processor/tracker.hpp"

namespace rm_auto_aim
{
using tf2_filter = tf2_ros::MessageFilter<apriltag_ros::AprilTagDetectionArray>;
struct Target
{
  std_msgs::Header header;
  std::string id;
  bool tracking;
  geometry_msgs::Point position;
  double yaw;
  geometry_msgs::Vector3 velocity;
  double v_yaw;
  double radius_1;
  double radius_2;
  double z_2;
};
class ArmorProcessorNode
{
public:
  explicit ArmorProcessorNode(ros::NodeHandle& nh);

private:
  void armorsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

  void publishMarkers(const Target & target);

  // The time when the last message was received
  ros::Time last_time_;
  double dt_;

  // Armor tracker
  std::unique_ptr<Tracker> tracker_;

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<apriltag_ros::AprilTagDetectionArray> armors_sub_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  // Publisher
  ros::Publisher target_pub_;

  // Visualization marker publisher
  visualization_msgs::Marker position_marker_;
  visualization_msgs::Marker linear_v_marker_;
  visualization_msgs::Marker angular_v_marker_;
  visualization_msgs::Marker armors_marker_;
  ros::Publisher marker_pub_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
