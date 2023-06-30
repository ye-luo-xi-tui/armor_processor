// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rm_msgs/TargetDetectionArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <rm_msgs/TrackData.h>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_processor/tracker.hpp"

namespace rm_auto_aim
{
using tf2_filter = tf2_ros::MessageFilter<rm_msgs::TargetDetectionArray>;
class ArmorProcessorNode
{
public:
  explicit ArmorProcessorNode(ros::NodeHandle& nh);

private:
  void armorsCallback(const rm_msgs::TargetDetectionArray::ConstPtr& msg);

  void publishMarkers(const rm_msgs::TrackData & track_data);

  // The time when the last message was received
  ros::Time last_time_;
  double dt_;

  // Armor tracker
  std::unique_ptr<Tracker> tracker_;

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  tf2_ros::TransformBroadcaster br_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<rm_msgs::TargetDetectionArray> armors_sub_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  // Publisher
  ros::Publisher track_pub_;

  // Visualization marker publisher
  visualization_msgs::Marker position_marker_;
  visualization_msgs::Marker linear_v_marker_;
  visualization_msgs::Marker angular_v_marker_;
  visualization_msgs::Marker armors_marker_;
  ros::Publisher marker_pub_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
