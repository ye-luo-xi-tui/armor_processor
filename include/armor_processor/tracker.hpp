// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

// STD
#include <memory>

#include "armor_processor/extended_kalman_filter.hpp"
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <rm_msgs/TrackData.h>

namespace rm_auto_aim
{
struct Armor
{
  int id;
  geometry_msgs::Pose pose;
};
using Armors = std::vector<Armor>;
class Tracker
{
public:
  Tracker(double max_match_distance, int tracking_threshold, int lost_threshold);

  void init(const Armors & armors_msg);

  void update(const Armors & armors_msg);

  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
  } tracker_state;

  ExtendedKalmanFilter ekf;

  Armor tracked_armor;
  int tracked_id;
  Eigen::VectorXd target_state;

  // To store another pair of armors message
  double last_z, last_r;

private:
  void initEKF(const Armor & a);

  void handleArmorJump(const Armor & a);

  double orientationToYaw(const geometry_msgs::Quaternion & q);

  Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

  double max_match_distance_;

  int tracking_threshold_;
  int lost_threshold_;

  int detect_count_;
  int lost_count_;

  double last_yaw_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
