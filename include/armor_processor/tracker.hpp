// Copyright 2022 Chen Jun

#pragma once

// Eigen
#include <Eigen/Eigen>

// ROS
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// STD
#include <memory>

#include "armor_processor/extended_kalman_filter.hpp"

namespace armor_processor
{
struct Armor
{
  int id;
  std::string type;
  double distance_to_image_center;
  tf2::Transform transform;
};
using Armors = std::vector<Armor>;
class Tracker
{
public:
  Tracker(double max_match_distance, int tracking_threshold, int lost_threshold);

  void init(const Armor* armor);

  void update(const Armor* armor);

  enum State
  {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
  } tracker_state;

  ExtendedKalmanFilter ekf;

  Armor tracked_armor;
  int tracked_id;
  int armors_num;
  Eigen::VectorXd target_state;

  // To store another pair of armors message
  double dz, another_r;

private:
  void initEKF(const Armor& a);

  void updateArmorsNum(const Armor& a);

  void handleArmorJump(const Armor& a);

  double orientationToYaw(const tf2::Quaternion& q);

  Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd& x);

  double max_match_distance_;

  int tracking_threshold_;
  int lost_threshold_;

  int detect_count_;
  int lost_count_;

  double last_yaw_;
};

}  // namespace armor_processor
