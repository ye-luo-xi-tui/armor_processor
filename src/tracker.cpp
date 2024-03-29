// Copyright 2022 Chen Jun

#include "armor_processor/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
// STD
#include <cfloat>
#include <string>

namespace armor_processor
{
Tracker::Tracker(double max_match_distance, int tracking_threshold, int lost_threshold)
  : tracker_state(LOST)
  , tracked_id(0)
  , target_state(Eigen::VectorXd::Zero(9))
  , max_match_distance_(max_match_distance)
  , tracking_threshold_(tracking_threshold)
  , lost_threshold_(lost_threshold)
{
}

void Tracker::init(const Armor* armor)
{
  if (armor == nullptr)
    return;
  tracked_armor = *armor;

  initEKF(tracked_armor);

  tracked_id = tracked_armor.id;
  tracker_state = DETECTING;

  updateArmorsNum(tracked_armor);
}

void Tracker::update(const Armor* armor)
{
  // KF predict
  Eigen::VectorXd ekf_prediction = ekf.predict();

  bool matched = false;
  // Use KF prediction as default target state if no matched armor is found
  target_state = ekf_prediction;

  if (armor != nullptr)
  {
    auto predicted_position = getArmorPositionFromState(ekf_prediction);
    auto p = armor->transform.getOrigin();
    Eigen::Vector3d position_vec(p.x(), p.y(), p.z());
    // Difference of the current armor position and tracked armor's predicted position
    double position_diff = (predicted_position - position_vec).norm();
    tracked_armor = *armor;

    if (position_diff < max_match_distance_)
    {
      // Matching armor found
      matched = true;
      p = tracked_armor.transform.getOrigin();
      // Update EKF
      double measured_yaw = orientationToYaw(tracked_armor.transform.getRotation());
      Eigen::Vector4d z(p.x(), p.y(), p.z(), measured_yaw);
      target_state = ekf.update(z);
    }
    else
    {
      // Check if there is same id armor in current frame
      if (armor->id == tracked_id)
      {
        // Armor jump happens
        matched = true;
        handleArmorJump(tracked_armor);
      }
    }
  }

  // Prevent radius from spreading
  if (target_state(8) < 0.1)
  {
    target_state(8) = 0.1;
    ekf.setState(target_state);
  }
  else if (target_state(8) > 0.4)
  {
    target_state(8) = 0.4;
    ekf.setState(target_state);
  }

  // Tracking state machine
  if (tracker_state == DETECTING)
  {
    if (matched)
    {
      detect_count_++;
      if (detect_count_ > tracking_threshold_)
      {
        detect_count_ = 0;
        tracker_state = TRACKING;
      }
    }
    else
    {
      detect_count_ = 0;
      tracker_state = LOST;
    }
  }
  else if (tracker_state == TRACKING)
  {
    if (!matched)
    {
      tracker_state = TEMP_LOST;
      lost_count_++;
    }
  }
  else if (tracker_state == TEMP_LOST)
  {
    if (!matched)
    {
      lost_count_++;
      if (lost_count_ > lost_threshold_)
      {
        lost_count_ = 0;
        tracker_state = LOST;
      }
    }
    else
    {
      tracker_state = TRACKING;
      lost_count_ = 0;
    }
  }
}

void Tracker::initEKF(const Armor& a)
{
  double xa = a.transform.getOrigin().x();
  double ya = a.transform.getOrigin().y();
  double za = a.transform.getOrigin().z();
  last_yaw_ = 0;
  double yaw = orientationToYaw(a.transform.getRotation());

  // Set initial position at 0.2m behind the target
  target_state = Eigen::VectorXd::Zero(9);
  double r = 0.2;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  double zc = za;
  dz = 0, another_r = r;
  target_state << xc, yc, zc, yaw, 0, 0, 0, 0, r;

  ekf.setState(target_state);
}

void Tracker::updateArmorsNum(const armor_processor::Armor& a)
{
  if (a.type == "large" && (tracked_id == 3 || tracked_id == 4 || tracked_id == 5))
    armors_num = 2;
  else if (tracked_id == 6)
    armors_num = 3;
  else
    armors_num = 4;
}

void Tracker::handleArmorJump(const Armor& a)
{
  double yaw = orientationToYaw(a.transform.getRotation());
  target_state(3) = yaw;
  updateArmorsNum(a);
  if (armors_num == 4)
  {
    dz = target_state(2) - a.transform.getOrigin().z();
    target_state(2) = a.transform.getOrigin().z();
    std::swap(target_state(8), another_r);
  }

  auto p = a.transform.getOrigin();
  Eigen::Vector3d current_p(p.x(), p.y(), p.z());
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);

  if ((current_p - infer_p).norm() > max_match_distance_)
  {
    double r = target_state(8);
    target_state(0) = p.x() + r * cos(yaw);
    target_state(1) = p.y() + r * sin(yaw);
    target_state(4) = 0;
    target_state(5) = 0;
    target_state(6) = 0;
  }

  ekf.setState(target_state);
}

double Tracker::orientationToYaw(const tf2::Quaternion& q)
{
  // Get armor yaw
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  // Make yaw change continuous
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd& x)
{
  // Calculate predicted position of the current armor
  double xc = x(0), yc = x(1), zc = x(2);
  double yaw = x(3), r = x(8);
  double xa = xc - r * cos(yaw);
  double ya = yc - r * sin(yaw);
  return Eigen::Vector3d(xa, ya, zc);
}

}  // namespace armor_processor
