// Copyright 2022 Chen Jun
#include "armor_processor/processor_node.hpp"

#include <rm_msgs/TrackData.h>

// STD
#include <memory>
#include <vector>

namespace rm_auto_aim
{
ArmorProcessorNode::ArmorProcessorNode(ros::NodeHandle& nh)
: last_time_(0), dt_(0.0)
{
  ROS_INFO("Starting ProcessorNode!");

  // Tracker
  ros::NodeHandle tracker_nh(nh,"tracker");
  double max_match_distance = tracker_nh.param("max_match_distance", 0.2);
  int tracking_threshold = tracker_nh.param("tracking_threshold", 5);
  int lost_threshold = tracker_nh.param("lost_threshold", 5);


  tracker_ = std::make_unique<Tracker>(max_match_distance, tracking_threshold, lost_threshold);

  // EKF
  // xa = x_armor, xc = x_robot_center
  // state: xc, yc, zc, yaw, v_xc, v_yc, v_zc, v_yaw, r
  // measurement: xa, ya, za, yaw
  // f - Process function
  auto f = [this](const Eigen::VectorXd & x) {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(4) * dt_;
    x_new(1) += x(5) * dt_;
    x_new(2) += x(6) * dt_;
    x_new(3) += x(7) * dt_;
    return x_new;
  };
  // J_f - Jacobian of process function
  auto j_f = [this](const Eigen::VectorXd &) {
    Eigen::MatrixXd f(9, 9);
    // clang-format off
    f <<  1,   0,   0,   0,   dt_, 0,   0,   0,   0,
          0,   1,   0,   0,   0,   dt_, 0,   0,   0,
          0,   0,   1,   0,   0,   0,   dt_, 0,   0, 
          0,   0,   0,   1,   0,   0,   0,   dt_, 0,
          0,   0,   0,   0,   1,   0,   0,   0,   0,
          0,   0,   0,   0,   0,   1,   0,   0,   0,
          0,   0,   0,   0,   0,   0,   1,   0,   0,
          0,   0,   0,   0,   0,   0,   0,   1,   0,
          0,   0,   0,   0,   0,   0,   0,   0,   1;
    // clang-format on
    return f;
  };
  // h - Observation function
  auto h = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd z(4);
    double xc = x(0), yc = x(1), yaw = x(3), r = x(8);
    z(0) = xc - r * cos(yaw);  // xa
    z(1) = yc - r * sin(yaw);  // ya
    z(2) = x(2);               // za
    z(3) = x(3);               // yaw
    return z;
  };
  // J_h - Jacobian of observation function
  auto j_h = [](const Eigen::VectorXd & x) {
    Eigen::MatrixXd h(4, 9);
    double yaw = x(3), r = x(8);
    // clang-format off
    //    xc   yc   zc   yaw         vxc  vyc  vzc  vyaw r
    h <<  1,   0,   0,   r*sin(yaw), 0,   0,   0,   0,   -cos(yaw),
          0,   1,   0,   -r*cos(yaw),0,   0,   0,   0,   -sin(yaw),
          0,   0,   1,   0,          0,   0,   0,   0,   0,
          0,   0,   0,   1,          0,   0,   0,   0,   0;
    // clang-format on
    return h;
  };
  // Q - process noise covariance matrix
  ros::NodeHandle ekf_nh(nh,"ekf");
  auto q_v = ekf_nh.param(
    "q", std::vector<double>{//xc  yc    zc    yaw   vxc   vyc   vzc   vyaw  r
                                1e-2, 1e-2, 1e-2, 2e-2, 5e-2, 5e-2, 1e-4, 4e-2, 1e-3});
  Eigen::DiagonalMatrix<double, 9> q;
  q.diagonal() << q_v[0], q_v[1], q_v[2], q_v[3], q_v[4], q_v[5], q_v[6], q_v[7], q_v[8];
  // R - measurement noise covariance matrix
  auto r_v = ekf_nh.param(
    "r", std::vector<double>{//xa  ya    za    yaw
                                1e-1, 1e-1, 1e-1, 2e-1});
  Eigen::DiagonalMatrix<double, 4> r;
  r.diagonal() << r_v[0], r_v[1], r_v[2], r_v[3];
  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 9> p0;
  p0.setIdentity();
  tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, q, r, p0};

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(10));
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  armors_sub_.subscribe(nh, "/detection", 10);
  nh.param("target_frame", target_frame_,std::string("odom"));
  tf2_filter_ = std::make_shared<tf2_filter>(
    armors_sub_, *tf2_buffer_, target_frame_, 10, nullptr);
  // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  tf2_filter_->registerCallback(&ArmorProcessorNode::armorsCallback, this);

  // Publisher
  target_pub_ = nh.advertise<rm_msgs::TrackData>(
    "/track", 1);
  entire_car_state_pub_ = nh.advertise<rm_msgs::EntireCarState>("entire_car_state", 1);

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  linear_v_marker_.type = visualization_msgs::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armors_marker_.ns = "armors";
  armors_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
  armors_marker_.scale.x = armors_marker_.scale.y = armors_marker_.scale.z = 0.1;
  armors_marker_.color.a = 1.0;
  armors_marker_.color.r = 1.0;
  marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/processor/marker", 10);
}

void ArmorProcessorNode::armorsCallback(
  const rm_msgs::TargetDetectionArray::ConstPtr& msg)
{
  // Tranform armor position from image frame to world coordinate
  Armors armors;
  tf2::Transform t_t(tf2::Matrix3x3(0,-1,0,0,0,-1,1,0,0));
  for (auto & armor : msg->detections) {
    geometry_msgs::PoseStamped ps;
    ps.header = msg->header;
    ps.pose = armor.pose;
    try {
        tf2_buffer_->transform(ps, ps,target_frame_);
        tf2::Transform transform;
        tf2::fromMsg(ps.pose,transform);
        transform *= t_t;
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header = ps.header;
        transform_stamped.transform = tf2::toMsg(transform);
        transform_stamped.child_frame_id = "target" + std::to_string(armor.id);
        br_.sendTransform(transform_stamped);
        armors.push_back(Armor{.id = armor.id, .distance_to_image_center = armor.distance_to_image_center,.transform = transform});
    } catch (const tf2::ExtrapolationException & ex) {
      ROS_ERROR("Error while transforming %s", ex.what());
      return;
    }
  }

  rm_msgs::EntireCarState entire_car_state;
  rm_msgs::TrackData target_msg;
  ros::Time time = msg->header.stamp;
  target_msg.header.stamp = time;
  entire_car_state.header.stamp = time;
  target_msg.header.frame_id = target_frame_;
  entire_car_state.header.frame_id  = target_frame_;

  if (tracker_->tracker_state == Tracker::LOST) {
    tracker_->init(armors);
    entire_car_state.tracking = false;
  } else {
    dt_ = (time - last_time_).toSec();
    tracker_->update(armors);

    if (tracker_->tracker_state == Tracker::DETECTING) {
      entire_car_state.tracking = false;
    } else if (
      tracker_->tracker_state == Tracker::TRACKING ||
      tracker_->tracker_state == Tracker::TEMP_LOST) {
      entire_car_state.tracking = true;
      target_msg.id = tracker_->tracked_id;
    }
  }

  last_time_ = time;

  const auto state = tracker_->target_state;
  target_msg.target_pos.x = state(0) - state(8) * cos(state(3));
  target_msg.target_pos.y = state(1) - state(8) * sin(state(3));
  target_msg.target_pos.z = state(2);
  target_msg.target_vel.x = state(4) + state(7) * state(8) * sin(state(3));
  target_msg.target_vel.y = state(5) - state(7) * state(8) * cos(state(3));
  target_msg.target_vel.z = state(6);
  entire_car_state.position.x = state(0);
  entire_car_state.position.y = state(1);
  entire_car_state.position.z = state(2);
  entire_car_state.velocity.x = state(4);
  entire_car_state.velocity.y = state(5);
  entire_car_state.velocity.z = state(6);
  entire_car_state.yaw = state(3);
  entire_car_state.v_yaw = state(7);
  entire_car_state.radius_1 = state(8);
  entire_car_state.radius_2 = tracker_->last_r;
  entire_car_state.z_2 = tracker_->last_z;
  target_pub_.publish(target_msg);
  entire_car_state_pub_.publish(entire_car_state);

  publishMarkers(entire_car_state);
}

void ArmorProcessorNode::publishMarkers(const rm_msgs::EntireCarState & entire_car_state)
{
  position_marker_.header = entire_car_state.header;
  linear_v_marker_.header = entire_car_state.header;
  angular_v_marker_.header = entire_car_state.header;
  armors_marker_.header = entire_car_state.header;

  if (entire_car_state.tracking) {
    auto state = tracker_->target_state;
    double yaw = entire_car_state.yaw, r1 = entire_car_state.radius_1, r2 = entire_car_state.radius_2;
    double xc = entire_car_state.position.x, yc = entire_car_state.position.y, zc = entire_car_state.position.z;
    double z2 = entire_car_state.z_2;
    position_marker_.action = visualization_msgs::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = (zc + z2) / 2;

    linear_v_marker_.action = visualization_msgs::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += state(4);
    arrow_end.y += state(5);
    arrow_end.z += state(6);
    linear_v_marker_.points.emplace_back(arrow_end);

    angular_v_marker_.action = visualization_msgs::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += state(7) / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    armors_marker_.action = visualization_msgs::Marker::ADD;
    armors_marker_.points.clear();
    geometry_msgs::Point p_a;
    bool use_1 = true;
    for (size_t i = 0; i < 4; i++) {
      double tmp_yaw = yaw + i * M_PI_2;
      double r = use_1 ? r1 : r2;
      p_a.x = xc - r * cos(tmp_yaw);
      p_a.y = yc - r * sin(tmp_yaw);
      p_a.z = use_1 ? zc : z2;
      armors_marker_.points.emplace_back(p_a);
      use_1 = !use_1;
    }
  } else {
    position_marker_.action = visualization_msgs::Marker::DELETE;
    linear_v_marker_.action = visualization_msgs::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::Marker::DELETE;
    armors_marker_.action = visualization_msgs::Marker::DELETE;
  }

  visualization_msgs::MarkerArray marker_array;

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);
  marker_array.markers.emplace_back(armors_marker_);
  marker_pub_.publish(marker_array);
}

}  // namespace rm_auto_aim
