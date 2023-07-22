#pragma once

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include "armor_processor/tracker.hpp"

namespace armor_processor
{
class ArmorFilterBase
{
public:
  explicit ArmorFilterBase(XmlRpc::XmlRpcValue rpc_value);
  ArmorFilterBase(){};
  virtual void input(armor_processor::Armors& armors){};

protected:
  double range_[2];
};

class HeightFilter : public ArmorFilterBase
{
public:
  explicit HeightFilter(const XmlRpc::XmlRpcValue& rpc_value, std::shared_ptr<tf2_ros::Buffer> tf_buffer);
  void input(armor_processor::Armors& armors) override;

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

class DistanceFilter : public ArmorFilterBase
{
public:
  explicit DistanceFilter(const XmlRpc::XmlRpcValue& rpc_value, std::shared_ptr<tf2_ros::Buffer> tf_buffer);
  void input(armor_processor::Armors& armors) override;

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

class IdFilter : public ArmorFilterBase
{
public:
  explicit IdFilter(const XmlRpc::XmlRpcValue& rpc_value);
  void input(armor_processor::Armors& armors) override;

private:
  int id_;
};

}  // namespace armor_processor
