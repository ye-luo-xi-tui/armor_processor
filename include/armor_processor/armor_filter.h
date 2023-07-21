#pragma once

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include "armor_processor/tracker.hpp"

namespace armor_processor
{
class LogicFilterBase
{
public:
  explicit LogicFilterBase(XmlRpc::XmlRpcValue rpc_value);
  LogicFilterBase(){};
  virtual void input(armor_processor::Armors& armors){};

protected:
  double range_[2];
};

class HeightFilter : public LogicFilterBase
{
public:
  explicit HeightFilter(const XmlRpc::XmlRpcValue& rpc_value, std::shared_ptr<tf2_ros::Buffer> tf_buffer);
  void input(armor_processor::Armors& armors) override;

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

class DistanceFilter : public LogicFilterBase
{
public:
  explicit DistanceFilter(const XmlRpc::XmlRpcValue& rpc_value, std::shared_ptr<tf2_ros::Buffer> tf_buffer);
  void input(armor_processor::Armors& armors) override;

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

class IdFilter : public LogicFilterBase
{
public:
  explicit IdFilter(const XmlRpc::XmlRpcValue& rpc_value);
  void input(armor_processor::Armors& armors) override;

private:
  int id_;
};

}  // namespace armor_processor
