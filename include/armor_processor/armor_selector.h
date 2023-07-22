//
// Created by yezi on 23-7-22.
//

#pragma once

#include <ros/ros.h>
#include "armor_processor/tracker.hpp"

namespace armor_processor
{
class ArmorSelectorBase
{
public:
  ArmorSelectorBase() = default;
  virtual bool input(armor_processor::Armors& armors)
  {
    return true;
  }
  Armor* output() const
  {
    return selected_armor_;
  }

protected:
  static Armor* selected_armor_;
};

class ClosestToImageCenterSelector : public ArmorSelectorBase
{
public:
  ClosestToImageCenterSelector()
  {
    ROS_INFO("Add closest_to_image_center_selector.");
  }
  bool input(armor_processor::Armors& armors) override;
};

class IdSelector : public ArmorSelectorBase
{
public:
  IdSelector(const XmlRpc::XmlRpcValue& rpc_value)
  {
    if (rpc_value.hasMember("id"))
    {
      id_ = (int)rpc_value["id"];
      ROS_INFO("Add id_selector");
    }
    else
      ROS_ERROR("id_selector: id is not set.");
  }
  bool input(armor_processor::Armors& armors) override;

private:
  int id_;
};
}  // namespace armor_processor
