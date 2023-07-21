#include "armor_processor/armor_filter.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace armor_processor
{
LogicFilterBase::LogicFilterBase(XmlRpc::XmlRpcValue rpc_value)
{
  if (rpc_value.hasMember("range"))
  {
    range_[0] = (double)rpc_value["range"][0];
    range_[1] = (double)rpc_value["range"][1];
  }
  else
    ROS_ERROR("Some filter params doesn't given");
}

HeightFilter::HeightFilter(const XmlRpc::XmlRpcValue& rpc_value, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
  : LogicFilterBase(rpc_value), tf_buffer_(tf_buffer)
{
  ROS_INFO("Height filter add.");
}
void HeightFilter::input(armor_processor::Armors& armors)
{
}

DistanceFilter::DistanceFilter(const XmlRpc::XmlRpcValue& rpc_value, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
  : LogicFilterBase(rpc_value), tf_buffer_(tf_buffer)
{
  ROS_INFO("Distance filter add.");
}
void DistanceFilter::input(armor_processor::Armors& armors)
{
}

IdFilter::IdFilter(const XmlRpc::XmlRpcValue& rpc_value) : LogicFilterBase()
{
  if (rpc_value.hasMember("id"))
    id_ = (int)rpc_value["id"];
  else
    ROS_ERROR("id_filter: id is not set.");
}

void IdFilter::input(armor_processor::Armors& armors)
{
  for (auto it = armors.begin(); it != armors.end();)
  {
    if (it->id == id_)
      armors.erase(it);
    else
      it++;
  }
}

}  // namespace armor_processor
