//
// Created by yezi on 23-4-29.
//

#include "armor_processor/processor_node.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "armor_processor");
  ros::NodeHandle nh("~");
  auto* processor = new rm_auto_aim::ArmorProcessorNode(nh);
  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}