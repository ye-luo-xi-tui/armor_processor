//
// Created by yezi on 23-7-22.
//

#include "armor_processor/armor_selector.h"

namespace armor_processor
{
bool ClosestToImageCenterSelector::input(armor_processor::Armors& armors)
{
  double min_distance_from_light_center = DBL_MAX;
  if (armors.empty())
    return false;
  else
    for (auto& armor : armors)
    {
      if (armor.distance_to_image_center < min_distance_from_light_center)
      {
        selected_armor_ = &armor;
        min_distance_from_light_center = armor.distance_to_image_center;
      }
    }
  return true;
}

bool IdSelector::input(armor_processor::Armors& armors)
{
  for (auto& armor : armors)
    if (armor.id == id_)
    {
      selected_armor_ = &armor;
      return true;
    }
  return false;
}

Armor* ArmorSelectorBase::selected_armor_;
}  // namespace armor_processor
