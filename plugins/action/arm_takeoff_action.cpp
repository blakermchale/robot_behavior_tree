#include <memory>
#include <string>

#include "robot_behavior_tree/plugins/action/arm_takeoff_action.hpp"

namespace robot_behavior_tree
{

ArmTakeoffAction::ArmTakeoffAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<robot_control_interfaces::action::ArmTakeoff>(xml_tag_name, action_name, conf)
{
}

void ArmTakeoffAction::on_tick()
{
  getInput("altitude", goal_.altitude);
}

}  // namespace robot_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<robot_behavior_tree::ArmTakeoffAction>(
        name, "arm_takeoff", config);
    };

  factory.registerBuilder<robot_behavior_tree::ArmTakeoffAction>(
    "ArmTakeoff", builder);
}
