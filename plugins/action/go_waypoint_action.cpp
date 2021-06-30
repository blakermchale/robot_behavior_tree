#include <memory>
#include <string>

#include "robot_behavior_tree/plugins/action/go_waypoint_action.hpp"

namespace robot_behavior_tree
{

GoWaypointAction::GoWaypointAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<robot_control_interfaces::action::GoWaypoint>(xml_tag_name, action_name, conf)
{
}

void GoWaypointAction::on_tick()
{
  getInput("frame", goal_.waypoint.frame);
  getInput("x", goal_.waypoint.position.x);
  getInput("y", goal_.waypoint.position.y);
  getInput("z", goal_.waypoint.position.z);
  getInput("heading", goal_.waypoint.heading);
}

}  // namespace robot_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<robot_behavior_tree::GoWaypointAction>(
        name, "go_waypoint", config);
    };

  factory.registerBuilder<robot_behavior_tree::GoWaypointAction>(
    "GoWaypoint", builder);
}
