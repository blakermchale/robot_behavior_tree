#include <memory>
#include <string>

#include "robot_behavior_tree/plugins/action/arm_takeoff_action.hpp"
#include "behaviortree_ros2/plugins.hpp"


namespace robot_behavior_tree
{

bool ArmTakeoffAction::setGoal(RosActionNode::Goal& goal)
{
  auto altitude = getInput<double>("altitude");
  goal.altitude = altitude.value();
  return true;
}

NodeStatus ArmTakeoffAction::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  return NodeStatus::SUCCESS;
}

NodeStatus ArmTakeoffAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void ArmTakeoffAction::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

}  // namespace robot_behavior_tree

CreateRosNodePlugin(robot_behavior_tree::ArmTakeoffAction, "ArmTakeoff");
