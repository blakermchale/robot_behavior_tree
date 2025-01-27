#include <memory>
#include <string>

#include "robot_behavior_tree/plugins/action/land_action.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace robot_behavior_tree
{

bool LandAction::setGoal(RosActionNode::Goal& goal)
{
  return true;
}

NodeStatus LandAction::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  return NodeStatus::SUCCESS;
}

NodeStatus LandAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void LandAction::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

}  // namespace robot_behavior_tree

CreateRosNodePlugin(robot_behavior_tree::LandAction, "Land");
