#include <memory>
#include <string>

#include "robot_behavior_tree/plugins/action/go_waypoint_action.hpp"
#include "behaviortree_ros2/plugins.hpp"


namespace robot_behavior_tree
{

bool GoWaypointAction::setGoal(RosActionNode::Goal& goal)
{
  // auto altitude = getInput<double>("altitude");
  // goal.altitude = altitude.value();
  getInput("frame", goal.waypoint.frame);
  getInput("x", goal.waypoint.position.x);
  getInput("y", goal.waypoint.position.y);
  getInput("z", goal.waypoint.position.z);
  getInput("heading", goal.waypoint.heading);
  return true;
}

NodeStatus GoWaypointAction::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  return NodeStatus::SUCCESS;
}

NodeStatus GoWaypointAction::onFailure(ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return NodeStatus::FAILURE;
}

void GoWaypointAction::onHalt()
{
  RCLCPP_INFO(logger(), "%s: onHalt", name().c_str());
}

}  // namespace robot_behavior_tree

CreateRosNodePlugin(robot_behavior_tree::GoWaypointAction, "GoWaypoint");
