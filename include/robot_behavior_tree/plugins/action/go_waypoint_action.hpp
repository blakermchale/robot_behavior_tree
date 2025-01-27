#ifndef ROBOT_BEHAVIOR_TREE__PLUGINS__ACTION__GO_WAYPOINT_ACTION_HPP_
#define ROBOT_BEHAVIOR_TREE__PLUGINS__ACTION__GO_WAYPOINT_ACTION_HPP_

#include <string>

#include "robot_control_interfaces/action/go_waypoint.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

namespace robot_behavior_tree
{

using namespace BT;

class GoWaypointAction : public RosActionNode<robot_control_interfaces::action::GoWaypoint>
{
public:
  GoWaypointAction(const std::string& name, const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<robot_control_interfaces::action::GoWaypoint>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    PortsList basic = providedBasicPorts({
      BT::InputPort<uint>("frame", 1, "Reference frame"),
      BT::InputPort<double>("x", 0.0, "x position m"),
      BT::InputPort<double>("y", 0.0, "y position m"),
      BT::InputPort<double>("z", 0.0, "z position m"),
      BT::InputPort<double>("heading", 0.0, "heading rad/s"),
    });
    auto name_port = InputPort<std::string>("action_name", "go_waypoint", "Action server name");
    basic["action_name"] = name_port.second;
    return basic;
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};

}  // namespace robot_behavior_tree

#endif  // ROBOT_BEHAVIOR_TREE__PLUGINS__ACTION__GO_WAYPOINT_ACTION_HPP_