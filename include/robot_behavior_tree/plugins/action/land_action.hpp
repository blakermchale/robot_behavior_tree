#ifndef ROBOT_BEHAVIOR_TREE__PLUGINS__ACTION__LAND_ACTION_HPP_
#define ROBOT_BEHAVIOR_TREE__PLUGINS__ACTION__LAND_ACTION_HPP_

#include <string>

#include "robot_control_interfaces/action/land.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"

namespace robot_behavior_tree
{

using namespace BT;

class LandAction : public RosActionNode<robot_control_interfaces::action::Land>
{
public:
  LandAction(const std::string& name, const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<robot_control_interfaces::action::Land>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    PortsList basic = providedBasicPorts({});
    auto name_port = InputPort<std::string>("action_name", "land", "Action server name");
    basic["action_name"] = name_port.second;
    return basic;
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};

}  // namespace robot_behavior_tree

#endif  // ROBOT_BEHAVIOR_TREE__PLUGINS__ACTION__LAND_ACTION_HPP_