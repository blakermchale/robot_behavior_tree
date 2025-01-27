#ifndef ROBOT_BEHAVIOR_TREE__PLUGINS__ACTION__ARM_TAKEOFF_ACTION_HPP_
#define ROBOT_BEHAVIOR_TREE__PLUGINS__ACTION__ARM_TAKEOFF_ACTION_HPP_

#include <string>

#include "robot_control_interfaces/action/arm_takeoff.hpp"

#include "behaviortree_ros2/bt_action_node.hpp"

namespace robot_behavior_tree
{

using namespace BT;

class ArmTakeoffAction : public RosActionNode<robot_control_interfaces::action::ArmTakeoff>
{
public:
  ArmTakeoffAction(const std::string& name, const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<robot_control_interfaces::action::ArmTakeoff>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    PortsList basic = providedBasicPorts({
      InputPort<double>("altitude", "Takeoff altitude."),
    });
    auto name_port = InputPort<std::string>("action_name", "arm_takeoff", "Action server name");
    basic["action_name"] = name_port.second;
    return basic;
  }

  bool setGoal(Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override;
};

}  // namespace robot_behavior_tree

#endif  // ROBOT_BEHAVIOR_TREE__PLUGINS__ACTION__ARM_TAKEOFF_ACTION_HPP_