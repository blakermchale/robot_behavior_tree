#ifndef ROBOT_BEHAVIOR_TREE__PLUGINS__ACTION__LAND_ACTION_HPP_
#define ROBOT_BEHAVIOR_TREE__PLUGINS__ACTION__LAND_ACTION_HPP_

#include <string>

#include "robot_control_interfaces/action/land.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace robot_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps robot_control_interfaces::action::Land
 */
class LandAction : public nav2_behavior_tree::BtActionNode<robot_control_interfaces::action::Land>
{
public:
  /**
   * @brief A constructor for robot_behavior_tree::LandAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  LandAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }
};

}  // namespace robot_behavior_tree

#endif  // ROBOT_BEHAVIOR_TREE__PLUGINS__ACTION__LAND_ACTION_HPP_