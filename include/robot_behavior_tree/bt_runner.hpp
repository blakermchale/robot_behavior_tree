#ifndef ROBOT_BEHAVIOR_TREE__BT_RUNNER_HPP_
#define ROBOT_BEHAVIOR_TREE__BT_RUNNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "robot_control_interfaces/action/run_bt.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

namespace robot_behavior_tree
{
/**
 * @class robot_behavior_tree::BtRunner
 * @brief An action server that uses behavior tree for navigating a robot to its
 * goal position.
 */
class BtRunner : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for robot_behavior_tree::BtRunner class
   */
  BtRunner();
  /**
   * @brief A destructor for robot_behavior_tree::BtRunner class
   */
  ~BtRunner();

protected:
  /**
   * @brief Configures member variables
   *
   * Initializes action server for "RunBT"; subscription to
   * "goal_sub"; and builds behavior tree from xml file.
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using Action = robot_control_interfaces::action::RunBT;

  using ActionServer = nav2_util::SimpleActionServer<Action>;

  // Our action server implements the NavigateToPose action
  std::unique_ptr<ActionServer> action_server_;

  /**
   * @brief Action server callbacks
   */
  void run();

  /**
   * @brief Replace current BT with another one
   * @param bt_xml_filename The file containing the new BT
   * @return true if the resulting BT correspond to the one in bt_xml_filename. false
   * if something went wrong, and previous BT is mantained
   */
  bool loadBehaviorTree(const std::string & bt_id);

  BT::Tree tree_;

  // The blackboard shared by all of the nodes in the tree
  BT::Blackboard::Ptr blackboard_;

  // The XML fiñe that cointains the Behavior Tree to create
  std::string current_bt_xml_filename_;
  std::string default_bt_xml_filename_;

  // The wrapper class for the BT functionality
  std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_;

  // Libraries to pull plugins (BT Nodes) from
  std::vector<std::string> plugin_lib_names_;

  // A client that we'll use to send a command message to our own task server
  rclcpp_action::Client<Action>::SharedPtr self_client_;

  // A regular, non-spinning ROS node that we can use for calls to the action client
  rclcpp::Node::SharedPtr client_node_;

  // Spinning transform that can be used by the BT nodes
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Metrics for feedback
  rclcpp::Time start_time_;
};

}  // namespace robot_behavior_tree

#endif  // ROBOT_BEHAVIOR_TREE__BT_RUNNER_HPP_