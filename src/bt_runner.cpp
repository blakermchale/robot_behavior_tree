#include "robot_behavior_tree/bt_runner.hpp"

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>
#include <set>
#include <exception>

#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav2_bt_navigator/ros_topic_logger.hpp"

namespace robot_behavior_tree
{

BtRunner::BtRunner()
: nav2_util::LifecycleNode("bt_runner", "", false),
  start_time_(0)
{
  RCLCPP_INFO(get_logger(), "Creating");

  const std::vector<std::string> plugin_libs = {
    "robot_arm_takeoff_action_bt_node"
  };

  // Declare this node's parameters
  declare_parameter("default_bt_xml_filename");
  declare_parameter("plugin_lib_names", plugin_libs);
  declare_parameter("enable_groot_monitoring", true);
  declare_parameter("groot_zmq_publisher_port", 1666);
  declare_parameter("groot_zmq_server_port", 1667);
}

BtRunner::~BtRunner()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_util::CallbackReturn
BtRunner::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // use suffix '_rclcpp_node' to keep parameter file consistency #1773
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args",
      "-r", std::string("__node:=") + get_name() + "_rclcpp_node",
      "--"});
  // Support for handling the topic-based goal pose from rviz
  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  self_client_ = rclcpp_action::create_client<robot_control_interfaces::action::RunBT>(
    client_node_, "run_tree");

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

  action_server_ = std::make_unique<ActionServer>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "run_tree", std::bind(&BtRunner::run, this), false);

  // Get the libraries to pull plugins from
  plugin_lib_names_ = get_parameter("plugin_lib_names").as_string_array();

  // Create the class that registers our custom nodes and executes the BT
  bt_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_lib_names_);

  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create();

  // Put items on the blackboard
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);  // NOLINT
  blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(10));  // NOLINT

  // Get the BT filename to use from the node parameter
  get_parameter("default_bt_xml_filename", default_bt_xml_filename_);

  if (!loadBehaviorTree(default_bt_xml_filename_)) {
    RCLCPP_ERROR(get_logger(), "Error loading XML file: %s", default_bt_xml_filename_.c_str());
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

bool
BtRunner::loadBehaviorTree(const std::string & bt_xml_filename)
{
  // Use previous BT if it is the existing one
  if (current_bt_xml_filename_ == bt_xml_filename) {
    RCLCPP_DEBUG(get_logger(), "BT will not be reloaded as the given xml is already loaded");
    return true;
  }

  // if a new tree is created, than the ZMQ Publisher must be destroyed
  bt_->resetGrootMonitor();

  // Read the input BT XML from the specified file into a string
  std::ifstream xml_file(bt_xml_filename);

  if (!xml_file.good()) {
    RCLCPP_ERROR(get_logger(), "Couldn't open input XML file: %s", bt_xml_filename.c_str());
    return false;
  }

  auto xml_string = std::string(
    std::istreambuf_iterator<char>(xml_file),
    std::istreambuf_iterator<char>());

  // Create the Behavior Tree from the XML input
  tree_ = bt_->createTreeFromText(xml_string, blackboard_);
  current_bt_xml_filename_ = bt_xml_filename;

  // get parameter for monitoring with Groot via ZMQ Publisher
  if (get_parameter("enable_groot_monitoring").as_bool()) {
    uint16_t zmq_publisher_port = get_parameter("groot_zmq_publisher_port").as_int();
    uint16_t zmq_server_port = get_parameter("groot_zmq_server_port").as_int();
    // optionally add max_msg_per_second = 25 (default) here
    try {
      bt_->addGrootMonitoring(&tree_, zmq_publisher_port, zmq_server_port);
    } catch (const std::logic_error & e) {
      RCLCPP_ERROR(get_logger(), "ZMQ already enabled, Error: %s", e.what());
    }
  }
  return true;
}

nav2_util::CallbackReturn
BtRunner::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  action_server_->activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtRunner::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtRunner::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // TODO(orduno) Fix the race condition between the worker thread ticking the tree
  //              and the main thread resetting the resources, see #1344
  client_node_.reset();
  self_client_.reset();

  // Reset the listener before the buffer
  tf_listener_.reset();
  tf_.reset();

  action_server_.reset();
  plugin_lib_names_.clear();
  current_bt_xml_filename_.clear();
  blackboard_.reset();
  bt_->haltAllActions(tree_.rootNode());
  bt_->resetGrootMonitor();
  bt_.reset();

  RCLCPP_INFO(get_logger(), "Completed Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
BtRunner::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void
BtRunner::run()
{

  auto is_canceling = [this]() {
      if (action_server_ == nullptr) {
        RCLCPP_DEBUG(get_logger(), "Action server unavailable. Canceling.");
        return true;
      }

      if (!action_server_->is_server_active()) {
        RCLCPP_DEBUG(get_logger(), "Action server is inactive. Canceling.");
        return true;
      }

      return action_server_->is_cancel_requested();
    };

  std::string bt_xml_filename = action_server_->get_current_goal()->behavior_tree;

  // Empty id in request is default for backward compatibility
  bt_xml_filename = bt_xml_filename == "" ? default_bt_xml_filename_ : bt_xml_filename;

  if (!loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      get_logger(), "BT file not found: %s. Navigation canceled.",
      bt_xml_filename.c_str());
    action_server_->terminate_current();
    return;
  }

  nav2_bt_navigator::RosTopicLogger topic_logger(client_node_, tree_);
  std::shared_ptr<Action::Feedback> feedback_msg = std::make_shared<Action::Feedback>();

  auto on_loop = [&]() {
      if (action_server_->is_preempt_requested()) {
        RCLCPP_INFO(get_logger(), "Received goal preemption request");
        action_server_->accept_pending_goal();
      }
      topic_logger.flush();

      feedback_msg->time = now() - start_time_;
      action_server_->publish_feedback(feedback_msg);
    };

  // Execute the BT that was previously created in the configure step
  nav2_behavior_tree::BtStatus rc = bt_->run(&tree_, on_loop, is_canceling);
  // Make sure that the Bt is not in a running state from a previous execution
  // note: if all the ControlNodes are implemented correctly, this is not needed.
  bt_->haltAllActions(tree_.rootNode());

  switch (rc) {
    case nav2_behavior_tree::BtStatus::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Navigation succeeded");
      action_server_->succeeded_current();
      break;

    case nav2_behavior_tree::BtStatus::FAILED:
      RCLCPP_ERROR(get_logger(), "Navigation failed");
      action_server_->terminate_current();
      break;

    case nav2_behavior_tree::BtStatus::CANCELED:
      RCLCPP_INFO(get_logger(), "Navigation canceled");
      action_server_->terminate_all();
      break;
  }
}

}  // namespace robot_behavior_tree