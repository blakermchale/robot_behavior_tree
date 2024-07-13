// Copyright (c) 2021 Samsung Research
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <vector>
#include <string>
#include <set>
#include <memory>
#include <limits>
#include "robot_behavior_tree/runner.hpp"

namespace robot_behavior_tree
{

bool
Runner::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();

  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

  return true;
}

std::string
Runner::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();
  if (!node->has_parameter("default_run_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("robot_behavior_tree");
    std::string tree_file = pkg_share_dir +
      "/trees/test_actions.xml";
    node->declare_parameter("default_run_bt_xml", tree_file);
  }
  node->get_parameter("default_run_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool
Runner::cleanup()
{
  self_client_.reset();
  return true;
}

bool
Runner::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = goal->behavior_tree;
  if (bt_xml_filename == "") {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("robot_behavior_tree");
    bt_xml_filename = pkg_share_dir +
      "/trees/test_actions.xml";
  }
  RCLCPP_INFO(logger_, "Received goal request: %s", bt_xml_filename.c_str());

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "BT file not found: %s. Runner canceled.",
      bt_xml_filename.c_str());
    return false;
  }

  initializeRun();

  return true;
}

void
Runner::goalCompleted(typename ActionT::Result::SharedPtr /*result*/)
{
}

void
Runner::onLoop()
{
  auto feedback_msg = std::make_shared<ActionT::Feedback>();

  feedback_msg->time = clock_->now() - start_time_;

  bt_action_server_->publishFeedback(feedback_msg);
}

void
Runner::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Received goal preemption request");

  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
    (goal->behavior_tree.empty() &&
    bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
  {
    // if pending goal requests the same BT as the current goal, accept the pending goal
    // if pending goal has an empty behavior_tree field, it requests the default BT file
    // accept the pending goal if the current goal is running the default BT file
    initializeRun();
  } else {
    RCLCPP_WARN(
      logger_,
      "Preemption request was rejected since the requested BT XML file is not the same "
      "as the one that the current goal is executing. Preemption with a new BT is invalid "
      "since it would require cancellation of the previous goal instead of true preemption."
      "\nCancel the current goal and send a new action request if you want to use a "
      "different BT XML file. For now, continuing to track the last goal until completion.");
    bt_action_server_->terminatePendingGoal();
  }
}

void
Runner::initializeRun()
{
  // Reset state for new action feedback
  start_time_ = clock_->now();
}

}  // namespace robot_behavior_tree
