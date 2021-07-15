// Copyright (c) 2021 Samsung Research America
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

#ifndef ROBOT_BEHAVIOR_TREE__RUNNER_HPP_
#define ROBOT_BEHAVIOR_TREE__RUNNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"
#include "robot_control_interfaces/action/run_bt.hpp"

namespace robot_behavior_tree
{

/**
 * @class RunnerMuxer
 * @brief A class to control the state of the BT runner by allowing only a single
 * plugin to be processed at a time.
 */
class RunnerMuxer
{
public:
  /**
   * @brief A Runner Muxer constructor
   */
  RunnerMuxer()
  : current_runner_(std::string("")) {}

  /**
   * @brief Get the runner muxer state
   * @return bool If a runner is in progress
   */
  bool isRunning()
  {
    std::scoped_lock l(mutex_);
    return !current_runner_.empty();
  }

  /**
   * @brief Start navigating with a given runner
   * @param string Name of the runner to start
   */
  void startRunning(const std::string & runner_name)
  {
    std::scoped_lock l(mutex_);
    if (!current_runner_.empty()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("RunnerMutex"),
        "Major error! Runner requested while another runner"
        " task is in progress! This likely occurred from an incorrect"
        "implementation of a runner plugin.");
    }
    current_runner_ = runner_name;
  }

  /**
   * @brief Stop navigating with a given runner
   * @param string Name of the runner ending task
   */
  void stopRunning(const std::string & runner_name)
  {
    std::scoped_lock l(mutex_);
    if (current_runner_ != runner_name) {
      RCLCPP_ERROR(
        rclcpp::get_logger("RunnerMutex"),
        "Major error! Runner stopped while another runner"
        " task is in progress! This likely occurred from an incorrect"
        "implementation of a runner plugin.");
    } else {
      current_runner_ = std::string("");
    }
  }

protected:
  std::string current_runner_;
  std::mutex mutex_;
};

/**
 * @class Runner
 * @brief Runner interface that acts as a base class for all BT-based Runner action's plugins
 */
class Runner
{
public:
    using ActionT = robot_control_interfaces::action::RunBT;
    using Ptr = std::shared_ptr<robot_behavior_tree::Runner>;

    /**
     * @brief A Runner constructor
     */
    Runner()
    {
        plugin_muxer_ = nullptr;
    }

    /**
     * @brief Virtual destructor
     */
    virtual ~Runner() = default;

    /**
     * @brief Configuration to setup the runner's backend BT and actions
     * @param parent_node The ROS parent node to utilize
     * @param plugin_lib_names a vector of plugin shared libraries to load
     * @param plugin_muxer The muxing object to ensure only one runner
     * can be active at a time
     * @return bool If successful
     */
    bool on_configure(
        rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
        const std::vector<std::string> & plugin_lib_names,
        robot_behavior_tree::RunnerMuxer * plugin_muxer)
    {
        auto node = parent_node.lock();
        logger_ = node->get_logger();
        clock_ = node->get_clock();
        plugin_muxer_ = plugin_muxer;

        // get the default behavior tree for this runner
        std::string default_bt_xml_filename = getDefaultBTFilepath(parent_node);

        // Create the Behavior Tree Action Server for this runner
        bt_action_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<ActionT>>(
        node,
        getName(),
        plugin_lib_names,
        default_bt_xml_filename,
        std::bind(&Runner::onGoalReceived, this, std::placeholders::_1),
        std::bind(&Runner::onLoop, this),
        std::bind(&Runner::onPreempt, this, std::placeholders::_1),
        std::bind(&Runner::onCompletion, this, std::placeholders::_1));

        bool ok = true;
        if (!bt_action_server_->on_configure()) {
        ok = false;
        }

        return configure(parent_node) && ok;
    }

    /**
     * @brief Actiation of the runner's backend BT and actions
     * @return bool If successful
     */
    bool on_activate()
    {
        bool ok = true;

        if (!bt_action_server_->on_activate()) {
        ok = false;
        }

        return activate() && ok;
    }

    /**
     * @brief Dectiation of the runner's backend BT and actions
     * @return bool If successful
     */
    bool on_deactivate()
    {
        bool ok = true;
        if (!bt_action_server_->on_deactivate()) {
        ok = false;
        }

        return deactivate() && ok;
    }

    /**
     * @brief Cleanup a runner
     * @return bool If successful
     */
    bool on_cleanup()
    {
        bool ok = true;
        if (!bt_action_server_->on_cleanup()) {
        ok = false;
        }

        bt_action_server_.reset();

        return cleanup() && ok;
    }

    /**
     * @brief Get the action name of this runner to expose
     * @return string Name of action to expose
     */
    std::string getName() {return std::string("run_bt");}

    std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node);

protected:
    /**
     * @brief An intermediate goal reception function to mux navigators.
     */
    bool onGoalReceived(typename ActionT::Goal::ConstSharedPtr goal)
    {
        if (plugin_muxer_->isRunning()) {
        RCLCPP_ERROR(
            logger_,
            "Requested runner from %s while another runner is processing,"
            " rejecting request.", getName().c_str());
        return false;
        }

        plugin_muxer_->startRunning(getName());

        return goalReceived(goal);
    }

    /**
     * @brief An intermediate compution function to mux navigators
     */
    void onCompletion(typename ActionT::Result::SharedPtr result)
    {
        plugin_muxer_->stopRunning(getName());
        goalCompleted(result);
    }

    /**
     * @brief A callback to be called when a new goal is received by the BT action server
     * Can be used to check if goal is valid and put values on
     * the blackboard which depend on the received goal
     */
    bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal);

    /**
     * @brief A callback that defines execution that happens on one iteration through the BT
     * Can be used to publish action feedback
     */
    void onLoop();

    /**
     * @brief A callback that is called when a preempt is requested
     */
    void onPreempt(typename ActionT::Goal::ConstSharedPtr goal);

    /**
     * @brief A callback that is called when a the action is completed, can fill in
     * action result message or indicate that this action is done.
     */
    void goalCompleted(typename ActionT::Result::SharedPtr result);

    /**
     * @param Method to configure resources.
     */
    bool configure(rclcpp_lifecycle::LifecycleNode::WeakPtr /*node*/);

    /**
     * @brief Method to cleanup resources.
     */
    bool cleanup();

    /**
     * @brief Method to active and any threads involved in execution.
     */
    virtual bool activate() {return true;}

    /**
     * @brief Method to deactive and any threads involved in execution.
     */
    virtual bool deactivate() {return true;}

    /**
     * @brief Initialize run
     */
    void initializeRun();

    rclcpp::Time start_time_;

    rclcpp_action::Client<ActionT>::SharedPtr self_client_;

    std::unique_ptr<nav2_behavior_tree::BtActionServer<ActionT>> bt_action_server_;
    rclcpp::Logger logger_{rclcpp::get_logger("Runner")};
    rclcpp::Clock::SharedPtr clock_;
    RunnerMuxer * plugin_muxer_;
};

}  // namespace robot_behavior_tree

#endif  // ROBOT_BEHAVIOR_TREE__RUNNER_HPP_