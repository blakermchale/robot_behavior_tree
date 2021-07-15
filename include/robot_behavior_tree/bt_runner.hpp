// Copyright (c) 2018 Intel Corporation
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

#ifndef ROBOT_BEHAVIOR_TREE__BT_RUNNER_HPP_
#define ROBOT_BEHAVIOR_TREE__BT_RUNNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "robot_behavior_tree/runner.hpp"

namespace robot_behavior_tree
{

/**
 * @class robot_behavior_tree::BtRunner
 * @brief An action server that runs a behavior tree.
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
   * Initializes action server for "Runner"; 
   * and builds behavior tree from xml file.
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

  // To handle all the BT related execution
  std::unique_ptr<robot_behavior_tree::Runner> bt_runner_;
  robot_behavior_tree::RunnerMuxer plugin_muxer_;
};

}  // namespace robot_behavior_tree

#endif  // ROBOT_BEHAVIOR_TREE__BT_RUNNER_HPP_
