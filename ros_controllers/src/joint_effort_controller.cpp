// Copyright 2019 Dyno Robotics.
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

#include "ros_controllers/joint_effort_controller.hpp"

#include <cassert>
#include <chrono>
#include <iterator>
#include <string>
#include <memory>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rcutils/logging_macros.h"

namespace ros_controllers
{

using namespace std::chrono_literals;

JointEffortController::JointEffortController()
: controller_interface::ControllerInterface()
{
}

JointEffortController::JointEffortController(
  const std::vector<std::string> & joint_names,
  const std::vector<std::string> & write_op_names)
: controller_interface::ControllerInterface(),
  joint_names_(joint_names),
  write_op_names_(write_op_names)
{}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointEffortController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;

  joint_names_ = {};
  if (!lifecycle_node_->get_parameter("joint_names", joint_names_)) {
    RCUTILS_LOG_ERROR("joint_names parameter not found, aborting!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  for (auto joint_name : joint_names_) {
    position_commands_[joint_name] = 0.0;
    velocity_commands_[joint_name] = 0.0;
    effort_commands_[joint_name] = 0.0;
  }

  if (auto robot_hardware = robot_hardware_.lock()) {
    // register handle
    registered_joint_state_handles_.resize(joint_names_.size());
    for (size_t index = 0; index < joint_names_.size(); ++index) {
      auto ret = robot_hardware->get_joint_state_handle(
        joint_names_[index].c_str(), &registered_joint_state_handles_[index]);
      if (ret != hardware_interface::HW_RET_OK) {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
    }

    registered_joint_cmd_handles_.resize(joint_names_.size());
    for (size_t index = 0; index < joint_names_.size(); ++index) {
      auto ret = robot_hardware->get_joint_command_handle(
        joint_names_[index].c_str(), &registered_joint_cmd_handles_[index]);
      if (ret != hardware_interface::HW_RET_OK) {
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
      }
    }
  } else {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // subscriber call back
  // non realtime
  auto callback = [this](const std::shared_ptr<sensor_msgs::msg::JointState> msg)
    -> void
    {
      int joint_index = 0;
      for (auto name : msg->name) {
        // RCUTILS_LOG_INFO("Set position: %f", msg->position[joint_index]);
        position_commands_[name] = msg->position[joint_index];
        velocity_commands_[name] = msg->velocity[joint_index];
        effort_commands_[name] = msg->effort[joint_index];
        ++joint_index;
      }
    };

  // // TODO(sam): create subscriber with subscription deactivated
  joint_command_subscriber_ =
    lifecycle_node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_commands", callback);

  RCUTILS_LOG_INFO("Subscriber created");

  // TODO(sam): no lifecyle for subscriber yet
  // joint_command_subscriber_->on_activate();

  set_op_mode(hardware_interface::OperationMode::INACTIVE);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointEffortController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;
  //
  // is_halted = false;
  // subscriber_is_active_ = true;
  //
  // TODO(sam): activate subscriptions of subscriber
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointEffortController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;
  //
  // subscriber_is_active_ = false;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointEffortController::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointEffortController::on_error(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;
  if (!reset()) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool
JointEffortController::reset()
{
  RCUTILS_LOG_INFO("Position controller reset");

  //TODO(sam): reset handles?

  // subscriber_is_active_ = false;
  // joint_command_subscriber_.reset();
  //
  // is_halted = false;

  return true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
JointEffortController::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::controller_interface_ret_t
JointEffortController::update()
{
  using controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
  using lifecycle_msgs::msg::State;

  // RCUTILS_LOG_INFO("Position controller update");

  if (lifecycle_node_->get_current_state().id() == State::PRIMARY_STATE_INACTIVE) {
    if (!is_halted) {
      halt();
      is_halted = true;
    }
    return CONTROLLER_INTERFACE_RET_SUCCESS;
  }
  set_op_mode(hardware_interface::OperationMode::ACTIVE);

  int joint_index = 0;
  for (auto & registered_joint_cmd_handle : registered_joint_cmd_handles_) {
    std::string joint_name = joint_names_[joint_index];

    registered_joint_cmd_handle->set_position(position_commands_[joint_name]);
    registered_joint_cmd_handle->set_velocity(velocity_commands_[joint_name]);
    registered_joint_cmd_handle->set_effort(effort_commands_[joint_name]);
    ++joint_index;
  }

  return CONTROLLER_INTERFACE_RET_SUCCESS;
}

void
JointEffortController::set_op_mode(const hardware_interface::OperationMode & mode)
{
  // RCUTILS_LOG_INFO("Position controller set op mode");
  for (auto & op_mode_handle : registered_operation_mode_handles_) {
    op_mode_handle->set_mode(mode);
  }
}

void
JointEffortController::halt()
{
  RCUTILS_LOG_INFO("Position controller halt");

  size_t joint_num = registered_joint_cmd_handles_.size();
  for (size_t index = 0; index < joint_num; ++index) {
    registered_joint_cmd_handles_[index]->set_position(
      registered_joint_state_handles_[index]->get_position());
  }
  set_op_mode(hardware_interface::OperationMode::ACTIVE);
}

}  // namespace ros_controllers

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  ros_controllers::JointEffortController, controller_interface::ControllerInterface)
