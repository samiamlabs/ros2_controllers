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

#ifndef ROS_CONTROLLERS__JOINT_POSITION_CONTROLLER_HPP_
#define ROS_CONTROLLERS__JOINT_POSITION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"

#include "hardware_interface/operation_mode_handle.hpp"
#include "hardware_interface/robot_hardware.hpp"

#include "rclcpp_lifecycle/state.hpp"

#include "ros_controllers/visibility_control.h"

#include "sensor_msgs/msg/joint_state.hpp"

namespace ros_controllers
{

class JointPositionController : public controller_interface::ControllerInterface
{
public:
  ROS_CONTROLLERS_PUBLIC
  JointPositionController();

  ROS_CONTROLLERS_PUBLIC
  JointPositionController(
    const std::vector<std::string> & joint_names,
    const std::vector<std::string> & write_op_names);

  ROS_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ROS_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ROS_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ROS_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  ROS_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State & previous_state) override;

  ROS_CONTROLLERS_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  ROS_CONTROLLERS_PUBLIC
  controller_interface::controller_interface_ret_t
  update() override;

private:
  std::vector<std::string> joint_names_;
  std::vector<std::string> write_op_names_;

  std::map<std::string, double> position_commands_;
  std::map<std::string, double> velocity_commands_;
  std::map<std::string, double> effort_commands_;

  std::vector<hardware_interface::JointCommandHandle *> registered_joint_cmd_handles_;
  std::vector<const hardware_interface::JointStateHandle *> registered_joint_state_handles_;
  std::vector<hardware_interface::OperationModeHandle *> registered_operation_mode_handles_;
  // hardware_interface::JointCommandHandle * registered_joint_cmd_handle_;
  // const hardware_interface::JointStateHandle * registered_joint_state_handle_;
  // hardware_interface::OperationModeHandle * registered_operation_mode_handle_;

  // TODO(sam): eventually activate and deactive subscriber directly when its supported
  bool subscriber_is_active_ = false;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
  joint_command_subscriber_ = nullptr;

  // TrajectoryPointConstIter prev_traj_point_ptr_;
  // std::shared_ptr<Trajectory> * traj_point_active_ptr_ = nullptr;
  // std::shared_ptr<Trajectory> traj_external_point_ptr_ = nullptr;
  // std::shared_ptr<Trajectory> traj_home_point_ptr_ = nullptr;
  // std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg_home_ptr_ = nullptr;

  bool is_halted = false;

  bool reset();
  void set_op_mode(const hardware_interface::OperationMode & mode);
  void halt();
};

}  // namespace ros_controllers

#endif  // ROS_CONTROLLERS__JOINT_TRAJECTORY_CONTROLLER_HPP_
