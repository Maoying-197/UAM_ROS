// Copyright 2024 UAM_ROS maintainers
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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace mujoco_arm_hardware
{

/// ros2_control SystemInterface backed by the MuJoCo physics simulator.
///
/// Hardware parameters (set in the <hardware> section of the ros2_control xacro):
///   model_path   – absolute path to the MJCF (.xml) model file (required)
///   n_substeps   – MuJoCo steps per ros2_control update cycle (default 1)
///
/// Actuator naming convention in the MJCF: each joint named "jointN" must have
/// a corresponding position actuator named "act_jointN".
class MujocoArmHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MujocoArmHardware);

  // ── Lifecycle ──────────────────────────────────────────────────────────────

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  // ── ros2_control interface export ─────────────────────────────────────────

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ── Read / Write ──────────────────────────────────────────────────────────

  /// Copy qpos/qvel from the MuJoCo state into the ros2_control state buffers.
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Apply command positions to MuJoCo actuators and advance the simulation by
  /// n_substeps_ steps.
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ── MuJoCo model & data ───────────────────────────────────────────────────
  mjModel * m_{nullptr};
  mjData *  d_{nullptr};

  /// Path to the MJCF XML file.
  std::string model_path_;

  /// Number of mj_step() calls per ros2_control write() call.
  int n_substeps_{1};

  // ── Joint mapping ─────────────────────────────────────────────────────────
  std::vector<std::string> joint_names_;

  /// Index into mjData::qpos for each joint (m_->jnt_qposadr[jid]).
  std::vector<int> joint_qpos_idx_;

  /// Index into mjData::qvel for each joint (m_->jnt_dofadr[jid]).
  std::vector<int> joint_qvel_idx_;

  /// Index into mjData::ctrl for each joint's position actuator.
  /// -1 means no actuator was found for that joint.
  std::vector<int> act_idx_;

  // ── ros2_control state/command buffers ────────────────────────────────────
  std::vector<double> hw_commands_;    ///< commanded positions (rad)
  std::vector<double> hw_states_;      ///< feedback positions  (rad)
  std::vector<double> hw_velocities_;  ///< feedback velocities (rad/s)

  // ── Logger ────────────────────────────────────────────────────────────────
  std::shared_ptr<rclcpp::Logger> logger_;

  // ── Helpers ───────────────────────────────────────────────────────────────

  /// Free mjModel and mjData if they are allocated.
  void free_mujoco();
};

}  // namespace mujoco_arm_hardware
