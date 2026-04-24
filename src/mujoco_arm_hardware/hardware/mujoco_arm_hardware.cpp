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

#include "mujoco_arm_hardware/mujoco_arm_hardware.hpp"

#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace mujoco_arm_hardware
{

// ── on_init ──────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn MujocoArmHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("MujocoArmHardware"));

  // ── Required parameter: model_path ──────────────────────────────────────
  auto it = info_.hardware_parameters.find("model_path");
  if (it == info_.hardware_parameters.end()) {
    RCLCPP_FATAL(*logger_, "'model_path' hardware parameter is required");
    return hardware_interface::CallbackReturn::ERROR;
  }
  model_path_ = it->second;

  // ── Optional parameter: n_substeps ──────────────────────────────────────
  auto it_ns = info_.hardware_parameters.find("n_substeps");
  if (it_ns != info_.hardware_parameters.end()) {
    try {
      n_substeps_ = std::stoi(it_ns->second);
      if (n_substeps_ < 1) {
        n_substeps_ = 1;
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(*logger_, "Invalid n_substeps value '%s': %s — using 1",
        it_ns->second.c_str(), e.what());
      n_substeps_ = 1;
    }
  }

  // ── Build joint name list from HardwareInfo ──────────────────────────────
  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);
  }

  const std::size_t n = joint_names_.size();
  hw_commands_.assign(n, 0.0);
  hw_states_.assign(n, 0.0);
  hw_velocities_.assign(n, 0.0);
  joint_qpos_idx_.assign(n, -1);
  joint_qvel_idx_.assign(n, -1);
  act_idx_.assign(n, -1);

  RCLCPP_INFO(*logger_,
    "MujocoArmHardware: %zu joints, model='%s', n_substeps=%d",
    n, model_path_.c_str(), n_substeps_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_configure ─────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn MujocoArmHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(*logger_, "Loading MuJoCo model: %s", model_path_.c_str());

  char error_buf[1000] = "";
  m_ = mj_loadXML(model_path_.c_str(), nullptr, error_buf, sizeof(error_buf));
  if (!m_) {
    RCLCPP_FATAL(*logger_, "mj_loadXML failed: %s", error_buf);
    return hardware_interface::CallbackReturn::ERROR;
  }

  d_ = mj_makeData(m_);
  if (!d_) {
    RCLCPP_FATAL(*logger_, "mj_makeData failed");
    mj_deleteModel(m_);
    m_ = nullptr;
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ── Map joint names → qpos/qvel indices ─────────────────────────────────
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const int jid = mj_name2id(m_, mjOBJ_JOINT, joint_names_[i].c_str());
    if (jid < 0) {
      RCLCPP_FATAL(*logger_,
        "Joint '%s' not found in MuJoCo model", joint_names_[i].c_str());
      free_mujoco();
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_qpos_idx_[i] = m_->jnt_qposadr[jid];
    joint_qvel_idx_[i] = m_->jnt_dofadr[jid];

    // Actuator convention: "act_<joint_name>"
    const std::string act_name = "act_" + joint_names_[i];
    const int aid = mj_name2id(m_, mjOBJ_ACTUATOR, act_name.c_str());
    if (aid < 0) {
      RCLCPP_WARN(*logger_,
        "Actuator '%s' not found — joint '%s' will not be driven",
        act_name.c_str(), joint_names_[i].c_str());
    }
    act_idx_[i] = aid;
  }

  // ── Apply initial positions from ros2_control xacro initial_value ────────
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    for (const auto & si : info_.joints[i].state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION && !si.initial_value.empty()) {
        try {
          d_->qpos[joint_qpos_idx_[i]] = std::stod(si.initial_value);
        } catch (const std::exception & e) {
          RCLCPP_WARN(*logger_,
            "Could not parse initial_value for joint '%s': %s",
            joint_names_[i].c_str(), e.what());
        }
        break;
      }
    }
  }
  // Recompute forward kinematics at the newly set initial configuration
  mj_forward(m_, d_);

  RCLCPP_INFO(*logger_,
    "MuJoCo model loaded: %d joints, %d actuators, timestep=%.4f s",
    m_->njnt, m_->nu, m_->opt.timestep);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_activate ──────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn MujocoArmHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Seed command buffers from current sim state so controllers start without jumps
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    hw_states_[i]     = d_->qpos[joint_qpos_idx_[i]];
    hw_velocities_[i] = d_->qvel[joint_qvel_idx_[i]];
    hw_commands_[i]   = hw_states_[i];
    if (act_idx_[i] >= 0) {
      d_->ctrl[act_idx_[i]] = hw_commands_[i];
    }
  }
  RCLCPP_INFO(*logger_, "MujocoArmHardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_deactivate ────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn MujocoArmHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(*logger_, "MujocoArmHardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_cleanup ───────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn MujocoArmHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  free_mujoco();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_shutdown ──────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn MujocoArmHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  free_mujoco();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── export_state_interfaces ───────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface>
MujocoArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;
  ifaces.reserve(joint_names_.size() * 2);
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    ifaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_states_[i]);
    ifaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return ifaces;
}

// ── export_command_interfaces ─────────────────────────────────────────────────

std::vector<hardware_interface::CommandInterface>
MujocoArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifaces;
  ifaces.reserve(joint_names_.size());
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    ifaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return ifaces;
}

// ── read ─────────────────────────────────────────────────────────────────────

hardware_interface::return_type MujocoArmHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!m_ || !d_) {
    return hardware_interface::return_type::ERROR;
  }
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    hw_states_[i]     = d_->qpos[joint_qpos_idx_[i]];
    hw_velocities_[i] = d_->qvel[joint_qvel_idx_[i]];
  }
  return hardware_interface::return_type::OK;
}

// ── write ────────────────────────────────────────────────────────────────────

hardware_interface::return_type MujocoArmHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!m_ || !d_) {
    return hardware_interface::return_type::ERROR;
  }

  // Copy position commands to MuJoCo actuator ctrl inputs
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    if (act_idx_[i] >= 0) {
      d_->ctrl[act_idx_[i]] = hw_commands_[i];
    }
  }

  // Advance the simulation by n_substeps_ steps.
  // With MJCF timestep T and controller update period P, set n_substeps_ = P/T
  // so that simulation time advances by exactly one control period per call.
  for (int step = 0; step < n_substeps_; ++step) {
    mj_step(m_, d_);
  }

  return hardware_interface::return_type::OK;
}

// ── free_mujoco ───────────────────────────────────────────────────────────────

void MujocoArmHardware::free_mujoco()
{
  if (d_) {
    mj_deleteData(d_);
    d_ = nullptr;
  }
  if (m_) {
    mj_deleteModel(m_);
    m_ = nullptr;
  }
}

}  // namespace mujoco_arm_hardware

PLUGINLIB_EXPORT_CLASS(
  mujoco_arm_hardware::MujocoArmHardware,
  hardware_interface::SystemInterface)
