// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
/// @file mdrobot_system.cpp
/// Implementation of MdrobotSystemHardware.

#include "mdrobot_ros2_control/mdrobot_system.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

#include "mdrobot_cpp/exceptions.hpp"
#include "mdrobot_cpp/registers.hpp"
#include "mdrobot_cpp/units.hpp"

namespace mdrobot_ros2_control {

namespace {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

constexpr char kLogger[] = "MdrobotSystemHardware";

/// Look up a parameter from a string map, with a default.
std::string param_or(
    const std::unordered_map<std::string, std::string>& params,
    const std::string& key, const std::string& fallback) {
  auto it = params.find(key);
  return it != params.end() ? it->second : fallback;
}

bool has_interface(const std::vector<hardware_interface::InterfaceInfo>& ifaces,
                   const std::string& name) {
  for (const auto& i : ifaces) {
    if (i.name == name) return true;
  }
  return false;
}

}  // namespace

CallbackReturn MdrobotSystemHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto& info = get_hardware_info();
  const auto& hp = info.hardware_parameters;

  try {
    port_ = param_or(hp, "port", port_);
    baudrate_ = std::stoi(param_or(hp, "baudrate", std::to_string(baudrate_)));
    slave_id_ = static_cast<uint8_t>(
        std::stoi(param_or(hp, "motor_id", std::to_string(slave_id_))));
    use_limit_sw_ =
        std::stoi(param_or(hp, "use_limit_sw", std::to_string(use_limit_sw_)));
    auto_enable_ = param_or(hp, "auto_enable", "true") != "false";
    position_max_rpm_ = std::stoi(
        param_or(hp, "position_max_rpm", std::to_string(position_max_rpm_)));
    timeout_ = std::stod(param_or(hp, "timeout", std::to_string(timeout_)));
    max_comm_errors_ = std::stoi(
        param_or(hp, "max_comm_errors", std::to_string(max_comm_errors_)));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "bad hardware parameter: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // device_type may be given explicitly, else inferred from the joint count.
  std::string device_type = param_or(hp, "device_type", "");
  const std::size_t n = info.joints.size();
  if (n != 1 && n != 2) {
    RCLCPP_ERROR(get_logger(),
                 "expected 1 (single) or 2 (dual) joints, got %zu", n);
    return CallbackReturn::ERROR;
  }
  if ((device_type == "single" && n != 1) ||
      (device_type == "dual" && n != 2)) {
    RCLCPP_ERROR(get_logger(),
                 "device_type=%s but %zu joint(s) declared",
                 device_type.c_str(), n);
    return CallbackReturn::ERROR;
  }

  joints_.clear();
  for (const auto& j : info.joints) {
    JointCfg cfg;
    cfg.name = j.name;
    cfg.has_velocity_cmd =
        has_interface(j.command_interfaces, hardware_interface::HW_IF_VELOCITY);
    cfg.has_position_cmd =
        has_interface(j.command_interfaces, hardware_interface::HW_IF_POSITION);
    cfg.has_position_state =
        has_interface(j.state_interfaces, hardware_interface::HW_IF_POSITION);
    cfg.has_velocity_state =
        has_interface(j.state_interfaces, hardware_interface::HW_IF_VELOCITY);
    cfg.has_effort_state =
        has_interface(j.state_interfaces, hardware_interface::HW_IF_EFFORT);

    if (!cfg.has_velocity_cmd && !cfg.has_position_cmd) {
      RCLCPP_ERROR(get_logger(),
                   "joint '%s' declares no velocity/position command interface",
                   j.name.c_str());
      return CallbackReturn::ERROR;
    }
    try {
      cfg.counts_per_rev = std::stod(param_or(j.parameters, "counts_per_rev", "0"));
    } catch (const std::exception&) {
      cfg.counts_per_rev = 0.0;
    }
    cfg.mode = default_mode(cfg);
    joints_.push_back(cfg);
  }

  RCLCPP_INFO(get_logger(),
              "init: port=%s baud=%d id=%d type=%s joints=%zu",
              port_.c_str(), baudrate_, slave_id_,
              is_dual() ? "dual" : "single", joints_.size());
  for (const auto& j : joints_) {
    if (j.counts_per_rev > 0.0) {
      RCLCPP_INFO(get_logger(), "  joint '%s' SI units, counts_per_rev=%.3f",
                  j.name.c_str(), j.counts_per_rev);
    } else {
      RCLCPP_WARN(get_logger(),
                  "  joint '%s' raw units (count, rpm) — set counts_per_rev for SI",
                  j.name.c_str());
    }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MdrobotSystemHardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  try {
    transport_ = std::make_unique<mdrobot::SerialTransport>(port_, baudrate_,
                                                            timeout_);
    client_ = std::make_unique<mdrobot::ModbusClient>(*transport_, slave_id_);
    if (is_dual()) {
      dual_ = std::make_unique<mdrobot::DualMotorDriver>(*client_);
    } else {
      single_ = std::make_unique<mdrobot::SingleMotorDriver>(*client_);
    }

    // First transaction after open can be noisy; retry with ping.
    bool up = false;
    for (int attempt = 0; attempt < 5 && !up; ++attempt) {
      up = is_dual() ? dual_->ping() : single_->ping();
    }
    if (!up) {
      RCLCPP_ERROR(get_logger(),
                   "%s: initial communication failed — baudrate / port / wiring",
                   port_.c_str());
      return CallbackReturn::ERROR;
    }

    auto& base = is_dual() ? static_cast<mdrobot::DriverBase&>(*dual_)
                           : static_cast<mdrobot::DriverBase&>(*single_);
    RCLCPP_INFO(get_logger(), "opened %s: version=%d voltage=%.1fV",
                port_.c_str(), base.get_version(), base.get_voltage());

    // USE_LIMIT_SW policy (some controllers need 0 for serial drive).
    if (use_limit_sw_ >= 0) {
      uint16_t v = use_limit_sw_ ? 1 : 0;
      client_->write_register(mdrobot::PID_USE_LIMIT_SW, v);
      if (is_dual()) {
        client_->write_register(mdrobot::PID_USE_LIMIT_SW2, v);
      }
      RCLCPP_INFO(get_logger(), "USE_LIMIT_SW set to %u", v);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "on_configure failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MdrobotSystemHardware::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  dual_.reset();
  single_.reset();
  client_.reset();
  transport_.reset();  // closes the port via RAII.
  return CallbackReturn::SUCCESS;
}

CallbackReturn MdrobotSystemHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  try {
    if (auto_enable_) {
      if (is_dual()) {
        dual_->enable();
      } else {
        single_->enable();
      }
      RCLCPP_INFO(get_logger(), "enabled (UI_COM=1 + START_STOP arm)");
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "on_activate failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  for (auto& j : joints_) {
    j.position_cmd_valid = false;
    j.mode = default_mode(j);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MdrobotSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Stop, then drop torque — always proceed even on error.
  try {
    if (is_dual()) {
      dual_->stop();
      dual_->torque_off_both();
    } else {
      single_->stop();
      single_->torque_off();
    }
    RCLCPP_INFO(get_logger(), "deactivate: stop + torque_off");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "on_deactivate stop failed (ignored): %s",
                 e.what());
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MdrobotSystemHardware::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  try {
    if (is_dual() && dual_) {
      dual_->torque_off_both();
    } else if (single_) {
      single_->torque_off();
    }
  } catch (const std::exception&) {
    // best-effort safety stop.
  }
  return CallbackReturn::SUCCESS;
}

MdrobotSystemHardware::CmdMode MdrobotSystemHardware::default_mode(
    const JointCfg& j) {
  if (j.has_velocity_cmd && !j.has_position_cmd) return CmdMode::kVelocity;
  if (j.has_position_cmd && !j.has_velocity_cmd) return CmdMode::kPosition;
  return CmdMode::kNone;  // both declared -> wait for a controller to claim one.
}

return_type MdrobotSystemHardware::prepare_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/) {
  // Any velocity/position combination this hardware exports is acceptable.
  return return_type::OK;
}

return_type MdrobotSystemHardware::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  for (auto& j : joints_) {
    const std::string vel = j.name + "/" + hardware_interface::HW_IF_VELOCITY;
    const std::string pos = j.name + "/" + hardware_interface::HW_IF_POSITION;
    for (const auto& s : stop_interfaces) {
      if (s == vel && j.mode == CmdMode::kVelocity) j.mode = CmdMode::kNone;
      if (s == pos && j.mode == CmdMode::kPosition) j.mode = CmdMode::kNone;
    }
    for (const auto& s : start_interfaces) {
      if (s == vel) j.mode = CmdMode::kVelocity;
      if (s == pos) {
        j.mode = CmdMode::kPosition;
        j.position_cmd_valid = false;  // re-issue the goal on the next write.
      }
    }
  }
  return return_type::OK;
}

int MdrobotSystemHardware::velocity_cmd_to_rpm(const JointCfg& j,
                                               double cmd) const {
  double rpm = j.counts_per_rev > 0.0 ? mdrobot::rad_s_to_rpm(cmd) : cmd;
  return static_cast<int>(std::lround(rpm));
}

int32_t MdrobotSystemHardware::position_cmd_to_counts(const JointCfg& j,
                                                      double cmd) const {
  if (j.counts_per_rev > 0.0) {
    return static_cast<int32_t>(mdrobot::rad_to_counts(cmd, j.counts_per_rev));
  }
  return static_cast<int32_t>(std::lround(cmd));
}

void MdrobotSystemHardware::publish_joint_state(const JointCfg& j,
                                                const mdrobot::Monitor& mon) {
  const bool si = j.counts_per_rev > 0.0;
  if (j.has_position_state) {
    double p = si ? mdrobot::counts_to_rad(mon.position, j.counts_per_rev)
                  : static_cast<double>(mon.position);
    set_state(j.name + "/" + hardware_interface::HW_IF_POSITION, p);
  }
  if (j.has_velocity_state) {
    double v = si ? mdrobot::rpm_to_rad_s(mon.speed_rpm)
                  : static_cast<double>(mon.speed_rpm);
    set_state(j.name + "/" + hardware_interface::HW_IF_VELOCITY, v);
  }
  if (j.has_effort_state) {
    // Raw motor current (A) as an effort proxy — not calibrated torque.
    set_state(j.name + "/" + hardware_interface::HW_IF_EFFORT,
              mon.current_a.value_or(0.0));
  }
}

return_type MdrobotSystemHardware::read(const rclcpp::Time& /*time*/,
                                        const rclcpp::Duration& /*period*/) {
  try {
    if (is_dual()) {
      // read_main_data (PNT_MAIN_DATA) carries current, so effort is real;
      // PNT_MONITOR would report current=0.
      mdrobot::DualMonitor mon = dual_->read_main_data();
      publish_joint_state(joints_[0], mon.motor1);
      publish_joint_state(joints_[1], mon.motor2);
    } else {
      publish_joint_state(joints_[0], single_->read_monitor());
    }
  } catch (const std::exception& e) {
    if (++read_errors_ >= max_comm_errors_) {
      RCLCPP_ERROR(rclcpp::get_logger(kLogger),
                   "read failed %d times in a row: %s", read_errors_, e.what());
      return return_type::ERROR;
    }
    RCLCPP_WARN(rclcpp::get_logger(kLogger), "read failed (%d/%d): %s",
                read_errors_, max_comm_errors_, e.what());
    return return_type::OK;  // keep last state; tolerate a transient hiccup.
  }
  read_errors_ = 0;
  return return_type::OK;
}

return_type MdrobotSystemHardware::write(const rclcpp::Time& /*time*/,
                                         const rclcpp::Duration& /*period*/) {
  try {
    if (is_dual()) {
      // Both channels share the device; use joint[0] to pick the device mode.
      CmdMode mode = joints_[0].mode;
      if (mode == CmdMode::kVelocity) {
        double c0 = get_command<double>(
            joints_[0].name + "/" + hardware_interface::HW_IF_VELOCITY);
        double c1 = get_command<double>(
            joints_[1].name + "/" + hardware_interface::HW_IF_VELOCITY);
        if (std::isnan(c0)) c0 = 0.0;
        if (std::isnan(c1)) c1 = 0.0;
        dual_->set_velocities(velocity_cmd_to_rpm(joints_[0], c0),
                              velocity_cmd_to_rpm(joints_[1], c1));
      } else if (mode == CmdMode::kPosition) {
        double c0 = get_command<double>(
            joints_[0].name + "/" + hardware_interface::HW_IF_POSITION);
        double c1 = get_command<double>(
            joints_[1].name + "/" + hardware_interface::HW_IF_POSITION);
        if (std::isnan(c0) || std::isnan(c1)) return return_type::OK;
        const bool changed = !joints_[0].position_cmd_valid ||
                             c0 != joints_[0].last_position_cmd ||
                             c1 != joints_[1].last_position_cmd;
        if (changed) {
          dual_->move_to_both(position_cmd_to_counts(joints_[0], c0),
                              position_cmd_to_counts(joints_[1], c1),
                              position_max_rpm_);
          joints_[0].last_position_cmd = c0;
          joints_[1].last_position_cmd = c1;
          joints_[0].position_cmd_valid = true;
        }
      }
    } else {
      JointCfg& j = joints_[0];
      if (j.mode == CmdMode::kVelocity) {
        double c = get_command<double>(
            j.name + "/" + hardware_interface::HW_IF_VELOCITY);
        if (std::isnan(c)) c = 0.0;
        single_->set_velocity(velocity_cmd_to_rpm(j, c));
      } else if (j.mode == CmdMode::kPosition) {
        double c = get_command<double>(
            j.name + "/" + hardware_interface::HW_IF_POSITION);
        if (std::isnan(c)) return return_type::OK;
        if (!j.position_cmd_valid || c != j.last_position_cmd) {
          single_->move_to(position_cmd_to_counts(j, c), position_max_rpm_);
          j.last_position_cmd = c;
          j.position_cmd_valid = true;
        }
      }
    }
  } catch (const std::exception& e) {
    if (++write_errors_ >= max_comm_errors_) {
      RCLCPP_ERROR(rclcpp::get_logger(kLogger),
                   "write failed %d times in a row: %s", write_errors_, e.what());
      return return_type::ERROR;
    }
    RCLCPP_WARN(rclcpp::get_logger(kLogger), "write failed (%d/%d): %s",
                write_errors_, max_comm_errors_, e.what());
    return return_type::OK;  // tolerate a transient hiccup.
  }
  write_errors_ = 0;
  return return_type::OK;
}

}  // namespace mdrobot_ros2_control

PLUGINLIB_EXPORT_CLASS(mdrobot_ros2_control::MdrobotSystemHardware,
                       hardware_interface::SystemInterface)
