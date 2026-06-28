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

  // Resolve controller topology. device_type may be given explicitly; an empty
  // value is inferred from the joint count for backward compat (1 -> single,
  // 2 -> dual). "twin" can NEVER be inferred (it also has 2 joints) and must be
  // requested explicitly. Any other value is rejected (a typo must not silently
  // degrade to dual).
  const std::string device_type = param_or(hp, "device_type", "");
  const std::size_t n = info.joints.size();
  if (n != 1 && n != 2) {
    RCLCPP_ERROR(get_logger(),
                 "expected 1 (single) or 2 (dual/twin) joints, got %zu", n);
    return CallbackReturn::ERROR;
  }
  if (device_type.empty()) {
    device_type_ = (n == 1) ? DeviceType::kSingle : DeviceType::kDual;
  } else if (device_type == "single") {
    if (n != 1) {
      RCLCPP_ERROR(get_logger(), "device_type=single needs 1 joint, got %zu", n);
      return CallbackReturn::ERROR;
    }
    device_type_ = DeviceType::kSingle;
  } else if (device_type == "dual") {
    if (n != 2) {
      RCLCPP_ERROR(get_logger(), "device_type=dual needs 2 joints, got %zu", n);
      return CallbackReturn::ERROR;
    }
    device_type_ = DeviceType::kDual;
  } else if (device_type == "twin") {
    if (n != 2) {
      RCLCPP_ERROR(get_logger(), "device_type=twin needs 2 joints, got %zu", n);
      return CallbackReturn::ERROR;
    }
    device_type_ = DeviceType::kTwin;
  } else {
    RCLCPP_ERROR(get_logger(),
                 "unknown device_type='%s' (expected single, dual, or twin)",
                 device_type.c_str());
    return CallbackReturn::ERROR;
  }

  joints_.clear();
  for (std::size_t i = 0; i < info.joints.size(); ++i) {
    const auto& j = info.joints[i];
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

    // Per-joint Modbus slave id. twin: each wheel is its own controller, so the
    // default is index+1 (distinct) rather than the shared hw motor_id; single
    // and dual default to the hw-level motor_id.
    const std::string default_sid = (device_type_ == DeviceType::kTwin)
                                        ? std::to_string(i + 1)
                                        : std::to_string(slave_id_);
    try {
      cfg.slave_id = static_cast<uint8_t>(
          std::stoi(param_or(j.parameters, "motor_id", default_sid)));
    } catch (const std::exception&) {
      cfg.slave_id = slave_id_;
    }
    // Reverse a mirrored wheel (skid-steer). The sign cannot live in
    // counts_per_rev (>0 is the SI-mode gate), so it is a separate ±1 factor
    // applied to both commands and feedback.
    cfg.direction =
        (param_or(j.parameters, "reverse", "false") == "true") ? -1 : 1;

    cfg.mode = default_mode(cfg);
    joints_.push_back(cfg);
  }

  // twin needs two DISTINCT, in-range slave ids — two controllers at the same id
  // collide on the bus, and a missing/duplicated param must not silently share one.
  if (device_type_ == DeviceType::kTwin) {
    for (const auto& j : joints_) {
      if (j.slave_id < 1 || j.slave_id > 253) {
        RCLCPP_ERROR(get_logger(), "joint '%s' motor_id=%u out of range (1-253)",
                     j.name.c_str(), j.slave_id);
        return CallbackReturn::ERROR;
      }
    }
    if (joints_[0].slave_id == joints_[1].slave_id) {
      RCLCPP_ERROR(get_logger(),
                   "twin requires a distinct motor_id per joint (both = %u)",
                   joints_[0].slave_id);
      return CallbackReturn::ERROR;
    }
  }

  const char* type_str = device_type_ == DeviceType::kDual   ? "dual"
                         : device_type_ == DeviceType::kTwin ? "twin"
                                                             : "single";
  RCLCPP_INFO(get_logger(), "init: port=%s baud=%d type=%s joints=%zu",
              port_.c_str(), baudrate_, type_str, joints_.size());
  for (const auto& j : joints_) {
    if (j.counts_per_rev > 0.0) {
      RCLCPP_INFO(get_logger(),
                  "  joint '%s' slave_id=%u direction=%+d SI counts_per_rev=%.3f",
                  j.name.c_str(), j.slave_id, j.direction, j.counts_per_rev);
    } else {
      RCLCPP_WARN(get_logger(),
                  "  joint '%s' slave_id=%u direction=%+d raw units (count, rpm)"
                  " — set counts_per_rev for SI",
                  j.name.c_str(), j.slave_id, j.direction);
    }
  }
  return CallbackReturn::SUCCESS;
}

void MdrobotSystemHardware::reset_comm() {
  // Dependency order: drivers/dual reference clients reference transport, so tear
  // down drivers_ -> dual_ -> clients_ -> transport_. Idempotent.
  drivers_.clear();
  dual_.reset();
  clients_.clear();
  transport_.reset();  // closes the port via RAII.
}

CallbackReturn MdrobotSystemHardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Clear any partial state from a prior (possibly failed) configure before
  // rebuilding: re-creating transport_ while old clients still hold a Transport&
  // would dangle, and pushing onto non-empty vectors would duplicate clients.
  reset_comm();
  try {
    transport_ = std::make_unique<mdrobot::SerialTransport>(port_, baudrate_,
                                                            timeout_);

    if (device_type_ == DeviceType::kDual) {
      clients_.push_back(
          std::make_unique<mdrobot::ModbusClient>(*transport_, slave_id_));
      dual_ = std::make_unique<mdrobot::DualMotorDriver>(*clients_.back());
    } else {
      // single + twin: one ModbusClient + SingleMotorDriver per joint, each on
      // its own slave id over the shared transport.
      for (const auto& j : joints_) {
        clients_.push_back(
            std::make_unique<mdrobot::ModbusClient>(*transport_, j.slave_id));
        drivers_.push_back(
            std::make_unique<mdrobot::SingleMotorDriver>(*clients_.back()));
      }
    }

    // First transaction after open can be noisy; retry with ping. EVERY
    // controller must answer — for twin, a single endpoint succeeding is not
    // enough (would hide a dead controller or an unset slave id).
    if (device_type_ == DeviceType::kDual) {
      bool up = false;
      for (int attempt = 0; attempt < 5 && !up; ++attempt) up = dual_->ping();
      if (!up) {
        RCLCPP_ERROR(get_logger(),
                     "%s: initial communication failed — baudrate / port / wiring",
                     port_.c_str());
        return CallbackReturn::ERROR;
      }
      RCLCPP_INFO(get_logger(), "opened %s: version=%d voltage=%.1fV",
                  port_.c_str(), dual_->get_version(), dual_->get_voltage());
    } else {
      for (std::size_t i = 0; i < drivers_.size(); ++i) {
        bool up = false;
        for (int attempt = 0; attempt < 5 && !up; ++attempt) up = drivers_[i]->ping();
        if (!up) {
          RCLCPP_ERROR(
              get_logger(),
              "%s: controller slave_id=%u (joint '%s') did not respond — baudrate"
              " / port / wiring / slave id not changed",
              port_.c_str(), joints_[i].slave_id, joints_[i].name.c_str());
          return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(get_logger(), "opened %s slave_id=%u: version=%d voltage=%.1fV",
                    port_.c_str(), joints_[i].slave_id, drivers_[i]->get_version(),
                    drivers_[i]->get_voltage());
      }
    }

    // USE_LIMIT_SW policy (some controllers need 0 for serial drive). twin writes
    // the single-channel PID to EACH controller; only dual has a channel-2 PID.
    if (use_limit_sw_ >= 0) {
      uint16_t v = use_limit_sw_ ? 1 : 0;
      if (device_type_ == DeviceType::kDual) {
        clients_[0]->write_register(mdrobot::PID_USE_LIMIT_SW, v);
        clients_[0]->write_register(mdrobot::PID_USE_LIMIT_SW2, v);
      } else {
        for (auto& c : clients_) c->write_register(mdrobot::PID_USE_LIMIT_SW, v);
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
  reset_comm();
  return CallbackReturn::SUCCESS;
}

CallbackReturn MdrobotSystemHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  try {
    if (auto_enable_) {
      if (device_type_ == DeviceType::kDual) {
        dual_->enable();
      } else {
        for (auto& d : drivers_) d->enable();
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
  if (device_type_ == DeviceType::kDual) {
    try {
      dual_->stop();
      dual_->torque_off_both();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "on_deactivate stop failed (ignored): %s",
                   e.what());
    }
  } else {
    // Per-driver try so one controller's failure still stops the other wheel.
    for (std::size_t i = 0; i < drivers_.size(); ++i) {
      try {
        drivers_[i]->stop();
        drivers_[i]->torque_off();
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(),
                     "on_deactivate slave_id=%u stop failed (ignored): %s",
                     joints_[i].slave_id, e.what());
      }
    }
  }
  RCLCPP_INFO(get_logger(), "deactivate: stop + torque_off");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MdrobotSystemHardware::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Best-effort safety stop. Per-driver try so one failure does not skip the
  // other controller; null guards because on_error can fire before on_configure
  // built the drivers.
  if (device_type_ == DeviceType::kDual) {
    if (dual_) {
      try {
        dual_->torque_off_both();
      } catch (const std::exception&) {
      }
    }
  } else {
    for (auto& d : drivers_) {
      if (!d) continue;
      try {
        d->torque_off();
      } catch (const std::exception&) {
      }
    }
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
  return j.direction * static_cast<int>(std::lround(rpm));
}

int32_t MdrobotSystemHardware::position_cmd_to_counts(const JointCfg& j,
                                                      double cmd) const {
  int32_t counts =
      j.counts_per_rev > 0.0
          ? static_cast<int32_t>(mdrobot::rad_to_counts(cmd, j.counts_per_rev))
          : static_cast<int32_t>(std::lround(cmd));
  return static_cast<int32_t>(j.direction) * counts;
}

void MdrobotSystemHardware::publish_joint_state(const JointCfg& j,
                                                const mdrobot::Monitor& mon) {
  // direction (reverse) is applied symmetrically to commands and feedback so a
  // mirrored wheel's odometry sign stays consistent with its commanded sign.
  const bool si = j.counts_per_rev > 0.0;
  if (j.has_position_state) {
    double p = si ? mdrobot::counts_to_rad(mon.position, j.counts_per_rev)
                  : static_cast<double>(mon.position);
    set_state(j.name + "/" + hardware_interface::HW_IF_POSITION, j.direction * p);
  }
  if (j.has_velocity_state) {
    double v = si ? mdrobot::rpm_to_rad_s(mon.speed_rpm)
                  : static_cast<double>(mon.speed_rpm);
    set_state(j.name + "/" + hardware_interface::HW_IF_VELOCITY, j.direction * v);
  }
  if (j.has_effort_state) {
    // Raw motor current (A) as an effort proxy — not calibrated torque. Current
    // magnitude is unsigned, so direction is not applied.
    set_state(j.name + "/" + hardware_interface::HW_IF_EFFORT,
              mon.current_a.value_or(0.0));
  }
}

return_type MdrobotSystemHardware::read(const rclcpp::Time& /*time*/,
                                        const rclcpp::Duration& /*period*/) {
  bool any_fail = false;
  std::string err;
  if (device_type_ == DeviceType::kDual) {
    try {
      // read_main_data (PNT_MAIN_DATA) carries current, so effort is real;
      // PNT_MONITOR would report current=0.
      mdrobot::DualMonitor mon = dual_->read_main_data();
      publish_joint_state(joints_[0], mon.motor1);
      publish_joint_state(joints_[1], mon.motor2);
    } catch (const std::exception& e) {
      any_fail = true;
      err = e.what();
    }
  } else {
    // single + twin: per-driver try so a hiccup on one controller still
    // services the other (single keeps last state for that joint).
    for (std::size_t i = 0; i < drivers_.size(); ++i) {
      try {
        publish_joint_state(joints_[i], drivers_[i]->read_monitor());
      } catch (const std::exception& e) {
        any_fail = true;
        err = e.what();
      }
    }
  }
  if (any_fail) {
    if (++read_errors_ >= max_comm_errors_) {
      RCLCPP_ERROR(rclcpp::get_logger(kLogger),
                   "read failed %d times in a row: %s", read_errors_, err.c_str());
      return return_type::ERROR;
    }
    RCLCPP_WARN(rclcpp::get_logger(kLogger), "read failed (%d/%d): %s",
                read_errors_, max_comm_errors_, err.c_str());
    return return_type::OK;  // keep last state; tolerate a transient hiccup.
  }
  read_errors_ = 0;
  return return_type::OK;
}

void MdrobotSystemHardware::write_single_joint(JointCfg& j,
                                               mdrobot::SingleMotorDriver& drv) {
  if (j.mode == CmdMode::kVelocity) {
    double c =
        get_command<double>(j.name + "/" + hardware_interface::HW_IF_VELOCITY);
    if (std::isnan(c)) c = 0.0;
    drv.set_velocity(velocity_cmd_to_rpm(j, c));
  } else if (j.mode == CmdMode::kPosition) {
    double c =
        get_command<double>(j.name + "/" + hardware_interface::HW_IF_POSITION);
    if (std::isnan(c)) return;
    if (!j.position_cmd_valid || c != j.last_position_cmd) {
      drv.move_to(position_cmd_to_counts(j, c), position_max_rpm_);
      j.last_position_cmd = c;
      j.position_cmd_valid = true;
    }
  }
}

return_type MdrobotSystemHardware::write(const rclcpp::Time& /*time*/,
                                         const rclcpp::Duration& /*period*/) {
  bool any_fail = false;
  std::string err;
  if (device_type_ == DeviceType::kDual) {
    try {
      // Both channels share the device; use joint[0] to pick the device mode and
      // emit one paired call (set_velocities / move_to_both).
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
        if (!std::isnan(c0) && !std::isnan(c1)) {
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
      }
    } catch (const std::exception& e) {
      any_fail = true;
      err = e.what();
    }
  } else {
    // single + twin: each joint commands its own driver independently (own mode,
    // own position bookkeeping).
    for (std::size_t i = 0; i < drivers_.size(); ++i) {
      try {
        write_single_joint(joints_[i], *drivers_[i]);
      } catch (const std::exception& e) {
        any_fail = true;
        err = e.what();
      }
    }
    // twin safety: if any controller failed this cycle, a moving skid-steer base
    // must not keep driving on the healthy wheel — best-effort stop all.
    if (any_fail && device_type_ == DeviceType::kTwin) {
      for (auto& d : drivers_) {
        try {
          d->stop();
        } catch (const std::exception&) {
        }
      }
    }
  }
  if (any_fail) {
    if (++write_errors_ >= max_comm_errors_) {
      RCLCPP_ERROR(rclcpp::get_logger(kLogger),
                   "write failed %d times in a row: %s", write_errors_, err.c_str());
      return return_type::ERROR;
    }
    RCLCPP_WARN(rclcpp::get_logger(kLogger), "write failed (%d/%d): %s",
                write_errors_, max_comm_errors_, err.c_str());
    return return_type::OK;  // tolerate a transient hiccup.
  }
  write_errors_ = 0;
  return return_type::OK;
}

}  // namespace mdrobot_ros2_control

PLUGINLIB_EXPORT_CLASS(mdrobot_ros2_control::MdrobotSystemHardware,
                       hardware_interface::SystemInterface)
