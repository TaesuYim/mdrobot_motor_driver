// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
/// @file mdrobot_system.hpp
/// ros2_control SystemInterface for MDROBOT MD-series motor controllers.
///
/// Wraps the mdrobot_cpp Single/DualMotorDriver behind a pluginlib hardware
/// component. One controller, two device shapes:
///   - device_type=single -> 1 joint  (MD400, ...)
///   - device_type=dual   -> 2 joints (PNT50, MD400T, ...)
///
/// State interfaces (per joint): position, velocity, effort.
/// Command interfaces (per joint): velocity and/or position.
///
/// Units: a joint with a positive `counts_per_rev` parameter exports SI
/// (position=rad, velocity=rad/s) and accepts SI commands; without it the
/// joint stays raw (position=count, velocity=rpm). This mirrors the Python
/// node — the generic driver never hard-codes counts_per_rev.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "mdrobot_cpp/device.hpp"
#include "mdrobot_cpp/protocol.hpp"
#include "mdrobot_cpp/transport.hpp"

namespace mdrobot_ros2_control {

/// SystemInterface plugin. Single- and dual-channel via `device_type`.
class MdrobotSystemHardware : public hardware_interface::SystemInterface {
 public:
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

 private:
  /// Active command interface for a joint at any moment.
  enum class CmdMode { kNone, kVelocity, kPosition };

  /// Controller topology.
  ///   kSingle: one controller, one joint (MD400, ...).
  ///   kDual:   one two-channel controller, two joints (PNT50, MD400T, ...).
  ///   kTwin:   two single-channel controllers on ONE serial bus at DISTINCT
  ///            Modbus slave IDs, two joints (e.g. two MD400 for a skid-steer base).
  enum class DeviceType { kSingle, kDual, kTwin };

  /// Per-joint configuration parsed from the URDF.
  struct JointCfg {
    std::string name;
    double counts_per_rev = 0.0;  ///< >0 -> SI units; 0 -> raw (count, rpm).
    uint8_t slave_id = 1;         ///< Modbus slave id (single/twin: per joint).
    int direction = 1;            ///< +1, or -1 to invert a mirrored wheel (reverse param).
    bool has_velocity_cmd = false;
    bool has_position_cmd = false;
    bool has_position_state = false;
    bool has_velocity_state = false;
    bool has_effort_state = false;
    CmdMode mode = CmdMode::kNone;       ///< current active command interface.
    double last_position_cmd = 0.0;      ///< last issued position goal (cmd units).
    bool position_cmd_valid = false;     ///< a position goal has been issued.
  };

  /// Default mode when only one command interface is declared.
  static CmdMode default_mode(const JointCfg& j);
  /// Convert a joint's commanded velocity (cmd units) to signed motor rpm.
  int velocity_cmd_to_rpm(const JointCfg& j, double cmd) const;
  /// Convert a joint's commanded position (cmd units) to motor counts.
  int32_t position_cmd_to_counts(const JointCfg& j, double cmd) const;
  /// Push state into the exported interfaces for one joint from a Monitor.
  void publish_joint_state(const JointCfg& j, const mdrobot::Monitor& mon);
  /// Issue one joint's active velocity/position command on its own driver
  /// (single + each twin wheel). Mutates the joint's position bookkeeping.
  void write_single_joint(JointCfg& j, mdrobot::SingleMotorDriver& drv);
  /// Tear down the comm stack in dependency order (drivers -> dual -> clients ->
  /// transport). Idempotent — run at on_configure entry and from on_cleanup.
  void reset_comm();

  // --- configuration (from URDF <hardware> params) ---
  DeviceType device_type_ = DeviceType::kSingle;  ///< resolved in on_init.
  std::string port_ = "/dev/ttyUSB0";
  int baudrate_ = 19200;
  uint8_t slave_id_ = 1;     ///< hw-level motor_id (single/dual; twin uses per-joint).
  int use_limit_sw_ = -1;   ///< -1 leave as-is, 0 disable, 1 enable.
  bool auto_enable_ = true;
  int position_max_rpm_ = 100;
  double timeout_ = 0.3;
  int max_comm_errors_ = 5;  ///< consecutive read/write failures tolerated before ERROR.

  std::vector<JointCfg> joints_;

  // transient-fault tolerance: a single serial hiccup shouldn't deactivate the
  // hardware. Count consecutive failures; only surface ERROR past the threshold.
  int read_errors_ = 0;
  int write_errors_ = 0;

  // --- communication stack (built in on_configure) ---
  // Declaration order is load-bearing: a SingleMotorDriver references a
  // ModbusClient which references the SerialTransport, so destruction (reverse of
  // declaration) must run drivers_ -> dual_ -> clients_ -> transport_. The
  // unique_ptr indirection keeps each DriverBase's ModbusClient& stable across
  // vector growth — do NOT flatten clients_/drivers_ to by-value vectors.
  std::unique_ptr<mdrobot::SerialTransport> transport_;
  std::vector<std::unique_ptr<mdrobot::ModbusClient>> clients_;   ///< one per controller.
  std::vector<std::unique_ptr<mdrobot::SingleMotorDriver>> drivers_;  ///< single/twin.
  std::unique_ptr<mdrobot::DualMotorDriver> dual_;                ///< dual only (uses clients_[0]).
};

}  // namespace mdrobot_ros2_control
