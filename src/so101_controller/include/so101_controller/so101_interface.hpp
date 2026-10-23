#ifndef SO101_INTERFACE_H
#define SO101_INTERFACE_H

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "so101_controller/feetech_bus.hpp"
#include "so101_controller/joint_calibration.hpp"
#include "so101_controller/libserial_transport.hpp"

namespace so101_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// ros2_control SystemInterface for the SO-101 arm (6x Feetech STS3215) with closed-loop
// position feedback. Joints are matched to servos by name (id / calibration come from the
// <joint><param/> tags in so101_ros2_control.xacro), so the order of joints does not matter.
class So101Interface : public hardware_interface::SystemInterface
{
public:
  So101Interface();
  ~So101Interface() override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & hardware_info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool pollServos();
  void checkHealth();
  bool enableTorque(bool enable);
  static bool throttled(std::chrono::steady_clock::time_point & last, int period_ms);

  // --- hardware parameters (xacro <hardware><param/>)
  std::string port_;
  int baud_rate_ = 115200;  // USB link to the ESP32 (not the servo bus)
  bool read_only_ = false;
  bool bus_echo_ = false;
  bool disable_torque_on_deactivate_ = false;
  double startup_delay_s_ = 2.0;
  int timeout_ms_ = 20;
  bool read_health_ = false;   // also read voltage/temperature (costs USB bandwidth)
  uint8_t read_len_ = 2;
  int acceleration_ = 254;
  int max_consecutive_failures_ = 20;

  // --- per joint (same order as info_.joints)
  std::vector<JointCalibration> cal_;
  std::vector<uint8_t> ids_;
  std::vector<double> position_commands_;
  std::vector<double> position_states_;
  std::vector<double> last_valid_command_;
  std::vector<uint8_t> voltage_;      // 0.1 V
  std::vector<uint8_t> temperature_;  // deg C
  std::vector<uint8_t> servo_error_;  // STS error flag byte
  std::vector<ReadResult> results_;

  std::unique_ptr<LibSerialTransport> transport_;
  std::unique_ptr<FeetechBus> bus_;
  int consecutive_failures_ = 0;
  std::chrono::steady_clock::time_point last_comm_warn_{};
  std::chrono::steady_clock::time_point last_health_warn_{};
};

}  // namespace so101_controller

#endif  // SO101_INTERFACE_H
