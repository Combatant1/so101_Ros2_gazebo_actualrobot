#include "so101_controller/so101_interface.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <thread>
#include <unordered_map>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace so101_controller
{
namespace
{
using ParamMap = std::unordered_map<std::string, std::string>;

rclcpp::Logger logger() {return rclcpp::get_logger("So101Interface");}

double paramD(const ParamMap & m, const std::string & key, double def)
{
  const auto it = m.find(key);
  if (it == m.end()) {return def;}
  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    return def;
  }
}

bool paramB(const ParamMap & m, const std::string & key, bool def)
{
  const auto it = m.find(key);
  if (it == m.end()) {return def;}
  std::string v = it->second;
  std::transform(v.begin(), v.end(), v.begin(), ::tolower);
  return v == "true" || v == "1" || v == "yes";
}

uint16_t u16(uint8_t lo, uint8_t hi) {return static_cast<uint16_t>(lo | (hi << 8));}
}  // namespace

So101Interface::So101Interface() = default;

So101Interface::~So101Interface()
{
  if (transport_) {
    try {
      transport_->close();
    } catch (...) {
    }
  }
}

CallbackReturn So101Interface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // ---- hardware-level parameters
  const auto & hp = info_.hardware_parameters;
  if (hp.find("port") == hp.end()) {
    RCLCPP_FATAL(logger(), "Hardware parameter 'port' is missing (e.g. /dev/ttyUSB0)");
    return CallbackReturn::ERROR;
  }
  port_ = hp.at("port");
  if (port_.empty()) {
    RCLCPP_FATAL(logger(),
      "Hardware parameter 'port' is empty. In the xacro use <param name=\"port\">/dev/ttyUSB0</param> "
      "(value in the element text, not a value=\"\" attribute).");
    return CallbackReturn::ERROR;
  }
  baud_rate_ = static_cast<int>(paramD(hp, "baud_rate", 115200.0));
  read_only_ = paramB(hp, "read_only", false);
  bus_echo_ = paramB(hp, "bus_echo", false);
  disable_torque_on_deactivate_ = paramB(hp, "disable_torque_on_deactivate", false);
  startup_delay_s_ = paramD(hp, "startup_delay", 2.0);
  timeout_ms_ = static_cast<int>(paramD(hp, "timeout_ms", 20.0));
  read_health_ = paramB(hp, "read_health", false);
  read_len_ = read_health_ ? sts3215::PRESENT_BLOCK_LEN : 2;
  acceleration_ = std::min(254, std::max(0, static_cast<int>(paramD(hp, "acceleration", 254.0))));
  max_consecutive_failures_ =
    std::max(1, static_cast<int>(paramD(hp, "max_consecutive_failures", 20.0)));

  // per joint parameters
  const size_t n = info_.joints.size();
  cal_.clear();
  ids_.clear();
  std::set<int> seen_ids;
  for (size_t i = 0; i < n; ++i) {
    const auto & joint = info_.joints[i];

    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(logger(), "Joint '%s' must have exactly one 'position' command interface",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 1 ||
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(logger(), "Joint '%s' must have exactly one 'position' state interface",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    const auto & p = joint.parameters;
    for (const char * key : {"id", "zero_tick", "min_tick", "max_tick"}) {
      if (p.find(key) == p.end()) {
        RCLCPP_FATAL(logger(), "Joint '%s' is missing required <param name=\"%s\">",
          joint.name.c_str(), key);
        return CallbackReturn::ERROR;
      }
    }

    JointCalibration c;
    const int id = static_cast<int>(paramD(p, "id", 0.0));
    if (id < 1 || id > 253 || !seen_ids.insert(id).second) {
      RCLCPP_FATAL(logger(), "Joint '%s' has an invalid or duplicate servo id %d",
        joint.name.c_str(), id);
      return CallbackReturn::ERROR;
    }
    c.id = static_cast<uint8_t>(id);
    c.zero_tick = paramD(p, "zero_tick", 2048.0);
    c.min_tick = paramD(p, "min_tick", 0.0);
    c.max_tick = paramD(p, "max_tick", 4095.0);
    c.direction = paramD(p, "direction", 1.0) < 0.0 ? -1.0 : 1.0;
    c.torque_limit = p.find("torque_limit") != p.end() ?
      static_cast<int>(paramD(p, "torque_limit", 1000.0)) : -1;
    if (c.min_tick >= c.max_tick) {
      RCLCPP_FATAL(logger(), "Joint '%s': min_tick must be < max_tick", joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    // Extra software limits taken from the URDF <command_interface> min/max.
    const auto & ci = joint.command_interfaces[0];
    try {
      if (!ci.min.empty()) {c.min_rad = std::stod(ci.min);}
      if (!ci.max.empty()) {c.max_rad = std::stod(ci.max);}
    } catch (const std::exception &) {
      RCLCPP_WARN(logger(), "Joint '%s': could not parse command min/max, ignoring",
        joint.name.c_str());
    }

    cal_.push_back(c);
    ids_.push_back(c.id);
  }

  position_commands_.assign(n, 0.0);
  position_states_.assign(n, 0.0);
  last_valid_command_.assign(n, 0.0);
  voltage_.assign(n, 0);
  temperature_.assign(n, 0);
  servo_error_.assign(n, 0);

  RCLCPP_INFO_STREAM(logger(), "So101Interface initialised: " << n << " joints, port " << port_
                                                              << ", " << baud_rate_ << " baud"
                                                              << (read_only_ ? ", READ-ONLY" : ""));
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> So101Interface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> So101Interface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]);
  }
  return interfaces;
}

CallbackReturn So101Interface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger(), "Connecting to %s ...", port_.c_str());
  transport_ = std::make_unique<LibSerialTransport>();
  try {
    transport_->open(port_, baud_rate_);
  } catch (const std::exception & e) {
    RCLCPP_FATAL_STREAM(logger(), "Could not open " << port_ << ": " << e.what());
    transport_.reset();
    return CallbackReturn::FAILURE;
  }
  bus_ = std::make_unique<FeetechBus>(*transport_, timeout_ms_, bus_echo_);

  // Opening the port resets most ESP32 boards (DTR/RTS) - give the firmware time to boot
  // and discard the boot-loader chatter before talking to the servos.
  std::this_thread::sleep_for(std::chrono::duration<double>(startup_delay_s_));
  try {
    transport_->flush_input();

    // 1. Every servo must answer.
    for (const auto & c : cal_) {
      bool alive = false;
      for (int attempt = 0; attempt < 5 && !alive; ++attempt) {
        alive = bus_->ping(c.id);
        if (!alive) {std::this_thread::sleep_for(std::chrono::milliseconds(100));}
      }
      if (!alive) {
        RCLCPP_FATAL(logger(),
          "Servo id %d did not answer. Check power, wiring, ESP32 firmware and baud rate.", c.id);
        return CallbackReturn::FAILURE;
      }
    }

    // 2. Position-servo mode expected (LeRobot calibration sets this).
    for (size_t i = 0; i < cal_.size(); ++i) {
      std::vector<uint8_t> mode;
      if (!bus_->readRegister(cal_[i].id, sts3215::REG_OPERATING_MODE, 1, mode)) {
        RCLCPP_FATAL(logger(), "Could not read operating mode of servo %d", cal_[i].id);
        return CallbackReturn::FAILURE;
      }
      if (mode[0] != 0) {
        RCLCPP_FATAL(logger(),
          "Servo %d (%s) is not in position mode (mode=%d). Re-run lerobot calibration.",
          cal_[i].id, info_.joints[i].name.c_str(), mode[0]);
        return CallbackReturn::FAILURE;
      }
    }

    // 3. Read the real pose; commands start AT the measured pose so nothing jumps.
    bool ok = false;
    for (int attempt = 0; attempt < 5 && !ok; ++attempt) {ok = pollServos();}
    if (!ok) {
      RCLCPP_FATAL(logger(), "Could not read the initial servo positions");
      return CallbackReturn::FAILURE;
    }
    for (size_t i = 0; i < cal_.size(); ++i) {
      position_commands_[i] = position_states_[i];
      last_valid_command_[i] = position_states_[i];
      RCLCPP_INFO(logger(), "  %-14s id=%d  %7.3f rad (%7.1f deg)", info_.joints[i].name.c_str(),
        cal_[i].id, position_states_[i], position_states_[i] * 180.0 / kPi);
    }

    // 4. Arm the servos: acceleration, goal = present position, then torque on.
    if (!read_only_) {
      for (size_t i = 0; i < cal_.size(); ++i) {
        const uint16_t present = radToTicks(cal_[i], position_states_[i]);
        const std::vector<uint8_t> block = {
          static_cast<uint8_t>(acceleration_),
          static_cast<uint8_t>(present & 0xFF), static_cast<uint8_t>(present >> 8),
          0, 0,   // goal time
          0, 0};  // goal speed (0 = no limit)
        if (!bus_->writeRegister(cal_[i].id, sts3215::REG_ACCELERATION, block)) {
          RCLCPP_FATAL(logger(), "Failed to configure servo %d", cal_[i].id);
          return CallbackReturn::FAILURE;
        }
        if (cal_[i].torque_limit >= 0) {
          const uint16_t lim = static_cast<uint16_t>(std::min(cal_[i].torque_limit, 1000));
          if (!bus_->writeRegister(cal_[i].id, sts3215::REG_TORQUE_LIMIT,
            {static_cast<uint8_t>(lim & 0xFF), static_cast<uint8_t>(lim >> 8)}))
          {
            RCLCPP_FATAL(logger(), "Failed to set torque limit of servo %d", cal_[i].id);
            return CallbackReturn::FAILURE;
          }
        }
      }
      if (!enableTorque(true)) {
        RCLCPP_FATAL(logger(), "Failed to enable torque");
        return CallbackReturn::FAILURE;
      }
      RCLCPP_INFO(logger(), "Torque enabled, closed-loop control active");
    } else {
      RCLCPP_WARN(logger(), "READ-ONLY mode: torque stays off, no commands are sent");
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL_STREAM(logger(), "Serial error during activation: " << e.what());
    return CallbackReturn::FAILURE;
  }

  consecutive_failures_ = 0;
  return CallbackReturn::SUCCESS;
}

CallbackReturn So101Interface::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (bus_ && !read_only_ && disable_torque_on_deactivate_) {
    try {
      enableTorque(false);
      RCLCPP_INFO(logger(), "Torque disabled");
    } catch (const std::exception & e) {
      RCLCPP_WARN_STREAM(logger(), "Could not disable torque: " << e.what());
    }
  }
  bus_.reset();
  if (transport_) {
    try {
      transport_->close();
    } catch (...) {
    }
    transport_.reset();
  }
  RCLCPP_INFO(logger(), "Serial port closed");
  return CallbackReturn::SUCCESS;
}

bool So101Interface::enableTorque(bool enable)
{
  bool all_ok = true;
  for (const auto & c : cal_) {
    all_ok &= bus_->writeRegister(c.id, sts3215::REG_TORQUE_ENABLE, {static_cast<uint8_t>(enable)});
  }
  return all_ok;
}

bool So101Interface::throttled(std::chrono::steady_clock::time_point & last, int period_ms)
{
  const auto now = std::chrono::steady_clock::now();
  if (now - last < std::chrono::milliseconds(period_ms)) {return false;}
  last = now;
  return true;
}

// Reads position/voltage/temperature of all servos with a single sync-read.
bool So101Interface::pollServos()
{
  bus_->syncRead(ids_, sts3215::REG_PRESENT_POSITION, read_len_, results_);
  bool all_ok = true;
  for (size_t i = 0; i < ids_.size(); ++i) {
    const auto & r = results_[i];
    if (!r.ok || r.data.size() < read_len_) {
      all_ok = false;
      continue;  // keep the previous value for this joint
    }
    position_states_[i] = ticksToRad(cal_[i], u16(r.data[0], r.data[1]));
    if (read_health_) {
      voltage_[i] = r.data[6];
      temperature_[i] = r.data[7];
    }
    servo_error_[i] = r.error;
  }
  return all_ok;
}

void So101Interface::checkHealth()
{
  if (!throttled(last_health_warn_, 2000)) {return;}
  for (size_t i = 0; i < ids_.size(); ++i) {
    if (servo_error_[i] != 0) {
      RCLCPP_WARN(logger(), "Servo %d (%s) reports error flags 0x%02X", ids_[i],
        info_.joints[i].name.c_str(), servo_error_[i]);
    }
    if (temperature_[i] >= 65) {
      RCLCPP_WARN(logger(), "Servo %d (%s) is hot: %d C", ids_[i], info_.joints[i].name.c_str(),
        temperature_[i]);
    }
    if (voltage_[i] != 0 && voltage_[i] < 60) {
      RCLCPP_WARN(logger(), "Servo %d (%s) low supply voltage: %.1f V", ids_[i],
        info_.joints[i].name.c_str(), voltage_[i] / 10.0);
    }
  }
}

hardware_interface::return_type So101Interface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!bus_) {return hardware_interface::return_type::ERROR;}

  bool ok = false;
  try {
    ok = pollServos();
  } catch (const std::exception & e) {
    if (throttled(last_comm_warn_, 1000)) {
      RCLCPP_WARN_STREAM(logger(), "Serial read error: " << e.what());
    }
  }

  if (ok) {
    consecutive_failures_ = 0;
    checkHealth();
  } else {
    if (throttled(last_comm_warn_, 1000)) {
      RCLCPP_WARN(logger(), "Incomplete reply from servo bus (%d consecutive)",
        consecutive_failures_ + 1);
    }
    if (++consecutive_failures_ >= max_consecutive_failures_) {
      RCLCPP_ERROR(logger(), "Lost communication with the servos");
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type So101Interface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!bus_) {return hardware_interface::return_type::ERROR;}
  if (read_only_) {return hardware_interface::return_type::OK;}

  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> items;
  items.reserve(ids_.size());
  for (size_t i = 0; i < ids_.size(); ++i) {
    double cmd = position_commands_[i];
    if (!std::isfinite(cmd)) {
      cmd = last_valid_command_[i];  // never send NaN: hold the last good goal
    } else {
      last_valid_command_[i] = cmd;
    }
    const uint16_t ticks = radToTicks(cal_[i], cmd);
    items.push_back({ids_[i], {static_cast<uint8_t>(ticks & 0xFF), static_cast<uint8_t>(ticks >> 8)}});
  }

  try {
    bus_->syncWrite(sts3215::REG_GOAL_POSITION, 2, items);
  } catch (const std::exception & e) {
    if (throttled(last_comm_warn_, 1000)) {
      RCLCPP_WARN_STREAM(logger(), "Serial write error: " << e.what());
    }
    if (++consecutive_failures_ >= max_consecutive_failures_) {
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

}  // namespace so101_controller

PLUGINLIB_EXPORT_CLASS(so101_controller::So101Interface, hardware_interface::SystemInterface)
