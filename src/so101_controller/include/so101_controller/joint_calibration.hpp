// Conversion between STS3215 ticks (0..4095 per revolution) and joint radians.
//
// The servos already contain the LeRobot homing offsets (written to EEPROM by
// `lerobot-calibrate`), so Present_Position is in the *calibrated* tick frame.
// Following LeRobot's "degrees" convention, joint zero is the middle of the recorded range:
//     rad = direction * (ticks - zero_tick) * 2*pi / 4096,   zero_tick = (range_min+range_max)/2
#ifndef SO101_CONTROLLER__JOINT_CALIBRATION_HPP_
#define SO101_CONTROLLER__JOINT_CALIBRATION_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace so101_controller
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTicksPerRad = 4096.0 / (2.0 * kPi);

struct JointCalibration
{
  uint8_t id = 0;
  double zero_tick = 2048.0;  // tick value that corresponds to joint angle 0
  double min_tick = 0.0;      // calibrated range (also the servo's EEPROM angle limits)
  double max_tick = 4095.0;
  double direction = 1.0;     // +1 or -1
  double min_rad = -std::numeric_limits<double>::infinity();  // from the URDF limits
  double max_rad = std::numeric_limits<double>::infinity();
  int torque_limit = -1;      // 0..1000, -1 = leave the servo default
};

inline double ticksToRad(const JointCalibration & c, uint16_t ticks)
{
  return c.direction * (static_cast<double>(ticks) - c.zero_tick) / kTicksPerRad;
}

// Clamps to the URDF limits first, then to the calibrated tick range.
inline uint16_t radToTicks(const JointCalibration & c, double rad)
{
  rad = std::min(std::max(rad, c.min_rad), c.max_rad);
  double t = c.zero_tick + c.direction * rad * kTicksPerRad;
  t = std::min(std::max(t, c.min_tick), c.max_tick);
  return static_cast<uint16_t>(std::lround(t));
}

}  // namespace so101_controller

#endif  // SO101_CONTROLLER__JOINT_CALIBRATION_HPP_
