#pragma once

#include <algorithm>
#include <cmath>

namespace luckrobot_nav2_plugins::motion_prediction
{
// Solve distance = speed * delay + speed^2 / (2 * deceleration).
// delay/deceleration are conservative modelling parameters, not calibrated
// motor specifications. No minimum-speed floor may override this bound.
inline double stoppingSpeed(double distance, double deceleration, double delay)
{
  if (distance <= 0.0) {return 0.0;}
  const double ad = deceleration * delay;
  return 2.0 * deceleration * distance /
         (std::sqrt(ad * ad + 2.0 * deceleration * distance) + ad);
}

inline double advanceRate(double current, double target, double accel, double decel, double dt)
{
  // A reversal must brake to zero before accelerating in the opposite direction.
  if (current * target < 0.0) {
    const double brake_time = std::abs(current) / decel;
    if (dt <= brake_time) {
      return current - std::copysign(decel * dt, current);
    }
    return std::copysign(std::min(std::abs(target), accel * (dt - brake_time)), target);
  }
  const double rate = std::abs(target) > std::abs(current) ? accel : decel;
  return current + std::clamp(target - current, -rate * dt, rate * dt);
}

inline double steering(double linear, double bearing, double distance,
  double measured_yaw_rate, double preview_time, double correction_limit)
{
  const double nominal = 2.0 * linear * std::sin(bearing) / std::max(distance, 0.01);
  // Predict heading, not a future path-segment tangent. This starts unwinding
  // before the chassis crosses the carrot bearing. It has no integrator and
  // cannot build up a breakaway command while collision_monitor holds stop.
  const double predicted = 2.0 * linear *
    std::sin(bearing - measured_yaw_rate * preview_time) / std::max(distance, 0.01);
  return nominal + std::clamp(predicted - nominal, -correction_limit, correction_limit);
}

inline double trackingSpeedScale(double bearing, double requested_yaw, double actual_yaw,
  double small_error, double large_error, double max_yaw)
{
  // Small heading errors retain forward speed. When both heading error and
  // execution mismatch are large, give yaw time to catch up instead of driving
  // the same forward distance on an arc the chassis cannot yet execute.
  const double heading = std::clamp(
    (std::abs(bearing) - small_error) / (large_error - small_error), 0.0, 1.0);
  const double lag = std::clamp(
    (std::abs(requested_yaw - actual_yaw) - 0.05) / max_yaw, 0.0, 1.0);
  return 1.0 - 0.65 * heading * lag;
}
}  // namespace luckrobot_nav2_plugins::motion_prediction
