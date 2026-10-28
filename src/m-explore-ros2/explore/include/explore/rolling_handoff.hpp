#pragma once
#include <cmath>
namespace explore {
inline bool rollingSuccessorSafe(double ox, double oy, double nx, double ny,
                                 double min_advance, double max_angle) {
  const double old_distance = std::hypot(ox, oy);
  const double next_distance = std::hypot(nx, ny);
  if (!std::isfinite(old_distance) || !std::isfinite(next_distance) || old_distance < 0.01) return false;
  const double change = std::atan2(ox * ny - oy * nx, ox * nx + oy * ny);
  return next_distance >= old_distance + min_advance && std::abs(change) <= max_angle;
}
}
