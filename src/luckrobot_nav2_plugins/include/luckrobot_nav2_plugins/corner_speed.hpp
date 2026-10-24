#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace luckrobot_nav2_plugins::corner_speed
{
struct Point {double x; double y;};

// Geometry is sampled in metres, never in planner vertex counts. This is a
// speed constraint only: it neither rounds corners nor certifies a safe arc.
inline double limit(const std::vector<Point> & path, double progress, double cruise,
  double max_yaw, double decel, double delay)
{
  if (path.size() < 3) {return cruise;}
  std::vector<double> arc(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); ++i) {
    arc[i] = arc[i-1] + std::hypot(path[i].x-path[i-1].x, path[i].y-path[i-1].y);
  }
  auto sample = [&](double s) {
      s = std::clamp(s, 0.0, arc.back());
      auto it = std::upper_bound(arc.begin(), arc.end(), s);
      if (it == arc.end()) {return path.back();}
      const size_t j = static_cast<size_t>(it-arc.begin());
      const double t = (s-arc[j-1])/(arc[j]-arc[j-1]);
      return Point{path[j-1].x+t*(path[j].x-path[j-1].x),
        path[j-1].y+t*(path[j].y-path[j-1].y)};
    };
  double result = cruise;
  constexpr double window = 0.20;
  // 1 m exceeds the modelled stopping distance at the supported 0.30 m/s.
  for (double s = std::max(0.0, progress); s <= std::min(arc.back(), progress+1.0);
    s += 0.025)
  {
    const auto a = sample(s-window), b = sample(s), c = sample(s+window);
    const double ax = b.x-a.x, ay = b.y-a.y, bx = c.x-b.x, by = c.y-b.y;
    const double product = std::hypot(ax, ay)*std::hypot(bx, by)*
      std::hypot(c.x-a.x, c.y-a.y);
    const double legs = std::hypot(ax, ay)*std::hypot(bx, by);
    if (legs < 1e-9) {continue;}  // terminal duplicates handled by goal braking
    const double k = ax*bx+ay*by < 0.0 ?
      std::max(10.0, product > 1e-9 ? 2.0*std::abs(ax*by-ay*bx)/product : 10.0) :
      (product > 1e-9 ? 2.0*std::abs(ax*by-ay*bx)/product : 0.0);
    if (k < 1e-6) {continue;}
    const double turn = std::min({cruise, 0.8*max_yaw/k, std::sqrt(0.15/k)});
    // Start braking before the geometry window, with 5 cm extra margin.
    // Solve d >= v*T + (v^2-turn^2)/(2*a). Never demand below turn speed
    // solely because no braking distance remains; existing tracking may do so.
    const double d = std::max(0.0, s-progress-window-0.05);
    const double ad = decel*delay;
    const double bound = std::max(turn,
      std::sqrt(ad*ad+turn*turn+2.0*decel*d)-ad);
    result = std::min(result, bound);
  }
  return result;
}
}
