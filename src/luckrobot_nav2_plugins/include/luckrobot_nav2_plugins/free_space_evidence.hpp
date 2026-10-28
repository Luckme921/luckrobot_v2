#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
namespace luckrobot_nav2_plugins {
// Whole-cell evidence: every beam covering the cell must be a valid finite
// return beyond its far edge. Unknown, occluded and missing rays never clear.
inline bool finiteFreeCell(double x, double y, double radius,
  double angle_min, double increment, double range_min, double range_max,
  const std::vector<float> & ranges)
{
  const double r=std::hypot(x,y);
  if (!std::isfinite(r) || r <= std::max(.30,range_min)+radius ||
      r+radius+.15 >= std::min(10.0,range_max) || increment<=0 || ranges.empty()) return false;
  const double a=std::atan2(y,x), spread=std::asin(std::min(1.0,radius/r));
  const int lo=static_cast<int>(std::floor((a-spread-angle_min)/increment));
  const int hi=static_cast<int>(std::ceil((a+spread-angle_min)/increment));
  if (lo<0 || hi>=static_cast<int>(ranges.size())) return false;
  for (int i=lo;i<=hi;++i) {
    const double v=ranges[i];
    if (!std::isfinite(v) || v<range_min || v>range_max || v<=r+radius+.15) return false;
  }
  return true;
}
struct FreeEvidence {
  int votes=0; double first=-1, last=-1; bool cleared=false;
  void observe(bool free, double stamp) {
    if (stamp<=last) return;
    if (!free) { votes=0; first=-1; last=stamp; return; }
    if (last<0 || stamp-last>.35) {votes=0;first=stamp;}
    if (first<0) first=stamp;
    last=stamp;
    if (++votes>=5 && stamp-first>=.5) cleared=true;
  }
};
}
