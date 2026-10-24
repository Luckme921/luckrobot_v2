#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace luckrobot_nav2_plugins::path_rounding
{
struct Point {double x; double y;};
inline double distance(Point a, Point b) {return std::hypot(a.x-b.x, a.y-b.y);}

// Tangent quadratic corner blends, not a curvature-continuous spline.
// Validate every replacement segment against the SAME locked costmap snapshot.
// No feasible blend means preserve the original corner, never erase a route.
template<class SegmentClear>
std::vector<Point> round(const std::vector<Point> & input, double trim_max,
  double step, SegmentClear clear)
{
  if (input.size()<3 || trim_max<=0 || step<=0) {return input;}
  std::vector<Point> out{input.front()};
  for (size_t i=1; i+1<input.size(); ++i) {
    const auto a=input[i-1], b=input[i], c=input[i+1];
    const double la=distance(a,b), lb=distance(b,c);
    if (la<1e-6 || lb<1e-6) {out.push_back(b); continue;}
    const double ux=(b.x-a.x)/la, uy=(b.y-a.y)/la;
    const double vx=(c.x-b.x)/lb, vy=(c.y-b.y)/lb;
    const double angle=std::acos(std::clamp(ux*vx+uy*vy,-1.0,1.0));
    // Near straight needs no deformation; hairpins remain explicit alignment.
    if (angle<0.10 || angle>2.35) {out.push_back(b); continue;}
    double trim=std::min({trim_max,.40*la,.40*lb});
    bool accepted=false;
    for (int attempt=0; attempt<4 && trim>=.04; ++attempt, trim*=.5) {
      Point entry{b.x-trim*ux,b.y-trim*uy}, exit{b.x+trim*vx,b.y+trim*vy};
      std::vector<Point> curve{entry};
      const int n=std::max(4,static_cast<int>(std::ceil(2*trim/step)));
      for (int j=1;j<=n;++j) {
        const double t=static_cast<double>(j)/n, s=1-t;
        curve.push_back({s*s*entry.x+2*s*t*b.x+t*t*exit.x,
          s*s*entry.y+2*s*t*b.y+t*t*exit.y});
      }
      bool valid=clear(out.back(),curve.front());
      for (size_t j=1;valid && j<curve.size();++j) {valid=clear(curve[j-1],curve[j]);}
      valid=valid && clear(curve.back(),c);
      if (valid) {out.insert(out.end(),curve.begin(),curve.end()); accepted=true; break;}
    }
    if (!accepted) {out.push_back(b);}
  }
  out.push_back(input.back());
  return out;
}
}
