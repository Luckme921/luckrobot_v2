#pragma once
#include "luckrobot_nav2_plugins/corner_speed.hpp"

namespace luckrobot_nav2_plugins::path_speed_envelope {
using Point=corner_speed::Point;
// Bounded geometric preview, not a torque-optimal or friction identification solver.
inline double limit(const std::vector<Point>& path,double progress,double cruise,
 double yaw,double decel,double delay,double lateral=.20) {
  if(path.size()<3) return cruise;
  std::vector<double> arc(path.size(),0.);
  for(size_t i=1;i<path.size();++i) arc[i]=arc[i-1]+std::hypot(
    path[i].x-path[i-1].x,path[i].y-path[i-1].y);
  if(arc.back()<1e-6) return 0.;
  auto sample=[&](double s) {
    s=std::clamp(s,0.,arc.back());
    auto it=std::upper_bound(arc.begin(),arc.end(),s);
    if(it==arc.end()) return path.back();
    size_t j=it-arc.begin();double t=(s-arc[j-1])/(arc[j]-arc[j-1]);
    return Point{path[j-1].x+t*(path[j].x-path[j-1].x),
      path[j-1].y+t*(path[j].y-path[j-1].y)};
  };
  double result=cruise;
  // Preview horizon covers stopping distance, response delay and geometry window.
  double horizon=std::max(1.,cruise*delay+cruise*cruise/(2*decel)+.25);
  for(double s=std::max(0.,progress);s<=std::min(arc.back(),progress+horizon);s+=.025) {
    double k=0.;
    for(double window:{.10,.20}) {
      auto a=sample(s-window),b=sample(s),c=sample(s+window);
      double ax=b.x-a.x,ay=b.y-a.y,bx=c.x-b.x,by=c.y-b.y;
      double legs=std::hypot(ax,ay)*std::hypot(bx,by);
      if(legs<1e-9) continue;
      double chord=std::hypot(c.x-a.x,c.y-a.y);
      double estimate=chord>1e-9 ? 2*std::abs(ax*by-ay*bx)/(legs*chord):10.;
      if(ax*bx+ay*by<0) estimate=std::max(10.,estimate);
      k=std::max(k,estimate);
    }
    if(k<1e-6) continue;
    double cap=std::min({cruise,.8*yaw/k,std::sqrt(lateral/k)});
    double distance=std::max(0.,s-progress-.25),ad=decel*delay;
    result=std::min(result,std::max(cap,std::sqrt(ad*ad+cap*cap+2*decel*distance)-ad));
  }
  return result;
}
inline double trackingConfidence(double error,double quiet,double stop) {
  const double u=std::clamp((std::abs(error)-quiet)/(stop-quiet),0.,1.);
  return 1-u*u*(3-2*u);
}
inline double blend(double old_speed,double envelope,double cross,double yaw_error) {
  // Small errors should not modulate cruise on every feedback sample. The
  // quiet region has zero slope; confidence still vanishes at the original
  // 5 cm / 0.20 rad/s limits. This is not a temporal filter: a lower safe
  // envelope takes effect immediately, with no delayed braking.
  double weight=trackingConfidence(cross,.01,.05)*
    trackingConfidence(yaw_error,.05,.20);
  // First rollout: cap uplift, never lift the computed feasible envelope.
  double requested=old_speed+weight*(std::min(envelope,old_speed*1.25)-old_speed);
  return std::min(envelope,requested);
}
}
