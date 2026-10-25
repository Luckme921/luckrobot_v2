#include "luckrobot_nav2_plugins/path_speed_envelope.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>
int main(int argc,char**argv) {
  if(argc!=2)return 2;
  std::ifstream in(argv[1]);int count;in>>count;
  if(!in)return 2;
  int samples=0,lower=0,cruise=0;
  double max_gap=0.;
  for(int c=0;c<count;++c) {
    int n;in>>n;std::vector<luckrobot_nav2_plugins::corner_speed::Point> p(n);
    double length=0.;
    for(int i=0;i<n;++i){in>>p[i].x>>p[i].y;if(i)length+=std::hypot(p[i].x-p[i-1].x,p[i].y-p[i-1].y);}
    if(!in)return 2;
    for(double s=0;s<length;s+=.1) {
      double v=luckrobot_nav2_plugins::path_speed_envelope::limit(p,s,.4,.8,.35,.25);
      double old=luckrobot_nav2_plugins::corner_speed::limit(p,s,.4,.8,.35,.25,.2);
      if(!std::isfinite(v)||v<0||v>.4000001)throw std::runtime_error("invalid envelope");
      // The new two-scale preview must never miss a constraint the old preview saw.
      if(v>old+1e-8)throw std::runtime_error("lost existing preview constraint");
      ++samples;lower+=v<old-1e-6;cruise+=v>.399;max_gap=std::max(max_gap,old-v);
    }
  }
  std::cout<<"paths="<<count<<" samples="<<samples<<" cruise_allowed="<<cruise
    <<" tighter_preview="<<lower<<" max_extra_reduction="<<max_gap<<"\n";
}
