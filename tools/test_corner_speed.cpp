#include "luckrobot_nav2_plugins/corner_speed.hpp"
#include "luckrobot_nav2_plugins/path_speed_envelope.hpp"
#include <iostream>
#include <stdexcept>
using namespace luckrobot_nav2_plugins::corner_speed;
void check(bool ok) {if (!ok) {throw std::runtime_error("corner preview regression");}}
int main()
{
  auto f = [](const std::vector<Point> & p, double s, double v=.30) {
      return limit(p,s,v,.35,.35,.25);
    };
  std::vector<Point> straight{{0,0},{1,0},{2,0}};
  std::vector<Point> left{{0,0},{1,0},{1,1}};
  std::vector<Point> right{{0,0},{1,0},{1,-1}};
  std::vector<Point> dense;
  for(int i=0;i<=100;++i) {dense.push_back({i*.01,0});}
  for(int i=1;i<=100;++i) {dense.push_back({1,i*.01});}
  std::vector<Point> duplicates{{0,0},{0,0},{1,0},{1,0},{1,1},{1,1}};
  for (int i=0;i<=190;++i) {
    double s=i*.01;
    check(std::abs(f(straight,s)-.30)<1e-10);
    check(std::abs(f(left,s)-f(right,s))<1e-10);
    check(std::abs(f(left,s)-f(dense,s))<1e-8);
    check(std::abs(f(left,s)-f(duplicates,s))<1e-8);
    check(f(left,s)>=0 && f(left,s)<=.30);
    check(f(left,s,.15)<=.15);
  }
  check(f(left,0)>.29);
  check(f(left,.70)<.15); // braking while still on the straight
  check(f(left,1.5)>.29); // release after completing corner
  check(f({{0,0},{1,0},{0,0}},.95)<.1); // hairpin cannot masquerade as straight
  check(f({{0,0},{0,0},{0,0}},0)==.30);
  check(f({{0,0},{1,0},{1,1},{2,1}},.8)<.15);
  for (int i=0;i<190;++i) {
    const double s=i*.01;
    const double fast=limit(left,s,.40,.80,.35,.25);
    check(fast>=0 && fast<=.40);
    check(std::abs(fast-limit(right,s,.40,.80,.35,.25))<1e-10);
    check(std::abs(fast-limit(dense,s,.40,.80,.35,.25))<1e-8);
  }
  check(limit(left,.70,.40,.70,.35,.25)<.20);
  namespace env=luckrobot_nav2_plugins::path_speed_envelope;
  check(env::limit(straight,0,.4,.8,.35,.25)==.4);
  check(env::limit(left,.95,.4,.8,.35,.25)<.15);
  for(int i=0;i<180;++i) {
    double s=i*.01;
    check(std::abs(env::limit(left,s,.4,.8,.35,.25)-env::limit(right,s,.4,.8,.35,.25))<1e-9);
    check(std::abs(env::limit(left,s,.4,.8,.35,.25)-env::limit(dense,s,.4,.8,.35,.25))<1e-8);
  }
  check(env::blend(.2,.4,0,0)==.25);
  check(env::blend(.2,.4,.05,0)==.2);
  check(env::blend(.2,.1,0,0)==.1);
  check(env::blend(.2,.4,.009,.049)==.25);
  check(env::blend(.2,.4,-.009,-.049)==.25);
  check(env::blend(.2,.4,0,.20)==.2);
  // All confidence levels retain immediate braking and the original uplift cap.
  for(int i=0;i<=100;++i) {
    const double cross=i*.001;
    const double v=env::blend(.2,.4,cross,.08);
    check(v>=.2 && v<=.25);
    check(env::blend(.2,.1,cross,.08)==.1);
    check(std::abs(v-env::blend(.2,.4,-cross,-.08))<1e-12);
    if(i) check(v<=env::blend(.2,.4,cross-.001,.08)+1e-12);
  }
  std::vector<Point> broad,tight;
  for(int i=0;i<=400;++i) {
    double a=i*.01;
    broad.push_back({std::sin(a),1-std::cos(a)});
    tight.push_back({.45*std::sin(a),.45*(1-std::cos(a))});
  }
  check(env::limit(broad,.6,.4,.8,.35,.25)>.399);
  check(env::limit(tight,.6,.4,.8,.35,.25)<.301);
  check(std::abs(trackingBoost(0,0)-1.15)<1e-12);
  check(trackingBoost(.03,0)==1.0 && trackingBoost(0,.20)==1.0);
  check(trackingBoost(.01,.1)==trackingBoost(-.01,-.1));
  for(int i=0;i<190;++i) {
    double s=i*.01;
    auto v=limit(left,s,.4,.8,.35,.25,.20);
    check(v<=.4 && v>=limit(left,s,.4,.8,.35,.25,.15)-1e-10);
  }
  std::cout<<"PASS straight, mirrored corners, density, duplicates, hairpin, S turn and braking onset\n";
}
