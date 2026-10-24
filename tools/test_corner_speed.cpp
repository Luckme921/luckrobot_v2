#include "luckrobot_nav2_plugins/corner_speed.hpp"
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
  std::cout<<"PASS straight, mirrored corners, density, duplicates, hairpin, S turn and braking onset\n";
}
