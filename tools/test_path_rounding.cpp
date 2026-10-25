#include "luckrobot_nav2_plugins/path_rounding.hpp"
#include <stdexcept>
#include <iostream>
using namespace luckrobot_nav2_plugins::path_rounding;
void check(bool b) {if(!b) throw std::runtime_error("path rounding regression");}
int main() {
  auto free=[](Point,Point){return true;};
  std::vector<Point> l{{0,0},{1,0},{1,1}};
  auto p=round(l,.25,.01,free);
  check(distance(p.front(),l.front())==0 && distance(p.back(),l.back())==0);
  check(p.size()>10);
  double max_turn=0;
  for(size_t i=1;i+1<p.size();++i) {
    auto a=p[i-1],b=p[i],c=p[i+1];
    double la=distance(a,b),lb=distance(b,c);
    if(la*lb>1e-10) max_turn=std::max(max_turn,std::acos(std::clamp(
      ((b.x-a.x)*(c.x-b.x)+(b.y-a.y)*(c.y-b.y))/(la*lb),-1.,1.)));
    check(b.x>=0 && b.x<=1 && b.y>=0 && b.y<=1);
  }
  check(max_turn<.10); // original 90-degree discontinuity substantially reduced
  auto blocked=round(l,.25,.01,[](Point,Point){return false;});
  check(blocked.size()==l.size() && distance(blocked[1],l[1])==0);
  auto straight=round(std::vector<Point>{{0,0},{1,0},{2,0}},.25,.01,free);
  check(straight.size()==3);
  auto reverse=round(std::vector<Point>{{0,0},{1,0},{0,0}},.25,.01,free);
  check(reverse.size()==3);
  auto duplicates=round(std::vector<Point>{{0,0},{0,0},{1,0}},.25,.01,free);
  check(duplicates.size()==3);
  auto disabled=round(l,0,.01,free);check(disabled.size()==3);
  // Simulated forbidden region on the inside of the corner. Every accepted
  // chord must pass a fine collision check, or the original corner survives.
  auto safe=[](Point a,Point b){
    for(int j=0;j<=100;++j) {double t=j/100.;double x=a.x+t*(b.x-a.x),y=a.y+t*(b.y-a.y);
      if(x<.96 && y>.04) return false;}
    return true;
  };
  auto narrow=round(l,.25,.01,safe);
  for(size_t i=1;i<narrow.size();++i) check(safe(narrow[i-1],narrow[i]));
  // An unroundable first corner must not cancel a feasible second corner.
  std::vector<Point> two{{0,0},{1,0},{1,1},{2,1}};
  auto selective=[](Point a,Point b){
    for(int j=0;j<=100;++j) {
      double t=j/100.,x=a.x+t*(b.x-a.x),y=a.y+t*(b.y-a.y);
      if(x<.999 && y>.001) return false;
    }
    return true;
  };
  auto mixed=round(two,.45,.01,selective);
  bool kept_first=false,kept_second=false;
  for(auto q:mixed) {kept_first|=distance(q,two[1])<1e-9;kept_second|=distance(q,two[2])<1e-9;}
  check(kept_first && !kept_second);
  for(size_t i=1;i<mixed.size();++i) check(selective(mixed[i-1],mixed[i]));
  std::cout<<"PASS corner geometry, endpoints, obstacle fallback, hairpin, duplicates\n";
}
