#pragma once
#include <cmath>
#include <cstdint>
#include <vector>
namespace explore {
// A bounded observation budget, not a declaration that a region is covered.
class ObservationProgress {
 struct State {std::uint64_t id; double x,y,area; unsigned attempts;};
 std::vector<State> states_;
 public:
 bool eligible(std::uint64_t id,double x,double y,double area) {
   for(auto& s:states_) if(s.id==id) {
     // New frontier geometry or substantial coverage change allows another try.
     if(std::hypot(x-s.x,y-s.y)>0.50 || area>s.area+0.25 || s.area-area>=0.15) {
       s={id,x,y,area,0};
     }
     return s.attempts<2;
   }
   states_.push_back({id,x,y,area,0});return true;
 }
 void reached(std::uint64_t id) {
   for(auto& s:states_) if(s.id==id){++s.attempts;return;}
 }
 static bool repeat(double distance,double frontier_shift,double old_area,double area) {
   // Shrinking unknown space is NOT new evidence warranting the same viewpoint.
   return distance<0.40 && frontier_shift<0.50 && area<=old_area+0.25;
 }
};
}
