#include "luckrobot_nav2_plugins/free_space_evidence.hpp"
#include <cassert>
#include <limits>
#include <iostream>
using namespace luckrobot_nav2_plugins;
int main() {
 std::vector<float> scan(721,3.f);
 auto free=[&](){return finiteFreeCell(1.,0.,.036,-3.141592653589793,.00872664626,.1,12.,scan);};
 assert(free());
 scan[360]=1.; assert(!free()); // actual obstacle
 scan[360]=.5; assert(!free()); // foreground occlusion
 scan[360]=std::numeric_limits<float>::infinity();assert(!free());
 scan[360]=std::numeric_limits<float>::quiet_NaN();assert(!free());
 scan[360]=3.;scan[363]=1.;assert(!free()); // edge beam protects entire cell
 FreeEvidence e;
 for(int i=0;i<5;i++)e.observe(true,1.+i*.1);
 assert(!e.cleared);e.observe(true,1.6);assert(e.cleared);
 FreeEvidence g;for(int i=0;i<20;i++)g.observe(true,i*.5);assert(!g.cleared);
 FreeEvidence h;for(int i=0;i<4;i++)h.observe(true,1.+i*.1);
 h.observe(false,1.4);h.observe(true,1.5);assert(!h.cleared);
 std::cout<<"PASS: finite whole-cell visibility, occlusion, invalid depth, temporal continuity\n";
}
