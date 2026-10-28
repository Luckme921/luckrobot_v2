#include "explore/coverage_policy.hpp"
#include <cassert>
#include <iostream>
int main(){
 auto wall=[](int x,int){return x==10 || x==16;};
 assert(explore::narrowFrontier(13,20,50,50,.05,.175,wall));
 auto doorway=[](int x,int){return x==10 || x==20;};
 assert(!explore::narrowFrontier(15,20,50,50,.05,.175,doorway));
 assert(!explore::narrowFrontier(1,20,50,50,.05,.175,wall));
 assert(!explore::narrowFrontier(15,20,50,50,.05,.175,[](int,int){return false;}));
 explore::CoverageConfirmation c;
 assert(!c.update(true,1,0));assert(!c.update(true,1,60));
 assert(!c.update(true,2,61));assert(c.update(true,3,62));
 assert(!c.update(false,4,63));assert(!c.update(true,5,64));
 assert(!c.update(true,6,65));assert(c.update(true,7,95));
 assert(!c.update(true,8,0));
 // Closed wall, a narrow slit, a wide door, and a new opening.
 std::vector<unsigned char> g(60*60,255);
 for(int y=0;y<60;++y)for(int x=0;x<25;++x)g[y*60+x]=0;
 for(int y=0;y<60;++y)g[y*60+25]=254;
 auto closed=explore::coverageReachable(g.data(),60,60,.05,.25);
 assert(!closed[30*60+40]);
 for(int y=28;y<=32;++y)g[y*60+25]=255;
 auto slit=explore::coverageReachable(g.data(),60,60,.05,.25);
 assert(!slit[30*60+40]);
 for(int y=20;y<=40;++y)g[y*60+25]=255;
 auto open=explore::coverageReachable(g.data(),60,60,.05,.25);
 assert(open[30*60+40]);
 // With no obstacle evidence, gray beyond the border is never a wall.
 std::fill(g.begin(),g.end(),255);g[30*60+10]=0;
 assert(explore::coverageReachable(g.data(),60,60,.05,.25)[30*60+59]);
 std::cout<<"coverage geometry and temporal checks passed\n";
}
