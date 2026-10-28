#pragma once
#include <cmath>
#include <cstdint>
#include <vector>
#include <queue>
namespace explore {
// UNKNOWN is not an obstacle. Only measured occupied cells prove a bottleneck.
template<class Occupied>
bool narrowFrontier(int x,int y,int width,int height,double resolution,
                    double radius,Occupied occupied) {
  const int n=static_cast<int>(std::ceil(radius/resolution));
  // Map borders never constitute evidence of a wall.

  for(int dy=-n;dy<=n;++dy)for(int dx=-n;dx<=n;++dx)
    if(x+dx>=0 && y+dy>=0 && x+dx<width && y+dy<height &&
       std::hypot(dx,dy)*resolution < radius && occupied(x+dx,y+dy))return true;
  return false;
}
// Configuration-space reachability used ONLY for coverage estimates.
// Unknown may be explored here; actual navigation still requires known free.
inline std::vector<unsigned char> coverageReachable(const unsigned char* grid,
    int w,int h,double resolution,double clearance) {
  std::vector<unsigned char> safe(w*h,1),seen(w*h,0);
  const double radius=clearance+resolution*std::sqrt(2.0)*0.5;
  for(int y=0;y<h;++y)for(int x=0;x<w;++x)
    if(narrowFrontier(x,y,w,h,resolution,radius,
        [&](int cx,int cy){return grid[cy*w+cx]==254;}))safe[y*w+x]=0;
  std::queue<int> q;
  // All known free components seed the assessment: disconnection from the
  // current robot by a temporary blockage must NOT declare a room complete.
  for(int i=0;i<w*h;++i)if(safe[i] && grid[i]==0){seen[i]=1;q.push(i);}
  while(!q.empty()) {
    int i=q.front();q.pop();int x=i%w,y=i/w;
    const int dx[4]={1,-1,0,0},dy[4]={0,0,1,-1};
    for(int k=0;k<4;++k){int nx=x+dx[k],ny=y+dy[k];
      if(nx<0||ny<0||nx>=w||ny>=h)continue;
      int j=ny*w+nx;
      if(!seen[j]&&safe[j]&&(grid[j]==0||grid[j]==255)){seen[j]=1;q.push(j);}
    }
  }
  return seen;
}
class CoverageConfirmation {
 double since_=-1; std::uint64_t revision_=0; unsigned updates_=0;
 public:
 bool update(bool empty,std::uint64_t revision,double now) {
  if(!empty || (since_>=0 && now<since_)){since_=-1;updates_=0;revision_=revision;return false;}
  if(since_<0){since_=now;revision_=revision;updates_=1;}
  else if(revision!=revision_){revision_=revision;++updates_;}
  return updates_>=3 && now-since_>=30.0;
 }
};
}
