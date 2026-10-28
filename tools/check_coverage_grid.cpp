#include "explore/coverage_policy.hpp"
#include <iostream>
int main(){int w,h;double r;std::cin>>w>>h>>r;std::vector<unsigned char>g(w*h);
 for(auto& v:g){int n;std::cin>>n;v=n<0?255:(n>=65?254:0);}
 auto a=explore::coverageReachable(g.data(),w,h,r,.25);int pending=0;
 for(int y=1;y<h-1;++y)for(int x=1;x<w-1;++x){int i=y*w+x;
 if(g[i]==255&&a[i]&&(g[i-1]==0||g[i+1]==0||g[i-w]==0||g[i+w]==0))++pending;}
 std::cout<<pending;
}
