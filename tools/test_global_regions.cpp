#include "explore/global_regions.hpp"
#include <cassert>
#include <iostream>
int main() {
  using namespace explore;
  GlobalRegions r;
  auto unknown=[](const RegionPoint&){return false;};
  auto known=[](const RegionPoint&){return true;};
  auto ids=r.update({{{0,0},{0,0.4}},{{4,0}}},1,unknown);
  assert(ids.size()==2 && ids[0]!=ids[1]);
  assert(r.choose({{ids[0],2},{ids[1],4}})==ids[0]);
  // A short step/changed nearest root must not discard the global task.
  assert(r.choose({{ids[0],3},{ids[1],1}})==ids[0]);
  auto moved=r.update({{{0.2,0},{0.2,0.4}},{{4.1,0}}},2,unknown);
  assert(moved==ids);
  // No feasible candidate is blocked, not covered.
  assert(r.choose({})==0);
  r.update({},3,unknown);
  assert(!r.regions[0].covered);
  // Incidental coverage requires separate map changes; repeated callback is not evidence.
  r.update({},4,known);r.update({},4,known);r.update({},4,known);
  assert(!r.regions[0].covered);
  r.update({},5,known);r.update({},6,known);
  assert(r.regions[0].covered);
  // Rediscovered frontier reopens same region.
  auto reopened=r.update({{{0.2,0},{0.2,0.4}},{{4.1,0}}},7,unknown);
  assert(reopened==ids && !r.regions[0].covered);
  // A split creates a child; repeating the split does not create new IDs.
  auto split=r.update({{{0.2,0}},{{0.2,0.4}},{{4.1,0}}},8,unknown);
  assert(r.regions.size()==3);
  auto repeated=r.update({{{0.2,0}},{{0.2,0.4}},{{4.1,0}}},9,unknown);
  assert(split==repeated && r.regions.size()==3);
  r.active=split[0];
  assert(r.choose({{split[0],1},{split[1],2},{split[2],0.1}})==split[1]);
  std::cout << "Global region identity, retention, blocking, coverage, reopening and split tests passed\n";
}
