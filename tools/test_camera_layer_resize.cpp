#include "luckrobot_nav2_plugins/persistent_camera_layer.hpp"
#include <pluginlib/class_loader.hpp>
#include <iostream>
#include <stdexcept>
class Harness : public luckrobot_nav2_plugins::PersistentCameraLayer {
public:
  void attach(nav2_costmap_2d::LayeredCostmap * p) {layered_costmap_ = p;}
};
int main() {
  auto require = [](bool ok) {if (!ok) {throw std::runtime_error("Camera memory assertion failed");}};
  nav2_costmap_2d::LayeredCostmap parent("map", false, true);
  Harness h; h.attach(&parent);
  parent.resizeMap(20,20,.05,0.,0.); h.matchSize();
  h.setCost(5,6,nav2_costmap_2d::LETHAL_OBSTACLE);
  double wx,wy; h.mapToWorld(5,6,wx,wy);
  for (int i=1;i<40;++i) {
    parent.resizeMap(20+i,20+i,.05,-.013*i,-.017*i); h.matchSize();
    unsigned int x,y; require(h.worldToMap(wx,wy,x,y));
    require(h.getCost(x,y)==nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  unsigned int x,y; h.worldToMap(wx,wy,x,y);
  h.setCost(x,y,nav2_costmap_2d::FREE_SPACE);
  parent.resizeMap(70,70,.05,-.9,-.9); h.matchSize();
  h.worldToMap(wx,wy,x,y);
  require(h.getCost(x,y)!=nav2_costmap_2d::LETHAL_OBSTACLE);
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  auto plugin=loader.createSharedInstance("luckrobot_nav2_plugins::PersistentCameraLayer");
  require(static_cast<bool>(plugin));
  std::cout << "PASS: 39 origin/size changes retain world position; cleared hit not resurrected; plugin loads\n";
}
