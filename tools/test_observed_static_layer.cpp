#include "luckrobot_nav2_plugins/observed_static_layer.hpp"
#include <tf2_ros/buffer.h>
#include <thread>
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>
class Harness: public luckrobot_nav2_plugins::ObservedStaticLayer {
public:
 void attach(nav2_costmap_2d::LayeredCostmap *parent,rclcpp_lifecycle::LifecycleNode::SharedPtr node,tf2_ros::Buffer *tf) {
  layered_costmap_=parent;node_=node;tf_=tf;enabled_=true;map_received_=true;
  map_received_in_update_bounds_=true;map_frame_="map";global_frame_="map";
  use_maximum_=false;resizeMap(80,80,.05,-2,-2);resetMap(0,0,80,80);
  setCost(60,40,254);setCost(70,40,254);
 }
 void scan(rclcpp::Time now,float range) {
  auto s=std::make_shared<sensor_msgs::msg::LaserScan>();s->header.stamp=now;s->header.frame_id="laser";
  s->angle_min=-M_PI;s->angle_max=M_PI;s->angle_increment=M_PI/360;
  s->range_min=.1;s->range_max=12;s->ranges.assign(721,range);scan_=s;
 }
};
int main(int argc,char **argv) {
 rclcpp::init(argc,argv);
 auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("static_clear_test");
 tf2_ros::Buffer buffer(node->get_clock());
 buffer.setUsingDedicatedThread(true); // test buffer is preloaded with static transforms
 geometry_msgs::msg::TransformStamped tr;tr.header.frame_id="map";tr.child_frame_id="odom";tr.transform.rotation.w=1;
 buffer.setTransform(tr,"test",true);
 tr.child_frame_id="laser";tr.transform.rotation.w=0;tr.transform.rotation.x=1; // inverted lidar
 buffer.setTransform(tr,"test",true);
 nav2_costmap_2d::LayeredCostmap parent("map",false,true);
 parent.resizeMap(80,80,.05,-2,-2);
 Harness h;h.attach(&parent,node,&buffer);auto *master=parent.getCostmap();
 for(int i=0;i<8;++i) {
  h.scan(node->now(),1.52f);h.updateCosts(*master,0,0,80,80);
  if(i==0)assert(master->getCost(60,40)==254);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
 }
 assert(master->getCost(60,40)==0); // old hit, continuous farther returns
 assert(master->getCost(70,40)==254); // real wall endpoint stays occupied
 h.scan(node->now(),1.025f);h.updateCosts(*master,0,0,80,80);
 assert(master->getCost(60,40)==254); // person returns: immediate restoration
 h.reset();
 for(int i=0;i<8;++i) {
  h.scan(node->now(),std::numeric_limits<float>::infinity());h.updateCosts(*master,0,0,80,80);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
 }
 assert(master->getCost(60,40)==254);
 std::cout<<"PASS: real StaticLayer merge, inverted lidar, delayed clear, wall preservation, reappearance, invalid returns\n";
 rclcpp::shutdown();
}
