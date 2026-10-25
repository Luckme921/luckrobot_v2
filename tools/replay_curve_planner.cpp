#include "luckrobot_nav2_plugins/luck_planner.hpp"
#include "luckrobot_nav2_plugins/curve_optimizer.hpp"
#include <fstream>
#include <chrono>
#include <iostream>
#include <cmath>
#include <stdexcept>
namespace luckrobot_nav2_plugins {
struct LuckPlannerTestAccess {
  static double maxTurn(const nav_msgs::msg::Path & path) {
    double result=0;
    for(size_t i=1;i+1<path.poses.size();++i) {
      auto a=path.poses[i-1].pose.position,b=path.poses[i].pose.position,c=path.poses[i+1].pose.position;
      if(std::hypot(b.x-a.x,b.y-a.y)<1e-6 || std::hypot(c.x-b.x,c.y-b.y)<1e-6) continue;
      result=std::max(result,std::abs(std::remainder(std::atan2(c.y-b.y,c.x-b.x)-
        std::atan2(b.y-a.y,b.x-a.x),2*M_PI))*180/M_PI);
    }
    return result;
  }
  static void run(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros,const char *file) {
    std::ifstream in(file); int count; in>>count;if(!in) throw std::runtime_error("fixture missing");
    LuckPlanner p;p.costmap_ros_=ros;p.costmap_=ros->getCostmap();
    p.global_frame_="map";p.clock_=ros->get_clock();p.plugin_name_="offline_curve_replay";
    p.route_hysteresis_enabled_=false;
    std::vector<geometry_msgs::msg::Point> footprint(32);
    for(int i=0;i<32;++i){footprint[i].x=.175*std::cos(i*2*M_PI/32);footprint[i].y=.175*std::sin(i*2*M_PI/32);}
    ros->setRobotFootprint(footprint);
    for(int k=0;k<count;++k) {
      unsigned w,h;double res,ox,oy;in>>w>>h>>res>>ox>>oy;
      p.costmap_->resizeMap(w,h,res,ox,oy);
      for(unsigned y=0;y<h;++y)for(unsigned x=0;x<w;++x){int c;in>>c;p.costmap_->setCost(x,y,c);}
      geometry_msgs::msg::PoseStamped a,b;a.header.frame_id=b.header.frame_id="map";
      a.pose.orientation.w=b.pose.orientation.w=1;
      in>>a.pose.position.x>>a.pose.position.y>>b.pose.position.x>>b.pose.position.y;
      if(!in) throw std::runtime_error("truncated fixture");
      p.use_astar_=true;p.constrained_curve_enabled_=false;
      auto t=std::chrono::steady_clock::now();auto old=p.createPlan(a,b);
      double old_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t).count();
      p.use_astar_=false;p.constrained_curve_enabled_=true;
      t=std::chrono::steady_clock::now();auto now=p.createPlan(a,b);
      double new_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t).count();
      std::cout<<"REPLAY,"<<k<<","<<old.poses.size()<<","<<now.poses.size()<<","<<maxTurn(old)
        <<","<<maxTurn(now)<<","<<old_ms<<","<<new_ms<<"\n";
    }
  }
};
}
int main(int argc,char **argv) {
  if(argc<2)return 2;
  rclcpp::init(argc,argv);
  auto ros=std::make_shared<nav2_costmap_2d::Costmap2DROS>("offline_curve_replay");
  ros->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{}));
  ros->on_configure(rclcpp_lifecycle::State());
  luckrobot_nav2_plugins::LuckPlannerTestAccess::run(ros,argv[1]);
  ros->on_cleanup(rclcpp_lifecycle::State());ros.reset();rclcpp::shutdown();
}
