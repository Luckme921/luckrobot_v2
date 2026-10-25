#include "luckrobot_nav2_plugins/luck_planner.hpp"
#include "luckrobot_nav2_plugins/curve_optimizer.hpp"
#include <cmath>
#include <stdexcept>
#include <iostream>
namespace luckrobot_nav2_plugins {
struct LuckPlannerTestAccess {
  static void run(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros) {
    auto check=[](bool v){if(!v) throw std::runtime_error("planner integration regression");};
    LuckPlanner p;
    p.costmap_ros_=ros;p.costmap_=ros->getCostmap();p.global_frame_="map";p.clock_=ros->get_clock();
    p.costmap_->resizeMap(80,80,.05,-1,-1);
    for(unsigned y=0;y<80;++y) for(unsigned x=0;x<80;++x) p.costmap_->setCost(x,y,0);
    std::vector<geometry_msgs::msg::Point> footprint(20);
    for(int i=0;i<20;++i) {footprint[i].x=.175*std::cos(i*2*M_PI/20);footprint[i].y=.175*std::sin(i*2*M_PI/20);}
    ros->setRobotFootprint(footprint);
    geometry_msgs::msg::PoseStamped start,goal;
    start.header.frame_id=goal.header.frame_id="map";
    start.pose.orientation.w=goal.pose.orientation.w=1;
    start.pose.position.x=.025;start.pose.position.y=.025;
    goal.pose.position.x=1.025;goal.pose.position.y=1.025;
    std::vector<LuckPlanner::Cell> cells{{20,20},{40,20},{40,40}};
    p.corner_rounding_enabled_=false;
    auto baseline=p.cellsToPath(cells,start,goal);
    p.corner_rounding_enabled_=true;
    auto rounded=p.cellsToPath(cells,start,goal);
    double nearest=10;
    for(auto & q:rounded.poses) nearest=std::min(nearest,std::hypot(q.pose.position.x-1.025,q.pose.position.y-.025));
    check(nearest>.07); // actual plugin removes the sharp corner in free space
    check(rounded.poses.front().pose.position.x==start.pose.position.x);
    check(rounded.poses.back().pose.position.y==goal.pose.position.y);
    // Unknown space must not authorize smoothing, even when A* permits unknown.
    for(unsigned y=0;y<80;++y) for(unsigned x=0;x<80;++x) p.costmap_->setCost(x,y,255);
    auto fallback=p.cellsToPath(cells,start,goal);
    check(fallback.poses.size()==baseline.poses.size());
    for(size_t i=0;i<fallback.poses.size();++i) {
      check(fallback.poses[i].pose.position.x==baseline.poses[i].pose.position.x);
      check(fallback.poses[i].pose.position.y==baseline.poses[i].pose.position.y);
    }
    for(unsigned y=0;y<80;++y) for(unsigned x=0;x<80;++x) p.costmap_->setCost(x,y,0);
    // Native optimizer is exercised, not a mock. Short L in free space can
    // safely relax toward the diagonal; endpoints and footprint stay valid.
    geometry_msgs::msg::PoseStamped short_goal=goal;
    short_goal.pose.position.x=.825;short_goal.pose.position.y=.825;
    auto short_seed=p.cellsToPath({{20,20},{36,20},{36,36}},start,short_goal);
    nav_msgs::msg::Path optimized;
    check(optimizeCurve(short_seed,optimized,p.costmap_,.45,.08));
    check(p.validateCurve(short_seed,optimized));
    check(optimized.poses.back().pose.position.x==short_goal.pose.position.x);
    // Validation must reject obstacles introduced after optimization.
    unsigned mx,my;
    const auto midpoint=optimized.poses[optimized.poses.size()/2].pose.position;
    check(p.costmap_->worldToMap(midpoint.x,midpoint.y,mx,my));
    p.costmap_->setCost(mx,my,254);
    check(!p.validateCurve(short_seed,optimized));
    p.costmap_->setCost(mx,my,255);
    check(!p.validateCurve(short_seed,optimized));
    p.costmap_->setCost(mx,my,0);
    // Inflated clearance at the outline is not a physical collision. Center
    // inflation is still forbidden; lethal and unknown outline cells are not.
    unsigned ox,oy;
    check(p.costmap_->worldToMap(start.pose.position.x,start.pose.position.y-.175,ox,oy));
    p.costmap_->setCost(ox,oy,253);
    check(p.validateCurve(short_seed,optimized));
    p.costmap_->setCost(ox,oy,254);
    check(!p.validateCurve(short_seed,optimized));
    p.costmap_->setCost(ox,oy,255);
    check(!p.validateCurve(short_seed,optimized));
    p.costmap_->setCost(ox,oy,0);
    // Sparse cached endpoints are free, but a new intervening obstacle blocks reuse.
    p.cached_path_.poses={start,goal};p.cached_goal_=goal;
    p.costmap_->setCost(30,30,254);
    nav_msgs::msg::Path reused;
    check(!p.buildReusablePath(start,goal,reused));
    std::cout<<"PASS plugin rounding, exact endpoints, unknown fallback, blocked cache segment\n";
  }
};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  auto ros=std::make_shared<nav2_costmap_2d::Costmap2DROS>("planner_rounding_test");
  ros->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{}));
  ros->on_configure(rclcpp_lifecycle::State());
  luckrobot_nav2_plugins::LuckPlannerTestAccess::run(ros);
  ros->on_cleanup(rclcpp_lifecycle::State());ros.reset();rclcpp::shutdown();
}
