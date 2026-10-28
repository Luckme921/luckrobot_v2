#pragma once
#include <nav2_costmap_2d/static_layer.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <mutex>
#include "luckrobot_nav2_plugins/free_space_evidence.hpp"
namespace luckrobot_nav2_plugins {
class ObservedStaticLayer : public nav2_costmap_2d::StaticLayer {
public:
 void onInitialize() override;
 void updateBounds(double,double,double,double*,double*,double*,double*) override;
 void updateCosts(nav2_costmap_2d::Costmap2D &,int,int,int,int) override;
 void reset() override;
protected:
 rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
 std::mutex scan_mutex_;
 sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
 std::vector<FreeEvidence> evidence_;
 double ox_=0,oy_=0,res_=0,last_stamp_=-1;
 unsigned int width_=0,height_=0;
 bool have_correction_=false;
 double correction_x_=0,correction_y_=0,correction_yaw_=0;
};
}
