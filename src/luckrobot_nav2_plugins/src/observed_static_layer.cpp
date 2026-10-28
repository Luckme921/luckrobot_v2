#include "luckrobot_nav2_plugins/observed_static_layer.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
namespace luckrobot_nav2_plugins {
void ObservedStaticLayer::onInitialize() {
 nav2_costmap_2d::StaticLayer::onInitialize();
 auto node=node_.lock();
 scan_sub_=node->create_subscription<sensor_msgs::msg::LaserScan>("/scan_filtered",
  rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){
   std::lock_guard<std::mutex> lock(scan_mutex_);scan_=m;
  });
}
void ObservedStaticLayer::reset() {
 nav2_costmap_2d::StaticLayer::reset();
 evidence_.clear();last_stamp_=-1;have_correction_=false;
}
void ObservedStaticLayer::updateBounds(double x,double y,double yaw,
 double *minx,double *miny,double *maxx,double *maxy) {
 nav2_costmap_2d::StaticLayer::updateBounds(x,y,yaw,minx,miny,maxx,maxy);
 // Restoring a disproved static cell must also invalidate old inflation.
 // Small household maps: full bounds avoids stale inflated borders.
 if (map_received_ && !layered_costmap_->isRolling()) {
  *minx=std::min(*minx,origin_x_);*miny=std::min(*miny,origin_y_);
  *maxx=std::max(*maxx,origin_x_+size_x_*resolution_);
  *maxy=std::max(*maxy,origin_y_+size_y_*resolution_);
 }
}
void ObservedStaticLayer::updateCosts(nav2_costmap_2d::Costmap2D &master,
 int min_i,int min_j,int max_i,int max_j) {
 nav2_costmap_2d::StaticLayer::updateCosts(master,min_i,min_j,max_i,max_j);
 if (!enabled_ || !map_received_ || layered_costmap_->isRolling() ||
     map_frame_!=layered_costmap_->getGlobalFrameID()) return;
 if (width_!=size_x_ || height_!=size_y_ || ox_!=origin_x_ || oy_!=origin_y_ ||res_!=resolution_) {
  width_=size_x_;height_=size_y_;ox_=origin_x_;oy_=origin_y_;res_=resolution_;
  evidence_.assign(size_x_*size_y_,{});have_correction_=false;
 }
 if(evidence_.size()!=size_x_*size_y_)evidence_.assign(size_x_*size_y_,{});
 sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
 {std::lock_guard<std::mutex> lock(scan_mutex_);scan=scan_;}
 auto node=node_.lock();
 if (!node || !scan) return;
 const rclcpp::Time stamp(scan->header.stamp);
 const double age=(node->now()-stamp).seconds(), seconds=stamp.seconds();
 if(age<-.1 || age>.3) return; // stale sensor cannot authorize an override
 try {
  auto correction=tf_->lookupTransform(map_frame_,"odom",stamp);
  const double cx=correction.transform.translation.x,cy=correction.transform.translation.y;
  const double cyaw=tf2::getYaw(correction.transform.rotation);
  if(have_correction_ && (std::hypot(cx-correction_x_,cy-correction_y_)>.05 ||
    std::abs(std::remainder(cyaw-correction_yaw_,2*M_PI))>.03)) {
   evidence_.assign(evidence_.size(),{});have_correction_=false;
  }
  if(!have_correction_) {
   correction_x_=cx;correction_y_=cy;correction_yaw_=cyaw;have_correction_=true;
  }
  auto tr=tf_->lookupTransform(map_frame_,scan->header.frame_id,stamp);
  tf2::Transform transform;tf2::fromMsg(tr.transform,transform);
  if(std::abs(transform.getBasis()[2][2])<.98) return; // planar or inverted planar lidar
  const auto inv=transform.inverse();
  if(seconds>last_stamp_) {
   last_stamp_=seconds;
   const double radius=resolution_*std::sqrt(2.)*.5;
   for(unsigned int my=0;my<size_y_;++my) for(unsigned int mx=0;mx<size_x_;++mx) {
    const auto idx=getIndex(mx,my);
    if(costmap_[idx]!=nav2_costmap_2d::LETHAL_OBSTACLE){evidence_[idx]={};continue;}
    double wx,wy;mapToWorld(mx,my,wx,wy);
    auto p=inv*tf2::Vector3(wx,wy,tr.transform.translation.z);
    const bool free=finiteFreeCell(p.x(),p.y(),radius,scan->angle_min,
     scan->angle_increment,scan->range_min,scan->range_max,scan->ranges);
    evidence_[idx].observe(free,seconds);
    // A new endpoint near a cleared cell immediately restores its static cost.
    const double a=std::atan2(p.y(),p.x()),r=std::hypot(p.x(),p.y());
    if(scan->angle_increment>0) {
     const int beam=std::lround((a-scan->angle_min)/scan->angle_increment);
     if(beam>=0 && beam<static_cast<int>(scan->ranges.size()) &&
       std::isfinite(scan->ranges[beam]) && std::abs(scan->ranges[beam]-r)<radius+.10)
       evidence_[idx]={};
    }
   }
  }
  // This layer runs FIRST. Live lidar and camera layers add their own occupied
  // cells afterwards; lidar never clears the separate camera height layer.
  for(int y=min_j;y<max_j;++y)for(int x=min_i;x<max_i;++x) {
   double wx,wy;master.mapToWorld(x,y,wx,wy);unsigned int mx,my;
   if(worldToMap(wx,wy,mx,my) && evidence_[getIndex(mx,my)].cleared &&
      costmap_[getIndex(mx,my)]==nav2_costmap_2d::LETHAL_OBSTACLE)
     master.setCost(x,y,nav2_costmap_2d::FREE_SPACE);
  }
 } catch(const tf2::TransformException &) {return;}
}
}
PLUGINLIB_EXPORT_CLASS(luckrobot_nav2_plugins::ObservedStaticLayer,nav2_costmap_2d::Layer)
