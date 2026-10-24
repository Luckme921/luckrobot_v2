#pragma once
#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <utility>
#include <vector>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace luckrobot_nav2_plugins
{
// For the non-rolling global camera layer only. Keep occupied world positions
// across SLAM map resizing; ordinary finite camera clearing rays still apply.
class PersistentCameraLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void onInitialize() override;
  void updateCosts(nav2_costmap_2d::Costmap2D & master, int min_i, int min_j,
    int max_i, int max_j) override;
  void matchSize() override;
  void reset() override
  {
    anchors_.clear();
    nav2_costmap_2d::ObstacleLayer::reset();
  }
private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_pub_;
  double last_debug_seconds_{-1.0};
  std::vector<std::pair<double, double>> anchors_;
};
}
