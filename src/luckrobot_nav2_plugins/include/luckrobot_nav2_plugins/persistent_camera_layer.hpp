#pragma once
#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <utility>
#include <vector>

namespace luckrobot_nav2_plugins
{
// For the non-rolling global camera layer only. Keep occupied world positions
// across SLAM map resizing; ordinary finite camera clearing rays still apply.
class PersistentCameraLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void matchSize() override;
  void reset() override
  {
    anchors_.clear();
    nav2_costmap_2d::ObstacleLayer::reset();
  }
private:
  std::vector<std::pair<double, double>> anchors_;
};
}
