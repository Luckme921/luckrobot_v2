#include "luckrobot_nav2_plugins/persistent_camera_layer.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <unordered_set>

namespace luckrobot_nav2_plugins
{
void PersistentCameraLayer::onInitialize()
{
  nav2_costmap_2d::ObstacleLayer::onInitialize();
  auto node = node_.lock();
  if (node) {
    debug_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/diagnostics/global_camera_obstacle_grid", rclcpp::QoS(1).transient_local());
  }
}

void PersistentCameraLayer::updateCosts(nav2_costmap_2d::Costmap2D & master,
  int min_i, int min_j, int max_i, int max_j)
{
  nav2_costmap_2d::ObstacleLayer::updateCosts(master, min_i, min_j, max_i, max_j);
  auto node = node_.lock();
  if (!node || !debug_pub_) {return;}
  const auto now = node->now();
  if (last_debug_seconds_ >= 0 && now.seconds() >= last_debug_seconds_ &&
    now.seconds() - last_debug_seconds_ < 1.0) {return;}
  last_debug_seconds_ = now.seconds();
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = now;
  grid.header.frame_id = layered_costmap_->getGlobalFrameID();
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
    grid.info.resolution = resolution_;
    grid.info.width = size_x_; grid.info.height = size_y_;
    grid.info.origin.position.x = origin_x_; grid.info.origin.position.y = origin_y_;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(size_x_ * size_y_);
    for (size_t i = 0; i < grid.data.size(); ++i) {
      const auto cost = costmap_[i];
      grid.data[i] = cost == nav2_costmap_2d::NO_INFORMATION ? -1 :
        (cost == nav2_costmap_2d::LETHAL_OBSTACLE ? 100 : 0);
    }
  }
  debug_pub_->publish(grid);
}

void PersistentCameraLayer::matchSize()
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
  std::vector<std::pair<double, double>> keep;
  std::unordered_set<unsigned int> anchored;
  // Reuse original world anchors rather than repeatedly rounding to each new
  // grid origin. Drop any old anchor whose cell was genuinely cleared.
  for (const auto & p : anchors_) {
    unsigned int x, y;
    if (worldToMap(p.first, p.second, x, y) &&
      getCost(x, y) == nav2_costmap_2d::LETHAL_OBSTACLE)
    {
      keep.push_back(p);
      anchored.insert(getIndex(x, y));
    }
  }
  for (unsigned int y = 0; y < getSizeInCellsY(); ++y) {
    for (unsigned int x = 0; x < getSizeInCellsX(); ++x) {
      if (getCost(x, y) != nav2_costmap_2d::LETHAL_OBSTACLE ||
        anchored.count(getIndex(x, y))) {continue;}
      double wx, wy;
      mapToWorld(x, y, wx, wy);
      keep.emplace_back(wx, wy);
    }
  }
  nav2_costmap_2d::CostmapLayer::matchSize();
  anchors_.clear();
  for (const auto & p : keep) {
    unsigned int x, y;
    if (worldToMap(p.first, p.second, x, y)) {
      setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
      anchors_.push_back(p);
      addExtraBounds(p.first - resolution_, p.second - resolution_,
        p.first + resolution_, p.second + resolution_);
    }
  }
}
}
PLUGINLIB_EXPORT_CLASS(luckrobot_nav2_plugins::PersistentCameraLayer, nav2_costmap_2d::Layer)
