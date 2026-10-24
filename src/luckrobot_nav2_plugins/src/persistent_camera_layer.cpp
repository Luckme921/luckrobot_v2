#include "luckrobot_nav2_plugins/persistent_camera_layer.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <unordered_set>

namespace luckrobot_nav2_plugins
{
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
