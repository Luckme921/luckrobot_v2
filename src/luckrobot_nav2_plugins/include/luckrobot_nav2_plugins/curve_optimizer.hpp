#pragma once
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
namespace luckrobot_nav2_plugins {
// Native Humble Nav2 Ceres optimizer; caller MUST collision-check its result.
bool optimizeCurve(const nav_msgs::msg::Path & seed,
  nav_msgs::msg::Path & candidate, const nav2_costmap_2d::Costmap2D * map,
  double preferred_radius, double max_time);
}
