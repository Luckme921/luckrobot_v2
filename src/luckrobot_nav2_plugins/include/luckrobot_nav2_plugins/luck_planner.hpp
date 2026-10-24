#ifndef LUCKROBOT_NAV2_PLUGINS__LUCK_PLANNER_HPP_
#define LUCKROBOT_NAV2_PLUGINS__LUCK_PLANNER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace luckrobot_nav2_plugins
{

class LuckPlanner : public nav2_core::GlobalPlanner
{
public:
  LuckPlanner() = default;
  ~LuckPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  friend struct LuckPlannerTestAccess;
  struct Cell
  {
    unsigned int x{0};
    unsigned int y{0};
  };

  std::vector<Cell> search(
    const Cell & start, const Cell & requested_goal,
    bool allow_tolerance_goal) const;
  std::vector<Cell> simplify(const std::vector<Cell> & cells) const;
  nav_msgs::msg::Path cellsToPath(
    const std::vector<Cell> & cells,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;
  bool buildReusablePath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & path);
  bool goalsMatch(
    const geometry_msgs::msg::PoseStamped & first,
    const geometry_msgs::msg::PoseStamped & second) const;
  bool isTraversable(unsigned int x, unsigned int y) const;
  bool lineIsTraversable(const Cell & start, const Cell & goal) const;
  bool lineIsClear(const Cell & start, const Cell & goal) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string plugin_name_;
  std::string global_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("LuckPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  bool allow_unknown_{true};
  double tolerance_{0.35};
  int max_planning_cost_{252};
  int smoothing_max_cost_{100};
  double cost_penalty_{5.0};
  double turn_penalty_{0.20};
  double interpolation_resolution_{0.05};
  bool corner_rounding_enabled_{true};
  double corner_rounding_distance_{0.25};
  int max_iterations_{1000000};
  bool route_hysteresis_enabled_{true};
  double route_reuse_goal_tolerance_{0.15};
  double route_reuse_goal_yaw_tolerance_{0.35};
  double route_reuse_start_tolerance_{0.45};
  nav_msgs::msg::Path cached_path_;
  geometry_msgs::msg::PoseStamped cached_goal_;
  std::size_t cached_progress_index_{0};
};

}  // namespace luckrobot_nav2_plugins

#endif  // LUCKROBOT_NAV2_PLUGINS__LUCK_PLANNER_HPP_
