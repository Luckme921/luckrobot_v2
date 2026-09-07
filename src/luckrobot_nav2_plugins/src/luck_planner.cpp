#include "luckrobot_nav2_plugins/luck_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace luckrobot_nav2_plugins
{

using nav2_util::declare_parameter_if_not_declared;

namespace
{

constexpr int kDirectionCount = 8;
constexpr int kStateDirections = 9;
constexpr int kNoDirection = 8;
constexpr std::array<int, kDirectionCount> kDx{{1, 1, 0, -1, -1, -1, 0, 1}};
constexpr std::array<int, kDirectionCount> kDy{{0, 1, 1, 1, 0, -1, -1, -1}};

struct QueueEntry
{
  double f{0.0};
  double g{0.0};
  std::int64_t state{0};

  bool operator<(const QueueEntry & other) const
  {
    return f > other.f;
  }
};

}  // namespace

void LuckPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("LuckPlanner cannot lock planner_server");
  }

  node_ = parent;
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  plugin_name_ = std::move(name);
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".allow_unknown", rclcpp::ParameterValue(allow_unknown_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".tolerance", rclcpp::ParameterValue(tolerance_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cost_penalty", rclcpp::ParameterValue(cost_penalty_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".turn_penalty", rclcpp::ParameterValue(turn_penalty_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".interpolation_resolution",
    rclcpp::ParameterValue(interpolation_resolution_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_planning_cost", rclcpp::ParameterValue(max_planning_cost_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".smoothing_max_cost", rclcpp::ParameterValue(smoothing_max_cost_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_iterations", rclcpp::ParameterValue(max_iterations_));

  node->get_parameter(plugin_name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(plugin_name_ + ".tolerance", tolerance_);
  node->get_parameter(plugin_name_ + ".cost_penalty", cost_penalty_);
  node->get_parameter(plugin_name_ + ".turn_penalty", turn_penalty_);
  node->get_parameter(
    plugin_name_ + ".interpolation_resolution", interpolation_resolution_);
  node->get_parameter(plugin_name_ + ".max_planning_cost", max_planning_cost_);
  node->get_parameter(plugin_name_ + ".smoothing_max_cost", smoothing_max_cost_);
  node->get_parameter(plugin_name_ + ".max_iterations", max_iterations_);

  if (tolerance_ < 0.0 || interpolation_resolution_ <= 0.0 ||
    max_planning_cost_ < 0 || max_planning_cost_ > 252 ||
    smoothing_max_cost_ < 0 || smoothing_max_cost_ > max_planning_cost_ ||
    max_iterations_ <= 0)
  {
    throw nav2_core::PlannerException("LuckPlanner parameters are outside safe bounds");
  }

  RCLCPP_INFO(
    logger_,
    "Configured %s: A* cost<=%d, smooth cost<=%d, tolerance=%.2f m",
    plugin_name_.c_str(), max_planning_cost_, smoothing_max_cost_, tolerance_);
}

void LuckPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaned up %s", plugin_name_.c_str());
}

void LuckPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activated %s", plugin_name_.c_str());
}

void LuckPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivated %s", plugin_name_.c_str());
}

nav_msgs::msg::Path LuckPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path empty;
  empty.header.frame_id = global_frame_;
  empty.header.stamp = clock_->now();

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      logger_, "%s requires start and goal in %s (received %s and %s)",
      plugin_name_.c_str(), global_frame_.c_str(),
      start.header.frame_id.c_str(), goal.header.frame_id.c_str());
    return empty;
  }

  Cell start_cell;
  Cell goal_cell;
  if (!costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y, start_cell.x, start_cell.y))
  {
    RCLCPP_ERROR(logger_, "%s start is outside the global costmap", plugin_name_.c_str());
    return empty;
  }
  if (!costmap_->worldToMap(
      goal.pose.position.x, goal.pose.position.y, goal_cell.x, goal_cell.y))
  {
    RCLCPP_ERROR(logger_, "%s goal is outside the global costmap", plugin_name_.c_str());
    return empty;
  }

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  std::vector<Cell> cells;
  if (isTraversable(goal_cell.x, goal_cell.y)) {
    cells = search(start_cell, goal_cell, false);
  }
  if (cells.empty() && tolerance_ > 0.0) {
    cells = search(start_cell, goal_cell, true);
  }
  if (cells.empty()) {
    RCLCPP_WARN(
      logger_, "%s found no path to (%.2f, %.2f), including %.2f m tolerance",
      plugin_name_.c_str(), goal.pose.position.x, goal.pose.position.y, tolerance_);
    return empty;
  }

  const auto simplified = simplify(cells);
  auto path = cellsToPath(simplified, start, goal);
  lock.unlock();

  RCLCPP_DEBUG(
    logger_, "%s planned %zu grid cells -> %zu waypoints -> %zu dense poses",
    plugin_name_.c_str(), cells.size(), simplified.size(), path.poses.size());
  return path;
}

bool LuckPlanner::isTraversable(unsigned int x, unsigned int y) const
{
  if (x >= costmap_->getSizeInCellsX() || y >= costmap_->getSizeInCellsY()) {
    return false;
  }
  const unsigned char cost = costmap_->getCost(x, y);
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }
  return static_cast<int>(cost) <= max_planning_cost_;
}

std::vector<LuckPlanner::Cell> LuckPlanner::search(
  const Cell & start, const Cell & requested_goal,
  bool allow_tolerance_goal) const
{
  const std::size_t width = costmap_->getSizeInCellsX();
  const std::size_t height = costmap_->getSizeInCellsY();
  const std::size_t state_count = width * height * kStateDirections;
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> g_score(state_count, infinity);
  std::vector<std::int64_t> parent(state_count, -1);
  std::priority_queue<QueueEntry> open;

  const auto state_index = [width](unsigned int x, unsigned int y, int direction) {
      return static_cast<std::int64_t>(
        (static_cast<std::size_t>(y) * width + x) * kStateDirections + direction);
    };
  const auto cell_from_state = [width](std::int64_t state) {
      const std::size_t cell_index = static_cast<std::size_t>(state) / kStateDirections;
      return Cell{
        static_cast<unsigned int>(cell_index % width),
        static_cast<unsigned int>(cell_index / width)};
    };
  const double tolerance_cells = tolerance_ / costmap_->getResolution();
  const auto goal_distance = [&](unsigned int x, unsigned int y) {
      return std::hypot(
        static_cast<double>(static_cast<int>(x) - static_cast<int>(requested_goal.x)),
        static_cast<double>(static_cast<int>(y) - static_cast<int>(requested_goal.y)));
    };
  const auto is_goal = [&](unsigned int x, unsigned int y) {
      if (!allow_tolerance_goal) {
        return x == requested_goal.x && y == requested_goal.y;
      }
      return goal_distance(x, y) <= tolerance_cells && isTraversable(x, y);
    };
  const auto heuristic = [&](unsigned int x, unsigned int y) {
      const double distance = goal_distance(x, y);
      return allow_tolerance_goal ? std::max(0.0, distance - tolerance_cells) : distance;
    };

  const std::int64_t start_state = state_index(start.x, start.y, kNoDirection);
  g_score[static_cast<std::size_t>(start_state)] = 0.0;
  open.push(QueueEntry{heuristic(start.x, start.y), 0.0, start_state});

  std::int64_t reached_state = -1;
  int iterations = 0;
  while (!open.empty() && iterations++ < max_iterations_) {
    const auto current = open.top();
    open.pop();
    if (current.g > g_score[static_cast<std::size_t>(current.state)] + 1.0e-9) {
      continue;
    }
    const Cell cell = cell_from_state(current.state);
    const int old_direction = static_cast<int>(current.state % kStateDirections);
    if (is_goal(cell.x, cell.y)) {
      reached_state = current.state;
      break;
    }

    for (int direction = 0; direction < kDirectionCount; ++direction) {
      const int nx_signed = static_cast<int>(cell.x) + kDx[direction];
      const int ny_signed = static_cast<int>(cell.y) + kDy[direction];
      if (nx_signed < 0 || ny_signed < 0 ||
        nx_signed >= static_cast<int>(width) || ny_signed >= static_cast<int>(height))
      {
        continue;
      }
      const auto nx = static_cast<unsigned int>(nx_signed);
      const auto ny = static_cast<unsigned int>(ny_signed);
      if (!isTraversable(nx, ny)) {
        continue;
      }
      if (kDx[direction] != 0 && kDy[direction] != 0) {
        const auto side_x = static_cast<unsigned int>(
          static_cast<int>(cell.x) + kDx[direction]);
        const auto side_y = static_cast<unsigned int>(
          static_cast<int>(cell.y) + kDy[direction]);
        if (!isTraversable(side_x, cell.y) || !isTraversable(cell.x, side_y)) {
          continue;
        }
      }

      const unsigned char raw_cost = costmap_->getCost(nx, ny);
      const double normalized_cost = raw_cost == nav2_costmap_2d::NO_INFORMATION ? 1.0 :
        static_cast<double>(raw_cost) /
        static_cast<double>(nav2_costmap_2d::MAX_NON_OBSTACLE);
      const double move_distance =
        (kDx[direction] != 0 && kDy[direction] != 0) ? std::sqrt(2.0) : 1.0;
      const double direction_change =
        (old_direction != kNoDirection && old_direction != direction) ? turn_penalty_ : 0.0;
      const double candidate = current.g + move_distance *
        (1.0 + cost_penalty_ * normalized_cost) + direction_change;
      const std::int64_t next_state = state_index(nx, ny, direction);
      if (candidate + 1.0e-9 < g_score[static_cast<std::size_t>(next_state)]) {
        g_score[static_cast<std::size_t>(next_state)] = candidate;
        parent[static_cast<std::size_t>(next_state)] = current.state;
        open.push(QueueEntry{
          candidate + heuristic(nx, ny), candidate, next_state});
      }
    }
  }

  if (reached_state < 0) {
    return {};
  }
  std::vector<Cell> reversed;
  std::int64_t state = reached_state;
  while (state >= 0) {
    reversed.push_back(cell_from_state(state));
    if (state == start_state) {
      break;
    }
    state = parent[static_cast<std::size_t>(state)];
  }
  if (reversed.empty() || reversed.back().x != start.x || reversed.back().y != start.y) {
    return {};
  }
  std::reverse(reversed.begin(), reversed.end());
  return reversed;
}

bool LuckPlanner::lineIsClear(const Cell & start, const Cell & goal) const
{
  int x0 = static_cast<int>(start.x);
  int y0 = static_cast<int>(start.y);
  const int x1 = static_cast<int>(goal.x);
  const int y1 = static_cast<int>(goal.y);
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;
  int previous_x = x0;
  int previous_y = y0;

  while (true) {
    const unsigned char cost = costmap_->getCost(
      static_cast<unsigned int>(x0), static_cast<unsigned int>(y0));
    if (cost == nav2_costmap_2d::NO_INFORMATION ||
      static_cast<int>(cost) > smoothing_max_cost_)
    {
      return false;
    }
    if (x0 == x1 && y0 == y1) {
      return true;
    }
    const int doubled_error = 2 * error;
    if (doubled_error >= dy) {
      error += dy;
      x0 += sx;
    }
    if (doubled_error <= dx) {
      error += dx;
      y0 += sy;
    }
    // Conservative "supercover" check: a diagonal line may not squeeze
    // through two touching high-cost corners that Bresenham would skip.
    if (x0 != previous_x && y0 != previous_y) {
      const unsigned char side_a = costmap_->getCost(
        static_cast<unsigned int>(x0), static_cast<unsigned int>(previous_y));
      const unsigned char side_b = costmap_->getCost(
        static_cast<unsigned int>(previous_x), static_cast<unsigned int>(y0));
      if (side_a == nav2_costmap_2d::NO_INFORMATION ||
        side_b == nav2_costmap_2d::NO_INFORMATION ||
        static_cast<int>(side_a) > smoothing_max_cost_ ||
        static_cast<int>(side_b) > smoothing_max_cost_)
      {
        return false;
      }
    }
    previous_x = x0;
    previous_y = y0;
  }
}

std::vector<LuckPlanner::Cell> LuckPlanner::simplify(const std::vector<Cell> & cells) const
{
  if (cells.size() <= 2) {
    return cells;
  }
  std::vector<Cell> output;
  output.push_back(cells.front());
  std::size_t anchor = 0;
  while (anchor + 1 < cells.size()) {
    std::size_t candidate = cells.size() - 1;
    while (candidate > anchor + 1 && !lineIsClear(cells[anchor], cells[candidate])) {
      --candidate;
    }
    output.push_back(cells[candidate]);
    anchor = candidate;
  }
  return output;
}

nav_msgs::msg::Path LuckPlanner::cellsToPath(
  const std::vector<Cell> & cells,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();
  if (cells.empty()) {
    return path;
  }

  std::vector<std::pair<double, double>> points;
  points.emplace_back(start.pose.position.x, start.pose.position.y);
  for (std::size_t i = 1; i < cells.size(); ++i) {
    double wx = 0.0;
    double wy = 0.0;
    costmap_->mapToWorld(cells[i].x, cells[i].y, wx, wy);
    const double start_x = points.back().first;
    const double start_y = points.back().second;
    const double length = std::hypot(wx - start_x, wy - start_y);
    const int samples = std::max(
      1, static_cast<int>(std::ceil(length / interpolation_resolution_)));
    for (int sample = 1; sample <= samples; ++sample) {
      const double ratio = static_cast<double>(sample) / static_cast<double>(samples);
      points.emplace_back(
        start_x + ratio * (wx - start_x),
        start_y + ratio * (wy - start_y));
    }
  }

  unsigned int requested_goal_x = 0;
  unsigned int requested_goal_y = 0;
  if (costmap_->worldToMap(
      goal.pose.position.x, goal.pose.position.y,
      requested_goal_x, requested_goal_y) &&
    cells.back().x == requested_goal_x && cells.back().y == requested_goal_y &&
    !points.empty())
  {
    points.back() = {goal.pose.position.x, goal.pose.position.y};
  }

  path.poses.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = points[i].first;
    pose.pose.position.y = points[i].second;
    pose.pose.position.z = 0.0;
    double yaw = tf2::getYaw(goal.pose.orientation);
    if (i + 1 < points.size()) {
      yaw = std::atan2(
        points[i + 1].second - points[i].second,
        points[i + 1].first - points[i].first);
    }
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation = tf2::toMsg(quaternion);
    path.poses.push_back(pose);
  }
  if (!path.poses.empty()) {
    path.poses.back().pose.orientation = goal.pose.orientation;
  }
  return path;
}

}  // namespace luckrobot_nav2_plugins

PLUGINLIB_EXPORT_CLASS(
  luckrobot_nav2_plugins::LuckPlanner,
  nav2_core::GlobalPlanner)
