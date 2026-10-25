#include "luckrobot_nav2_plugins/luck_planner.hpp"
#include "luckrobot_nav2_plugins/path_rounding.hpp"
#include "luckrobot_nav2_plugins/curve_optimizer.hpp"
#include <chrono>

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
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
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
constexpr double kPi = 3.14159265358979323846;
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
  declare_parameter_if_not_declared(node, plugin_name_ + ".use_astar", rclcpp::ParameterValue(use_astar_));
  declare_parameter_if_not_declared(node, plugin_name_ + ".constrained_curve_enabled",
    rclcpp::ParameterValue(constrained_curve_enabled_));
  declare_parameter_if_not_declared(node, plugin_name_ + ".preferred_turning_radius",
    rclcpp::ParameterValue(preferred_turning_radius_));
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
  declare_parameter_if_not_declared(node, plugin_name_ + ".corner_rounding_enabled",
    rclcpp::ParameterValue(corner_rounding_enabled_));
  declare_parameter_if_not_declared(node, plugin_name_ + ".corner_rounding_distance",
    rclcpp::ParameterValue(corner_rounding_distance_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".route_hysteresis_enabled",
    rclcpp::ParameterValue(route_hysteresis_enabled_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".route_reuse_goal_tolerance",
    rclcpp::ParameterValue(route_reuse_goal_tolerance_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".route_reuse_goal_yaw_tolerance",
    rclcpp::ParameterValue(route_reuse_goal_yaw_tolerance_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".route_reuse_start_tolerance",
    rclcpp::ParameterValue(route_reuse_start_tolerance_));

  node->get_parameter(plugin_name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(plugin_name_ + ".use_astar", use_astar_);
  node->get_parameter(plugin_name_ + ".constrained_curve_enabled", constrained_curve_enabled_);
  node->get_parameter(plugin_name_ + ".preferred_turning_radius", preferred_turning_radius_);
  node->get_parameter(plugin_name_ + ".tolerance", tolerance_);
  node->get_parameter(plugin_name_ + ".cost_penalty", cost_penalty_);
  node->get_parameter(plugin_name_ + ".turn_penalty", turn_penalty_);
  node->get_parameter(
    plugin_name_ + ".interpolation_resolution", interpolation_resolution_);
  node->get_parameter(plugin_name_ + ".max_planning_cost", max_planning_cost_);
  node->get_parameter(plugin_name_ + ".smoothing_max_cost", smoothing_max_cost_);
  node->get_parameter(plugin_name_ + ".max_iterations", max_iterations_);
  node->get_parameter(plugin_name_ + ".corner_rounding_enabled", corner_rounding_enabled_);
  node->get_parameter(plugin_name_ + ".corner_rounding_distance", corner_rounding_distance_);
  node->get_parameter(
    plugin_name_ + ".route_hysteresis_enabled", route_hysteresis_enabled_);
  node->get_parameter(
    plugin_name_ + ".route_reuse_goal_tolerance", route_reuse_goal_tolerance_);
  node->get_parameter(
    plugin_name_ + ".route_reuse_goal_yaw_tolerance",
    route_reuse_goal_yaw_tolerance_);
  node->get_parameter(
    plugin_name_ + ".route_reuse_start_tolerance", route_reuse_start_tolerance_);

  if (!std::isfinite(preferred_turning_radius_) || preferred_turning_radius_<.20 ||
    preferred_turning_radius_>1.0 ||
    !std::isfinite(corner_rounding_distance_) || corner_rounding_distance_ < 0.0 ||
    corner_rounding_distance_ > 0.40 ||
    tolerance_ < 0.0 || interpolation_resolution_ <= 0.0 ||
    max_planning_cost_ < 0 || max_planning_cost_ > 252 ||
    smoothing_max_cost_ < 0 || smoothing_max_cost_ > max_planning_cost_ ||
    max_iterations_ <= 0 || route_reuse_goal_tolerance_ < 0.0 ||
    route_reuse_goal_yaw_tolerance_ < 0.0 || route_reuse_start_tolerance_ <= 0.0)
  {
    throw nav2_core::PlannerException("LuckPlanner parameters are outside safe bounds");
  }

  RCLCPP_INFO(
    logger_,
    "Configured %s: grid cost<=%d, smooth cost<=%d, tolerance=%.2f m, "
    "route hysteresis=%s (start %.2f m, goal %.2f m / %.1f deg)",
    plugin_name_.c_str(), max_planning_cost_, smoothing_max_cost_, tolerance_,
    route_hysteresis_enabled_ ? "on" : "off", route_reuse_start_tolerance_,
    route_reuse_goal_tolerance_, route_reuse_goal_yaw_tolerance_ * 180.0 / kPi);
  RCLCPP_INFO(logger_, "Search=%s, constrained curve=%s, preferred radius=%.2f m",
    use_astar_ ? "A*" : "Dijkstra", constrained_curve_enabled_ ? "on" : "off",
    preferred_turning_radius_);
}

void LuckPlanner::cleanup()
{
  cached_path_.poses.clear();
  cached_progress_index_ = 0;
  RCLCPP_INFO(logger_, "Cleaned up %s", plugin_name_.c_str());
}

void LuckPlanner::activate()
{
  cached_path_.poses.clear();
  cached_progress_index_ = 0;
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

  nav_msgs::msg::Path reused_path;
  if (route_hysteresis_enabled_ && buildReusablePath(start, goal, reused_path)) {
    lock.unlock();
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 5000,
      "%s keeps the current safe route (%zu poses remain); refusing a shorter "
      "route-only switch",
      plugin_name_.c_str(), reused_path.poses.size());
    return reused_path;
  }

  std::vector<Cell> cells;
  if (isTraversable(goal_cell.x, goal_cell.y)) {
    cells = search(start_cell, goal_cell, false);
  }
  if (cells.empty() && tolerance_ > 0.0) {
    cells = search(start_cell, goal_cell, true);
  }
  if (cells.empty()) {
    cached_path_.poses.clear();
    cached_progress_index_ = 0;
    RCLCPP_WARN(
      logger_, "%s found no path to (%.2f, %.2f), including %.2f m tolerance",
      plugin_name_.c_str(), goal.pose.position.x, goal.pose.position.y, tolerance_);
    return empty;
  }

  const auto simplified = simplify(cells);
  auto path = cellsToPath(simplified, start, goal);
  if (constrained_curve_enabled_) {
    const auto started=std::chrono::steady_clock::now();
    nav_msgs::msg::Path candidate;
    const bool solved=optimizeCurve(path,candidate,costmap_,preferred_turning_radius_,.08);
    const bool accepted=solved && validateCurve(path,candidate);
    if(accepted) {path=std::move(candidate);}
    const double ms=std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-started).count();
    RCLCPP_INFO(logger_, "Curve optimization %s (solver=%s): %.1f ms, %zu poses",
      accepted ? "accepted" : "fallback to checked seed", solved ? "usable" : "failed", ms,path.poses.size());
  }
  cached_path_ = path;
  cached_goal_ = goal;
  cached_progress_index_ = 0;
  lock.unlock();

  RCLCPP_DEBUG(
    logger_, "%s planned %zu grid cells -> %zu waypoints -> %zu dense poses",
    plugin_name_.c_str(), cells.size(), simplified.size(), path.poses.size());
  return path;
}

bool LuckPlanner::goalsMatch(
  const geometry_msgs::msg::PoseStamped & first,
  const geometry_msgs::msg::PoseStamped & second) const
{
  if (first.header.frame_id != second.header.frame_id) {
    return false;
  }
  const double position_error = std::hypot(
    first.pose.position.x - second.pose.position.x,
    first.pose.position.y - second.pose.position.y);
  const double yaw_error = std::abs(std::remainder(
    tf2::getYaw(first.pose.orientation) - tf2::getYaw(second.pose.orientation),
    2.0 * kPi));
  return position_error <= route_reuse_goal_tolerance_ &&
         yaw_error <= route_reuse_goal_yaw_tolerance_;
}

bool LuckPlanner::buildReusablePath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & path)
{
  if (cached_path_.poses.size() < 2 || !goalsMatch(cached_goal_, goal)) {
    return false;
  }

  const std::size_t first_candidate = std::min(
    cached_progress_index_, cached_path_.poses.size() - 1);
  std::size_t nearest_index = first_candidate;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = first_candidate; i < cached_path_.poses.size(); ++i) {
    const double distance = std::hypot(
      cached_path_.poses[i].pose.position.x - start.pose.position.x,
      cached_path_.poses[i].pose.position.y - start.pose.position.y);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = i;
    }
  }
  if (nearest_distance > route_reuse_start_tolerance_) {
    return false;
  }

  Cell current_cell;
  Cell nearest_cell;
  if (!costmap_->worldToMap(
      start.pose.position.x, start.pose.position.y,
      current_cell.x, current_cell.y) ||
    !costmap_->worldToMap(
      cached_path_.poses[nearest_index].pose.position.x,
      cached_path_.poses[nearest_index].pose.position.y,
      nearest_cell.x, nearest_cell.y) ||
    !lineIsTraversable(current_cell, nearest_cell))
  {
    return false;
  }

  // Validate segments, not just vertices: even dense samples can straddle a
  // newly occupied cell or diagonal corner after rounding / map resizing.
  Cell previous = nearest_cell;
  for (std::size_t i = nearest_index; i < cached_path_.poses.size(); ++i) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_->worldToMap(
        cached_path_.poses[i].pose.position.x,
        cached_path_.poses[i].pose.position.y, mx, my) ||
      !isTraversable(mx, my) || !lineIsTraversable(previous, Cell{mx, my}))
    {
      return false;
    }
    previous = Cell{mx, my};
  }

  cached_progress_index_ = nearest_index;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();
  path.poses.reserve(cached_path_.poses.size() - nearest_index);
  // Preserve the geometry that was accepted when this route was created.
  // Prepending the latest robot pose on every periodic replan creates a new
  // short connector into the cached route.  On a differential-drive chassis,
  // that connector can alternate across the route as the robot crosses the
  // centerline, changing the near/far lookahead bend and producing repeated
  // left/right steering even though the safe global route never changed.
  //
  // The controller already projects the robot onto the closest remaining path
  // pose and tracks forward from there.  The traversability check above still
  // validates the physical connection from the current pose to that closest
  // pose, so omitting the synthetic connector does not bypass obstacle checks.
  for (std::size_t i = nearest_index; i < cached_path_.poses.size(); ++i) {
    auto pose = cached_path_.poses[i];
    pose.header = path.header;
    path.poses.push_back(std::move(pose));
  }
  if (path.poses.size() < 2) {
    return false;
  }
  path.poses.back().pose.orientation = goal.pose.orientation;
  return true;
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

bool LuckPlanner::validateCurve(
  const nav_msgs::msg::Path & seed, const nav_msgs::msg::Path & candidate) const
{
  auto reject=[&](const char * why) {RCLCPP_DEBUG(logger_, "Curve rejected: %s",why);return false;};
  if(seed.poses.size()<2 || candidate.poses.size()<2) return false;
  const auto footprint=costmap_ros_->getRobotFootprint();
  if(footprint.size()<3) return false;
  auto separation=[](const auto & a,const auto & b) {
    return std::hypot(a.pose.position.x-b.pose.position.x,a.pose.position.y-b.pose.position.y);
  };
  if(separation(seed.poses.front(),candidate.poses.front())>1e-6 ||
    separation(seed.poses.back(),candidate.poses.back())>1e-6) return false;
  double seed_length=0, new_length=0;
  int cost_limit=smoothing_max_cost_;
  for(size_t i=0;i<seed.poses.size();++i) {
    Cell c;
    const auto & p=seed.poses[i].pose.position;
    if(!costmap_->worldToMap(p.x,p.y,c.x,c.y)) return false;
    const auto cost=costmap_->getCost(c.x,c.y);
    if(cost==nav2_costmap_2d::NO_INFORMATION) return reject("seed unknown");
    cost_limit=std::max(cost_limit,static_cast<int>(cost));
    if(i) seed_length+=separation(seed.poses[i],seed.poses[i-1]);
  }
  cost_limit=std::min(cost_limit,max_planning_cost_);
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap_);
  Cell previous;
  bool have_previous=false;
  for(size_t i=1;i<candidate.poses.size();++i) {
    const auto & a=candidate.poses[i-1].pose.position;
    const auto & b=candidate.poses[i].pose.position;
    if(!std::isfinite(a.x) || !std::isfinite(a.y) ||
      !std::isfinite(b.x) || !std::isfinite(b.y)) return false;
    const double length=std::hypot(b.x-a.x,b.y-a.y);
    new_length+=length;
    const int n=std::max(1,static_cast<int>(std::ceil(length/(costmap_->getResolution()*.25))));
    const double yaw=std::atan2(b.y-a.y,b.x-a.x);
    for(int j=0;j<=n;++j) {
      const double t=static_cast<double>(j)/n, x=a.x+t*(b.x-a.x), y=a.y+t*(b.y-a.y);
      Cell c;
      if(!costmap_->worldToMap(x,y,c.x,c.y)) return false;
      if(static_cast<int>(costmap_->getCost(c.x,c.y))>cost_limit) return reject("center cost");
      if(have_previous && !lineIsTraversable(previous,c)) return false;
      previous=c; have_previous=true;
      const double cost=checker.footprintCostAtPose(x,y,yaw,footprint);
      // Inflation encodes clearance for the robot CENTER. Applying that
      // threshold again at the outline double counts the robot radius.
      // Match the controller: center <=252, outline excludes lethal/unknown.
      if(!std::isfinite(cost) || cost<0 || cost>=nav2_costmap_2d::LETHAL_OBSTACLE) return reject("footprint");
      // Keep the optimization within the searched corridor, avoiding a jump
      // to another homotopy or a large shortcut through an unobserved region.
      double nearest=std::numeric_limits<double>::infinity();
      for(size_t k=1;k<seed.poses.size();++k) {
        const auto & u=seed.poses[k-1].pose.position; const auto & v=seed.poses[k].pose.position;
        const double dx=v.x-u.x,dy=v.y-u.y,den=dx*dx+dy*dy;
        const double r=den>1e-12 ? std::clamp(((x-u.x)*dx+(y-u.y)*dy)/den,0.,1.) : 0.;
        nearest=std::min(nearest,std::hypot(x-u.x-r*dx,y-u.y-r*dy));
      }
      if(nearest>.45) return reject("corridor deviation");
    }
  }
  return new_length<=seed_length*1.25+.05;
}

bool LuckPlanner::lineIsTraversable(const Cell & start, const Cell & goal) const
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
    if (!isTraversable(
        static_cast<unsigned int>(x0), static_cast<unsigned int>(y0)))
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
    if (x0 != previous_x && y0 != previous_y &&
      (!isTraversable(static_cast<unsigned int>(x0), static_cast<unsigned int>(previous_y)) ||
      !isTraversable(static_cast<unsigned int>(previous_x), static_cast<unsigned int>(y0))))
    {
      return false;
    }
    previous_x = x0;
    previous_y = y0;
  }
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
      if (!use_astar_) {return 0.0;}
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
    points.emplace_back(wx, wy);
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

  if (corner_rounding_enabled_) {
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap_);
    const auto footprint = costmap_ros_->getRobotFootprint();
    std::vector<path_rounding::Point> sparse;
    for (const auto & p : points) {sparse.push_back({p.first,p.second});}
    const auto rounded = path_rounding::round(sparse, corner_rounding_distance_,
      std::min(interpolation_resolution_,costmap_->getResolution()*.25),
      [&](path_rounding::Point a, path_rounding::Point b) {
        Cell ca,cb;
        if (!costmap_->worldToMap(a.x,a.y,ca.x,ca.y) ||
          !costmap_->worldToMap(b.x,b.y,cb.x,cb.y) || !lineIsClear(ca,cb) ||
          footprint.size()<3) {return false;}
        const double yaw=std::atan2(b.y-a.y,b.x-a.x);
        const int n=std::max(1,static_cast<int>(std::ceil(
          path_rounding::distance(a,b)/(costmap_->getResolution()*.5))));
        for (int j=0;j<=n;++j) {
          const double t=static_cast<double>(j)/n;
          const double cost=checker.footprintCostAtPose(
            a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),yaw,footprint);
          if (!std::isfinite(cost) || cost<0 || cost>=nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            return false;
          }
        }
        return true;
      });
    points.clear();
    for (const auto & p : rounded) {points.emplace_back(p.x,p.y);}
  }
  std::vector<std::pair<double,double>> dense{points.front()};
  const double spacing=std::min(interpolation_resolution_,costmap_->getResolution()*.5);
  for (size_t i=1;i<points.size();++i) {
    const auto a=points[i-1], b=points[i];
    const double length=std::hypot(b.first-a.first,b.second-a.second);
    if (length<1e-9) {continue;}
    const int n=std::max(1,static_cast<int>(std::ceil(length/spacing)));
    for (int j=1;j<=n;++j) {
      const double t=static_cast<double>(j)/n;
      dense.emplace_back(a.first+t*(b.first-a.first),a.second+t*(b.second-a.second));
    }
  }
  points=std::move(dense);
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
