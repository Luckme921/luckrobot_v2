#include "luckrobot_nav2_plugins/luck_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace luckrobot_nav2_plugins
{

using nav2_util::declare_parameter_if_not_declared;

void LuckController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("LuckController cannot lock controller_server");
  }

  node_ = parent;
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();
  plugin_name_ = std::move(name);
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  auto declare_double = [&](const std::string & key, double value) {
      declare_parameter_if_not_declared(
        node, plugin_name_ + "." + key, rclcpp::ParameterValue(value));
    };
  auto declare_bool = [&](const std::string & key, bool value) {
      declare_parameter_if_not_declared(
        node, plugin_name_ + "." + key, rclcpp::ParameterValue(value));
    };

  declare_double("desired_linear_vel", desired_linear_vel_);
  declare_double("min_linear_vel", min_linear_vel_);
  declare_double("max_angular_vel", max_angular_vel_);
  declare_double("rotate_angular_vel", rotate_angular_vel_);
  declare_double("lookahead_dist", lookahead_dist_);
  declare_double("approach_dist", approach_dist_);
  declare_double("goal_rotate_dist", goal_rotate_dist_);
  declare_double("goal_position_tolerance", goal_position_tolerance_);
  declare_double("goal_yaw_tolerance", goal_yaw_tolerance_);
  declare_double("rotate_enter_angle", rotate_enter_angle_);
  declare_double("rotate_exit_angle", rotate_exit_angle_);
  declare_double("hard_rotate_angle", hard_rotate_angle_);
  declare_double("rotate_gain", rotate_gain_);
  declare_double("path_heading_gain", path_heading_gain_);
  declare_double("curvature_speed_gain", curvature_speed_gain_);
  declare_double("cost_speed_gain", cost_speed_gain_);
  declare_double("min_cost_speed_scale", min_cost_speed_scale_);
  declare_double("max_angular_accel", max_angular_accel_);
  declare_double("max_angular_decel", max_angular_decel_);
  declare_double("transform_tolerance", transform_tolerance_);
  declare_double("collision_horizon", collision_horizon_);
  declare_bool("use_collision_detection", use_collision_detection_);

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".min_linear_vel", min_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".rotate_angular_vel", rotate_angular_vel_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".approach_dist", approach_dist_);
  node->get_parameter(plugin_name_ + ".goal_rotate_dist", goal_rotate_dist_);
  node->get_parameter(
    plugin_name_ + ".goal_position_tolerance", goal_position_tolerance_);
  node->get_parameter(plugin_name_ + ".goal_yaw_tolerance", goal_yaw_tolerance_);
  node->get_parameter(plugin_name_ + ".rotate_enter_angle", rotate_enter_angle_);
  node->get_parameter(plugin_name_ + ".rotate_exit_angle", rotate_exit_angle_);
  node->get_parameter(plugin_name_ + ".hard_rotate_angle", hard_rotate_angle_);
  node->get_parameter(plugin_name_ + ".rotate_gain", rotate_gain_);
  node->get_parameter(plugin_name_ + ".path_heading_gain", path_heading_gain_);
  node->get_parameter(
    plugin_name_ + ".curvature_speed_gain", curvature_speed_gain_);
  node->get_parameter(plugin_name_ + ".cost_speed_gain", cost_speed_gain_);
  node->get_parameter(
    plugin_name_ + ".min_cost_speed_scale", min_cost_speed_scale_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".max_angular_decel", max_angular_decel_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(plugin_name_ + ".collision_horizon", collision_horizon_);
  node->get_parameter(
    plugin_name_ + ".use_collision_detection", use_collision_detection_);

  double controller_frequency = 10.0;
  node->get_parameter("controller_frequency", controller_frequency);
  control_period_ = controller_frequency > 0.0 ? 1.0 / controller_frequency : 0.10;
  base_desired_linear_vel_ = desired_linear_vel_;

  if (!(rotate_exit_angle_ < rotate_enter_angle_ &&
    rotate_enter_angle_ < hard_rotate_angle_))
  {
    throw nav2_core::PlannerException(
            "LuckController requires rotate_exit_angle < rotate_enter_angle < hard_rotate_angle");
  }
  if (lookahead_dist_ <= 0.0 || desired_linear_vel_ <= 0.0 ||
    max_angular_vel_ <= 0.0 || max_angular_accel_ < 0.0 || max_angular_decel_ <= 0.0)
  {
    throw nav2_core::PlannerException("LuckController speed and lookahead parameters must be positive");
  }

  collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<
      nav2_costmap_2d::Costmap2D *>>(costmap_);
  collision_checker_->setCostmap(costmap_);

  lookahead_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "/luck_controller/lookahead", rclcpp::QoS(10));
  tracking_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/luck_controller/tracking_error", rclcpp::QoS(10));

  RCLCPP_INFO(
    logger_,
    "Configured %s: v=%.2f m/s, lookahead=%.2f m, rotate hysteresis %.2f/%.2f rad",
    plugin_name_.c_str(), desired_linear_vel_, lookahead_dist_,
    rotate_enter_angle_, rotate_exit_angle_);
}

void LuckController::cleanup()
{
  std::lock_guard<std::mutex> lock(plan_mutex_);
  global_plan_.poses.clear();
  collision_checker_.reset();
  lookahead_pub_.reset();
  tracking_pub_.reset();
  RCLCPP_INFO(logger_, "Cleaned up %s", plugin_name_.c_str());
}

void LuckController::activate()
{
  lookahead_pub_->on_activate();
  tracking_pub_->on_activate();
  RCLCPP_INFO(logger_, "Activated %s", plugin_name_.c_str());
}

void LuckController::deactivate()
{
  lookahead_pub_->on_deactivate();
  tracking_pub_->on_deactivate();
  RCLCPP_INFO(logger_, "Deactivated %s", plugin_name_.c_str());
}

void LuckController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    throw nav2_core::PlannerException("LuckController received an empty path");
  }
  std::lock_guard<std::mutex> lock(plan_mutex_);
  bool new_navigation_goal = global_plan_.poses.empty();
  if (!new_navigation_goal) {
    const auto & old_goal = global_plan_.poses.back().pose.position;
    const auto & new_goal = path.poses.back().pose.position;
    new_navigation_goal = std::hypot(
      new_goal.x - old_goal.x, new_goal.y - old_goal.y) > 0.20;
  }
  global_plan_ = path;
  // Nav2 may replace the path every planning cycle. Preserve the rotate/drive
  // hysteresis for same-goal replans so the robot does not alternate between
  // turning and driving. A materially different goal starts a fresh state.
  if (new_navigation_goal) {
    rotate_mode_ = false;
  }
}

geometry_msgs::msg::TwistStamped LuckController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> plan_lock(plan_mutex_);
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("LuckController has no path");
  }

  const auto transformed_plan = transformPlan(pose);
  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("LuckController could not transform the path");
  }

  const auto target = selectTrackingTarget(transformed_plan, pose, lookahead_dist_);
  const auto & goal = transformed_plan.poses.back();
  const double robot_yaw = tf2::getYaw(pose.pose.orientation);
  const double goal_yaw = tf2::getYaw(goal.pose.orientation);
  const double goal_dx = goal.pose.position.x - pose.pose.position.x;
  const double goal_dy = goal.pose.position.y - pose.pose.position.y;
  const double distance_to_goal = std::hypot(goal_dx, goal_dy);
  const double goal_yaw_error = normalizeAngle(goal_yaw - robot_yaw);

  // Keep the controller's terminal phase synchronized with the GoalChecker
  // selected by Nav2. V11 used 0.12 m here while SimpleGoalChecker required
  // 0.08 m, so the controller stopped translating before Nav2 could latch XY.
  double active_position_tolerance = goal_position_tolerance_;
  double active_yaw_tolerance = goal_yaw_tolerance_;
  if (goal_checker != nullptr) {
    geometry_msgs::msg::Pose pose_tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    if (goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
      const double reported_position_tolerance = std::abs(pose_tolerance.position.x);
      if (std::isfinite(reported_position_tolerance) &&
        reported_position_tolerance > 0.0)
      {
        active_position_tolerance = reported_position_tolerance;
      }
      const auto & q = pose_tolerance.orientation;
      const bool valid_tolerance_quaternion =
        std::isfinite(q.x) && std::isfinite(q.y) &&
        std::isfinite(q.z) && std::isfinite(q.w) &&
        (std::abs(q.x) + std::abs(q.y) + std::abs(q.z) + std::abs(q.w) > 1.0e-9);
      if (valid_tolerance_quaternion) {
        const double reported_yaw_tolerance = std::abs(tf2::getYaw(q));
        if (std::isfinite(reported_yaw_tolerance) && reported_yaw_tolerance > 0.0) {
          active_yaw_tolerance = reported_yaw_tolerance;
        }
      }
    }
  }

  const double dx = target.pose.pose.position.x - pose.pose.position.x;
  const double dy = target.pose.pose.position.y - pose.pose.position.y;
  const double target_x = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  const double target_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
  const double carrot_distance_sq = target_x * target_x + target_y * target_y;
  const double carrot_distance = std::sqrt(carrot_distance_sq);
  const double angle_to_carrot = std::atan2(target_y, target_x);
  const double path_heading_error = normalizeAngle(target.tangent_yaw - robot_yaw);
  const double curvature = carrot_distance_sq > 1.0e-6 ?
    2.0 * target_y / carrot_distance_sq : 0.0;

  geometry_msgs::msg::TwistStamped command;
  command.header = pose.header;
  double linear = 0.0;
  double angular = 0.0;

  const double final_heading_distance =
    std::min(goal_rotate_dist_, active_position_tolerance);
  const bool final_heading_phase =
    distance_to_goal <= final_heading_distance &&
    std::abs(goal_yaw_error) > active_yaw_tolerance;

  if (final_heading_phase) {
    rotate_mode_ = true;
    const double yaw_error_margin =
      std::max(0.0, std::abs(goal_yaw_error) - active_yaw_tolerance);
    const double braking_limit = std::sqrt(
      2.0 * max_angular_decel_ * yaw_error_margin);
    const double final_angular_limit = std::min(rotate_angular_vel_, braking_limit);
    angular = std::copysign(
      std::min(rotate_gain_ * std::abs(goal_yaw_error), final_angular_limit),
      goal_yaw_error);
  } else {
    if (rotate_mode_ && std::abs(angle_to_carrot) <= rotate_exit_angle_) {
      rotate_mode_ = false;
    }
    if (!rotate_mode_ &&
      ((std::abs(velocity.linear.x) < 0.03 &&
      std::abs(angle_to_carrot) >= rotate_enter_angle_) ||
      std::abs(angle_to_carrot) >= hard_rotate_angle_))
    {
      rotate_mode_ = true;
    }

    if (rotate_mode_) {
      angular = clamp(
        rotate_gain_ * angle_to_carrot, -rotate_angular_vel_, rotate_angular_vel_);
    } else {
      const double heading_scale = clamp(std::cos(std::abs(angle_to_carrot)), 0.25, 1.0);
      const double curvature_scale = clamp(
        1.0 / (1.0 + curvature_speed_gain_ * std::abs(curvature)), 0.30, 1.0);
      const double remaining = std::max(distance_to_goal, target.remaining_path);
      const double approach_ratio = clamp(
        (remaining - active_position_tolerance) /
        std::max(approach_dist_ - active_position_tolerance, 1.0e-3), 0.0, 1.0);
      const double approach_speed = min_linear_vel_ +
        (desired_linear_vel_ - min_linear_vel_) * approach_ratio;

      linear = std::min(desired_linear_vel_, approach_speed) * heading_scale * curvature_scale;
      angular = linear * curvature + path_heading_gain_ * path_heading_error;
    }
  }

  unsigned char pose_cost = nav2_costmap_2d::NO_INFORMATION;
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(
      *(costmap_->getMutex()));
    pose_cost = costAtPose(pose.pose.position.x, pose.pose.position.y);
    if (linear > 0.0) {
      linear *= getParameterCostScale(pose_cost);
      linear = std::max(linear, min_linear_vel_);
    }

    angular = clamp(angular, -max_angular_vel_, max_angular_vel_);
    // The downstream velocity_smoother already limits angular acceleration and
    // has a faster deceleration limit. A second limiter here caused the command
    // sign to lag after crossing the target heading. Keep it optional for bench
    // experiments, but disable it in the shipped profile.
    if (max_angular_accel_ > 0.0) {
      const double angular_delta = max_angular_accel_ * control_period_;
      angular = clamp(
        angular, velocity.angular.z - angular_delta, velocity.angular.z + angular_delta);
    }

    if (use_collision_detection_ &&
      collisionAhead(pose, linear, angular, carrot_distance))
    {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "LuckController projected footprint collision; commanding zero for replanning");
      linear = 0.0;
      angular = 0.0;
    }
  }

  command.twist.linear.x = linear;
  command.twist.angular.z = angular;
  publishDebug(
    target, distance_to_goal, angle_to_carrot, curvature,
    linear, angular, pose_cost, goal_yaw_error, final_heading_phase,
    active_position_tolerance);
  return command;
}

void LuckController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    desired_linear_vel_ = base_desired_linear_vel_;
  } else if (percentage) {
    desired_linear_vel_ = base_desired_linear_vel_ * speed_limit / 100.0;
  } else {
    desired_linear_vel_ = std::min(base_desired_linear_vel_, speed_limit);
  }
}

nav_msgs::msg::Path LuckController::transformPlan(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  nav_msgs::msg::Path transformed;
  transformed.header.frame_id = robot_pose.header.frame_id;
  transformed.header.stamp = robot_pose.header.stamp;

  if (global_plan_.header.frame_id.empty()) {
    throw nav2_core::PlannerException("LuckController path frame is empty");
  }

  if (global_plan_.header.frame_id == robot_pose.header.frame_id) {
    transformed.poses = global_plan_.poses;
    for (auto & path_pose : transformed.poses) {
      path_pose.header = transformed.header;
    }
    return transformed;
  }

  try {
    const auto transform = tf_->lookupTransform(
      robot_pose.header.frame_id, global_plan_.header.frame_id,
      tf2::TimePointZero, tf2::durationFromSec(transform_tolerance_));
    transformed.poses.reserve(global_plan_.poses.size());
    for (const auto & path_pose : global_plan_.poses) {
      geometry_msgs::msg::PoseStamped output;
      tf2::doTransform(path_pose, output, transform);
      output.header = transformed.header;
      transformed.poses.push_back(output);
    }
  } catch (const tf2::TransformException & error) {
    throw nav2_core::PlannerException(
            std::string("LuckController path transform failed: ") + error.what());
  }
  return transformed;
}

LuckController::TrackingTarget LuckController::selectTrackingTarget(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double lookahead) const
{
  std::size_t nearest = 0;
  double nearest_sq = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x - robot_pose.pose.position.x;
    const double dy = path.poses[i].pose.position.y - robot_pose.pose.position.y;
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq < nearest_sq) {
      nearest_sq = distance_sq;
      nearest = i;
    }
  }

  TrackingTarget result;
  result.pose = path.poses[nearest];
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double nearest_dx = result.pose.pose.position.x - robot_pose.pose.position.x;
  const double nearest_dy = result.pose.pose.position.y - robot_pose.pose.position.y;
  result.cross_track_error =
    -std::sin(robot_yaw) * nearest_dx + std::cos(robot_yaw) * nearest_dy;

  double accumulated = 0.0;
  std::size_t selected = nearest;
  for (std::size_t i = nearest; i + 1 < path.poses.size(); ++i) {
    const auto & a = path.poses[i].pose.position;
    const auto & b = path.poses[i + 1].pose.position;
    const double segment = std::hypot(b.x - a.x, b.y - a.y);
    if (accumulated + segment >= lookahead && segment > 1.0e-6) {
      const double ratio = (lookahead - accumulated) / segment;
      result.pose = path.poses[i];
      result.pose.pose.position.x = a.x + ratio * (b.x - a.x);
      result.pose.pose.position.y = a.y + ratio * (b.y - a.y);
      selected = i;
      break;
    }
    accumulated += segment;
    selected = i + 1;
    result.pose = path.poses[selected];
  }

  std::size_t tangent_start = selected;
  std::size_t tangent_end = std::min(selected + 1, path.poses.size() - 1);
  if (tangent_start == tangent_end && tangent_start > 0) {
    --tangent_start;
  }
  const auto & tangent_a = path.poses[tangent_start].pose.position;
  const auto & tangent_b = path.poses[tangent_end].pose.position;
  if (std::hypot(tangent_b.x - tangent_a.x, tangent_b.y - tangent_a.y) > 1.0e-6) {
    result.tangent_yaw = std::atan2(tangent_b.y - tangent_a.y, tangent_b.x - tangent_a.x);
  } else {
    result.tangent_yaw = tf2::getYaw(path.poses.back().pose.orientation);
  }

  result.remaining_path = 0.0;
  for (std::size_t i = nearest; i + 1 < path.poses.size(); ++i) {
    const auto & a = path.poses[i].pose.position;
    const auto & b = path.poses[i + 1].pose.position;
    result.remaining_path += std::hypot(b.x - a.x, b.y - a.y);
  }
  return result;
}

bool LuckController::collisionAhead(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  double linear_velocity,
  double angular_velocity,
  double max_distance)
{
  if (footprintInCollision(
      robot_pose.pose.position.x, robot_pose.pose.position.y,
      tf2::getYaw(robot_pose.pose.orientation)))
  {
    return true;
  }
  if (std::abs(linear_velocity) < 1.0e-4 && std::abs(angular_velocity) < 1.0e-4) {
    return false;
  }

  double x = robot_pose.pose.position.x;
  double y = robot_pose.pose.position.y;
  double yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double dt = std::min(0.10, control_period_);
  double elapsed = 0.0;
  double travelled = 0.0;
  while (elapsed < collision_horizon_) {
    x += linear_velocity * std::cos(yaw) * dt;
    y += linear_velocity * std::sin(yaw) * dt;
    yaw = normalizeAngle(yaw + angular_velocity * dt);
    travelled += std::abs(linear_velocity) * dt;
    elapsed += dt;
    if (max_distance > 0.0 && travelled > max_distance) {
      break;
    }
    if (footprintInCollision(x, y, yaw)) {
      return true;
    }
  }
  return false;
}

bool LuckController::footprintInCollision(double x, double y, double yaw) const
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    return true;
  }
  const double footprint_cost = collision_checker_->footprintCostAtPose(
    x, y, yaw, costmap_ros_->getRobotFootprint());
  return footprint_cost < 0.0 ||
         footprint_cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
}

unsigned char LuckController::costAtPose(double x, double y) const
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_->worldToMap(x, y, mx, my)) {
    return nav2_costmap_2d::NO_INFORMATION;
  }
  return costmap_->getCost(mx, my);
}

double LuckController::getParameterCostScale(unsigned char cost) const
{
  if (cost == nav2_costmap_2d::NO_INFORMATION ||
    cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    return min_cost_speed_scale_;
  }
  const double normalized = static_cast<double>(cost) /
    static_cast<double>(nav2_costmap_2d::MAX_NON_OBSTACLE);
  return clamp(1.0 - cost_speed_gain_ * normalized, min_cost_speed_scale_, 1.0);
}

void LuckController::publishDebug(
  const TrackingTarget & target,
  double distance_to_goal,
  double angle_to_carrot,
  double curvature,
  double linear,
  double angular,
  unsigned char pose_cost,
  double goal_yaw_error,
  bool final_heading_phase,
  double active_position_tolerance)
{
  geometry_msgs::msg::PointStamped lookahead;
  lookahead.header = target.pose.header;
  lookahead.point = target.pose.pose.position;
  lookahead_pub_->publish(lookahead);

  std_msgs::msg::Float64MultiArray debug;
  debug.data = {
    distance_to_goal,
    angle_to_carrot,
    target.cross_track_error,
    curvature,
    linear,
    angular,
    static_cast<double>(pose_cost),
    rotate_mode_ ? 1.0 : 0.0,
    goal_yaw_error,
    final_heading_phase ? 1.0 : 0.0,
    active_position_tolerance};
  tracking_pub_->publish(debug);
}

double LuckController::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double LuckController::clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

}  // namespace luckrobot_nav2_plugins

PLUGINLIB_EXPORT_CLASS(
  luckrobot_nav2_plugins::LuckController,
  nav2_core::Controller)
