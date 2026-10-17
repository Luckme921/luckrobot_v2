#include "luckrobot_nav2_plugins/luck_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
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

namespace
{

// These constants deliberately remain local to the implementation while the
// recovery behavior is being isolated. They do not alter the plugin's public
// parameter surface or diagnostic message layout.
constexpr double kFollowPathResumeGap = 0.50;
constexpr double kCornerUnwindAngularVelocity = 0.12;
constexpr double kCornerUnwindTimeout = 2.0;
constexpr double kRadiansToDegrees = 57.29577951308232;

}  // namespace

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
  auto declare_int = [&](const std::string & key, int value) {
      declare_parameter_if_not_declared(
        node, plugin_name_ + "." + key,
        rclcpp::ParameterValue(static_cast<std::int64_t>(value)));
    };

  declare_double("desired_linear_vel", desired_linear_vel_);
  declare_double("min_linear_vel", min_linear_vel_);
  declare_double("max_angular_vel", max_angular_vel_);
  declare_double("rotate_angular_vel", rotate_angular_vel_);
  declare_double("min_rotate_angular_vel", min_rotate_angular_vel_);
  declare_double("lookahead_dist", lookahead_dist_);
  declare_double("corner_lookahead_dist", corner_lookahead_dist_);
  declare_double("corner_lookahead_start_angle", corner_lookahead_start_angle_);
  declare_double("corner_lookahead_full_angle", corner_lookahead_full_angle_);
  declare_double("approach_dist", approach_dist_);
  declare_double("goal_rotate_dist", goal_rotate_dist_);
  declare_double("goal_position_tolerance", goal_position_tolerance_);
  declare_double("goal_yaw_tolerance", goal_yaw_tolerance_);
  declare_double("rotate_enter_angle", rotate_enter_angle_);
  declare_double("rotate_exit_angle", rotate_exit_angle_);
  declare_double("hard_rotate_angle", hard_rotate_angle_);
  declare_double("rotate_gain", rotate_gain_);
  declare_double("rotate_braking_decel", rotate_braking_decel_);
  declare_double("path_heading_gain", path_heading_gain_);
  declare_double("curvature_speed_gain", curvature_speed_gain_);
  declare_double("cost_speed_gain", cost_speed_gain_);
  declare_double("min_cost_speed_scale", min_cost_speed_scale_);
  declare_double("max_angular_accel", max_angular_accel_);
  declare_double("max_angular_decel", max_angular_decel_);
  declare_double("transform_tolerance", transform_tolerance_);
  declare_double("collision_horizon", collision_horizon_);
  declare_double("avoidance_angular_span", avoidance_angular_span_);
  declare_double("avoidance_min_linear_scale", avoidance_min_linear_scale_);
  declare_double("avoidance_path_weight", avoidance_path_weight_);
  declare_double("avoidance_heading_weight", avoidance_heading_weight_);
  declare_double("avoidance_cost_weight", avoidance_cost_weight_);
  declare_double("avoidance_smoothness_weight", avoidance_smoothness_weight_);
  declare_double("avoidance_speed_weight", avoidance_speed_weight_);
  declare_double("avoidance_clearance_weight", avoidance_clearance_weight_);
  declare_double("avoidance_switch_penalty", avoidance_switch_penalty_);
  declare_double("avoidance_side_hold_time", avoidance_side_hold_time_);
  declare_double("reverse_escape_linear_vel", reverse_escape_linear_vel_);
  declare_double("reverse_escape_angular_vel", reverse_escape_angular_vel_);
  declare_double("reverse_escape_timeout", reverse_escape_timeout_);
  declare_double("reverse_escape_min_cost_drop", reverse_escape_min_cost_drop_);
  declare_double("obstacle_wait_timeout", obstacle_wait_timeout_);
  declare_double("obstacle_failure_timeout", obstacle_failure_timeout_);
  declare_double("execution_stall_timeout", execution_stall_timeout_);
  declare_double(
    "execution_rotation_stall_timeout", execution_rotation_stall_timeout_);
  declare_double("rotation_breakaway_delay", rotation_breakaway_delay_);
  declare_double(
    "rotation_breakaway_angular_vel", rotation_breakaway_angular_vel_);
  declare_double("execution_feedback_timeout", execution_feedback_timeout_);
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".execution_feedback_odom_topic",
    rclcpp::ParameterValue(execution_feedback_odom_topic_));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".execution_feedback_imu_topic",
    rclcpp::ParameterValue(execution_feedback_imu_topic_));
  declare_double(
    "motion_command_linear_threshold", motion_command_linear_threshold_);
  declare_double(
    "motion_command_angular_threshold", motion_command_angular_threshold_);
  declare_double(
    "motion_feedback_linear_threshold", motion_feedback_linear_threshold_);
  declare_double(
    "motion_feedback_angular_threshold", motion_feedback_angular_threshold_);
  declare_int("avoidance_angular_samples", avoidance_angular_samples_);
  declare_int("max_trajectory_center_cost", max_trajectory_center_cost_);
  declare_bool("use_collision_detection", use_collision_detection_);
  declare_bool("use_local_avoidance", use_local_avoidance_);

  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".min_linear_vel", min_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".rotate_angular_vel", rotate_angular_vel_);
  node->get_parameter(plugin_name_ + ".min_rotate_angular_vel", min_rotate_angular_vel_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".corner_lookahead_dist", corner_lookahead_dist_);
  node->get_parameter(
    plugin_name_ + ".corner_lookahead_start_angle", corner_lookahead_start_angle_);
  node->get_parameter(
    plugin_name_ + ".corner_lookahead_full_angle", corner_lookahead_full_angle_);
  node->get_parameter(plugin_name_ + ".approach_dist", approach_dist_);
  node->get_parameter(plugin_name_ + ".goal_rotate_dist", goal_rotate_dist_);
  node->get_parameter(
    plugin_name_ + ".goal_position_tolerance", goal_position_tolerance_);
  node->get_parameter(plugin_name_ + ".goal_yaw_tolerance", goal_yaw_tolerance_);
  node->get_parameter(plugin_name_ + ".rotate_enter_angle", rotate_enter_angle_);
  node->get_parameter(plugin_name_ + ".rotate_exit_angle", rotate_exit_angle_);
  node->get_parameter(plugin_name_ + ".hard_rotate_angle", hard_rotate_angle_);
  node->get_parameter(plugin_name_ + ".rotate_gain", rotate_gain_);
  node->get_parameter(plugin_name_ + ".rotate_braking_decel", rotate_braking_decel_);
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
  node->get_parameter(plugin_name_ + ".avoidance_angular_span", avoidance_angular_span_);
  node->get_parameter(
    plugin_name_ + ".avoidance_min_linear_scale", avoidance_min_linear_scale_);
  node->get_parameter(plugin_name_ + ".avoidance_path_weight", avoidance_path_weight_);
  node->get_parameter(plugin_name_ + ".avoidance_heading_weight", avoidance_heading_weight_);
  node->get_parameter(plugin_name_ + ".avoidance_cost_weight", avoidance_cost_weight_);
  node->get_parameter(
    plugin_name_ + ".avoidance_smoothness_weight", avoidance_smoothness_weight_);
  node->get_parameter(plugin_name_ + ".avoidance_speed_weight", avoidance_speed_weight_);
  node->get_parameter(
    plugin_name_ + ".avoidance_clearance_weight", avoidance_clearance_weight_);
  node->get_parameter(plugin_name_ + ".avoidance_switch_penalty", avoidance_switch_penalty_);
  node->get_parameter(
    plugin_name_ + ".avoidance_side_hold_time", avoidance_side_hold_time_);
  node->get_parameter(
    plugin_name_ + ".reverse_escape_linear_vel", reverse_escape_linear_vel_);
  node->get_parameter(
    plugin_name_ + ".reverse_escape_angular_vel", reverse_escape_angular_vel_);
  node->get_parameter(plugin_name_ + ".reverse_escape_timeout", reverse_escape_timeout_);
  node->get_parameter(
    plugin_name_ + ".reverse_escape_min_cost_drop", reverse_escape_min_cost_drop_);
  node->get_parameter(plugin_name_ + ".obstacle_wait_timeout", obstacle_wait_timeout_);
  node->get_parameter(plugin_name_ + ".obstacle_failure_timeout", obstacle_failure_timeout_);
  node->get_parameter(plugin_name_ + ".execution_stall_timeout", execution_stall_timeout_);
  node->get_parameter(
    plugin_name_ + ".execution_rotation_stall_timeout",
    execution_rotation_stall_timeout_);
  node->get_parameter(
    plugin_name_ + ".rotation_breakaway_delay", rotation_breakaway_delay_);
  node->get_parameter(
    plugin_name_ + ".rotation_breakaway_angular_vel",
    rotation_breakaway_angular_vel_);
  node->get_parameter(
    plugin_name_ + ".execution_feedback_timeout", execution_feedback_timeout_);
  node->get_parameter(
    plugin_name_ + ".execution_feedback_odom_topic", execution_feedback_odom_topic_);
  node->get_parameter(
    plugin_name_ + ".execution_feedback_imu_topic", execution_feedback_imu_topic_);
  node->get_parameter(
    plugin_name_ + ".motion_command_linear_threshold",
    motion_command_linear_threshold_);
  node->get_parameter(
    plugin_name_ + ".motion_command_angular_threshold",
    motion_command_angular_threshold_);
  node->get_parameter(
    plugin_name_ + ".motion_feedback_linear_threshold",
    motion_feedback_linear_threshold_);
  node->get_parameter(
    plugin_name_ + ".motion_feedback_angular_threshold",
    motion_feedback_angular_threshold_);
  std::int64_t avoidance_angular_samples = avoidance_angular_samples_;
  node->get_parameter(plugin_name_ + ".avoidance_angular_samples", avoidance_angular_samples);
  avoidance_angular_samples_ = static_cast<int>(avoidance_angular_samples);
  std::int64_t max_trajectory_center_cost = max_trajectory_center_cost_;
  node->get_parameter(
    plugin_name_ + ".max_trajectory_center_cost", max_trajectory_center_cost);
  max_trajectory_center_cost_ = static_cast<int>(max_trajectory_center_cost);
  node->get_parameter(
    plugin_name_ + ".use_collision_detection", use_collision_detection_);
  node->get_parameter(plugin_name_ + ".use_local_avoidance", use_local_avoidance_);

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
  if (lookahead_dist_ <= 0.0 || corner_lookahead_dist_ <= 0.0 ||
    corner_lookahead_dist_ > lookahead_dist_ ||
    corner_lookahead_start_angle_ < 0.0 ||
    corner_lookahead_full_angle_ <= corner_lookahead_start_angle_ ||
    desired_linear_vel_ <= 0.0 ||
    max_angular_vel_ <= 0.0 || min_rotate_angular_vel_ <= 0.0 ||
    min_rotate_angular_vel_ > rotate_angular_vel_ || rotate_braking_decel_ <= 0.0 ||
    max_angular_accel_ < 0.0 || max_angular_decel_ <= 0.0 ||
    collision_horizon_ <= 0.0 || avoidance_angular_span_ <= 0.0 ||
    avoidance_min_linear_scale_ <= 0.0 || avoidance_min_linear_scale_ > 1.0 ||
    avoidance_path_weight_ < 0.0 || avoidance_heading_weight_ < 0.0 ||
    avoidance_cost_weight_ < 0.0 || avoidance_smoothness_weight_ < 0.0 ||
    avoidance_speed_weight_ < 0.0 || avoidance_clearance_weight_ < 0.0 ||
    avoidance_switch_penalty_ < 0.0 ||
    avoidance_side_hold_time_ < 0.0 ||
    reverse_escape_linear_vel_ <= 0.0 || reverse_escape_angular_vel_ < 0.0 ||
    reverse_escape_timeout_ < 0.0 || reverse_escape_min_cost_drop_ < 0.0 ||
    avoidance_angular_samples_ < 3 || avoidance_angular_samples_ > 15 ||
    max_trajectory_center_cost_ < 0 ||
    max_trajectory_center_cost_ > static_cast<int>(nav2_costmap_2d::MAX_NON_OBSTACLE) ||
    obstacle_wait_timeout_ < 0.0 || obstacle_failure_timeout_ <= obstacle_wait_timeout_ ||
    execution_stall_timeout_ <= 0.0 || execution_rotation_stall_timeout_ <= 0.0 ||
    rotation_breakaway_delay_ < 0.0 ||
    rotation_breakaway_delay_ >= execution_rotation_stall_timeout_ ||
    rotation_breakaway_angular_vel_ < rotate_angular_vel_ ||
    rotation_breakaway_angular_vel_ > max_angular_vel_ ||
    execution_feedback_timeout_ <= 0.0 ||
    motion_command_linear_threshold_ < 0.0 ||
    motion_command_angular_threshold_ < 0.0 ||
    motion_feedback_linear_threshold_ < 0.0 ||
    motion_feedback_angular_threshold_ < 0.0)
  {
    throw nav2_core::PlannerException("LuckController parameters are outside safe bounds");
  }
  if (avoidance_angular_samples_ % 2 == 0) {
    ++avoidance_angular_samples_;
    RCLCPP_WARN(
      logger_, "%s avoidance_angular_samples must be odd; using %d",
      plugin_name_.c_str(), avoidance_angular_samples_);
  }

  collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<
      nav2_costmap_2d::Costmap2D *>>(costmap_);
  collision_checker_->setCostmap(costmap_);

  lookahead_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "/luck_controller/lookahead", rclcpp::QoS(10));
  tracking_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/luck_controller/tracking_error", rclcpp::QoS(10));
  local_trajectory_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "/luck_controller/local_trajectory", rclcpp::QoS(10));
  execution_feedback_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    execution_feedback_odom_topic_, rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(execution_feedback_mutex_);
      execution_feedback_linear_ = msg->twist.twist.linear.x;
      execution_feedback_angular_ = msg->twist.twist.angular.z;
      execution_feedback_received_seconds_ = clock_->now().seconds();
    });
  execution_imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
    execution_feedback_imu_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(execution_feedback_mutex_);
      execution_imu_angular_ = msg->angular_velocity.z;
      execution_imu_received_seconds_ = clock_->now().seconds();
    });

  RCLCPP_INFO(
    logger_,
    "Configured %s: v=%.2f m/s, lookahead=%.2f m (corner %.2f m), "
    "rotate hysteresis %.2f/%.2f rad, "
    "turn braking=%.2f rad/s^2, "
    "local avoidance=%s (%d angular samples, %.1f s horizon, max center cost %d), "
    "execution stall timeout=%.1f s (rotation %.1f s, wheel %s, IMU %s), "
    "rotation breakaway=%.2f rad/s after %.2f s, "
    "safe reverse <= %.1f s",
    plugin_name_.c_str(), desired_linear_vel_, lookahead_dist_, corner_lookahead_dist_,
    rotate_enter_angle_, rotate_exit_angle_, rotate_braking_decel_,
    use_local_avoidance_ ? "on" : "off",
    avoidance_angular_samples_, collision_horizon_, max_trajectory_center_cost_,
    execution_stall_timeout_, execution_rotation_stall_timeout_,
    execution_feedback_odom_topic_.c_str(), execution_feedback_imu_topic_.c_str(),
    rotation_breakaway_angular_vel_, rotation_breakaway_delay_, reverse_escape_timeout_);
}

void LuckController::cleanup()
{
  std::lock_guard<std::mutex> lock(plan_mutex_);
  global_plan_.poses.clear();
  collision_checker_.reset();
  lookahead_pub_.reset();
  tracking_pub_.reset();
  local_trajectory_pub_.reset();
  execution_feedback_sub_.reset();
  execution_imu_sub_.reset();
  resetObstacleState();
  last_compute_seconds_ = -1.0;
  execution_stall_since_seconds_ = -1.0;
  rotation_still_since_seconds_ = -1.0;
  reverse_escape_used_seconds_ = 0.0;
  RCLCPP_INFO(logger_, "Cleaned up %s", plugin_name_.c_str());
}

void LuckController::activate()
{
  execution_stall_since_seconds_ = -1.0;
  rotation_still_since_seconds_ = -1.0;
  reverse_escape_used_seconds_ = 0.0;
  lookahead_pub_->on_activate();
  tracking_pub_->on_activate();
  local_trajectory_pub_->on_activate();
  RCLCPP_INFO(logger_, "Activated %s", plugin_name_.c_str());
}

void LuckController::deactivate()
{
  execution_stall_since_seconds_ = -1.0;
  rotation_still_since_seconds_ = -1.0;
  lookahead_pub_->on_deactivate();
  tracking_pub_->on_deactivate();
  local_trajectory_pub_->on_deactivate();
  RCLCPP_INFO(logger_, "Deactivated %s", plugin_name_.c_str());
}

void LuckController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    throw nav2_core::PlannerException("LuckController received an empty path");
  }
  std::lock_guard<std::mutex> lock(plan_mutex_);
  const double now_seconds = clock_->now().seconds();
  const bool resumed_after_action_gap = last_compute_seconds_ >= 0.0 &&
    now_seconds - last_compute_seconds_ > kFollowPathResumeGap;
  bool new_navigation_goal = global_plan_.poses.empty();
  bool materially_new_path = global_plan_.poses.empty();
  if (!new_navigation_goal) {
    const auto & old_goal = global_plan_.poses.back().pose.position;
    const auto & new_goal = path.poses.back().pose.position;
    new_navigation_goal = std::hypot(
      new_goal.x - old_goal.x, new_goal.y - old_goal.y) > 0.20;

    const auto point_ahead = [this](const nav_msgs::msg::Path & candidate_path) {
        geometry_msgs::msg::Point point;
        if (candidate_path.poses.empty()) {
          return point;
        }
        point = candidate_path.poses.back().pose.position;
        const auto & start = candidate_path.poses.front().pose.position;
        for (const auto & path_pose : candidate_path.poses) {
          const auto & position = path_pose.pose.position;
          if (std::hypot(position.x - start.x, position.y - start.y) >= lookahead_dist_) {
            return position;
          }
        }
        return point;
      };
    const auto old_ahead = point_ahead(global_plan_);
    const auto new_ahead = point_ahead(path);
    materially_new_path = std::hypot(
      new_ahead.x - old_ahead.x, new_ahead.y - old_ahead.y) > 0.12;
  }
  global_plan_ = path;
  // Nav2 may replace the path every planning cycle. Preserve the rotate/drive
  // hysteresis for same-goal replans so the robot does not alternate between
  // turning and driving. A materially different goal starts a fresh state.
  if (new_navigation_goal) {
    rotate_mode_ = false;
    resetObstacleState();
    execution_stall_since_seconds_ = -1.0;
    rotation_still_since_seconds_ = -1.0;
    reverse_escape_used_seconds_ = 0.0;
  } else if (resumed_after_action_gap) {
    // The behavior tree waited before starting a fresh FollowPath action.
    // Give that action a new bounded local attempt. Ordinary 1 Hz path updates
    // arrive while computeVelocityCommands runs at 10 Hz and do not enter here.
    resetObstacleState();
    execution_stall_since_seconds_ = -1.0;
    rotation_still_since_seconds_ = -1.0;
  } else if (materially_new_path) {
    // A genuinely different near-field route gets a fresh opportunity to move.
    // Do not reset this timer for the identical 1 Hz path refresh, otherwise a
    // fixed obstruction could keep the controller waiting forever.
    resetObstacleState();
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
  last_compute_seconds_ = clock_->now().seconds();

  const auto transformed_plan = transformPlan(pose);
  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("LuckController could not transform the path");
  }

  const double robot_yaw = tf2::getYaw(pose.pose.orientation);
  const auto far_target = selectTrackingTarget(transformed_plan, pose, lookahead_dist_);
  const auto corner_target =
    selectTrackingTarget(transformed_plan, pose, corner_lookahead_dist_);
  const auto target_bearing_from_robot = [&](const TrackingTarget & candidate) {
      return std::atan2(
        candidate.pose.pose.position.y - pose.pose.position.y,
        candidate.pose.pose.position.x - pose.pose.position.x);
    };
  // A fixed 0.35 m carrot may lie after a close 90-degree turn, making a
  // correct planner path appear almost straight to Pure Pursuit. Measure how
  // much the near and far path bearings disagree and continuously shorten the
  // lookahead. This preserves the proven straight-line tuning while forcing
  // the controller to follow the first segment through tight corners.
  const double lookahead_bend = std::abs(normalizeAngle(
      target_bearing_from_robot(corner_target) -
      target_bearing_from_robot(far_target)));
  const double corner_ratio = clamp(
    (lookahead_bend - corner_lookahead_start_angle_) /
    std::max(
      corner_lookahead_full_angle_ - corner_lookahead_start_angle_, 1.0e-6),
    0.0, 1.0);
  const double effective_lookahead =
    lookahead_dist_ - corner_ratio * (lookahead_dist_ - corner_lookahead_dist_);
  const auto target = corner_ratio > 1.0e-6 ?
    selectTrackingTarget(transformed_plan, pose, effective_lookahead) : far_target;
  if (corner_ratio > 0.05) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000,
      "LuckController path corner: adaptive lookahead %.2f -> %.2f m "
      "(near/far bearing difference %.1f deg)",
      lookahead_dist_, effective_lookahead, lookahead_bend * kRadiansToDegrees);
  }

  const auto & goal = transformed_plan.poses.back();
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
  // Adaptive tracking may shorten the carrot to 0.12 m, but collision
  // prediction must retain the original far-field safety distance.
  const double collision_check_distance = std::min(distance_to_goal, lookahead_dist_);
  const double angle_to_carrot = std::atan2(target_y, target_x);
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
      // The old controller held rotate_angular_vel_ until very close to the
      // 8.6 degree exit boundary.  velocity_smoother plus the chassis' own
      // ramp then kept physical yaw velocity after the command changed, which
      // produced the measured left/right corrections after a 180 degree turn.
      // Treat the exit boundary as the end of the in-place phase and apply a
      // conservative stopping-speed envelope before it.  Small residual error
      // is still corrected continuously while driving; this does not add a
      // second acceleration-rate limiter or a settle pause.
      const double rotate_error_margin =
        std::max(0.0, std::abs(angle_to_carrot) - rotate_exit_angle_);
      const double rotate_braking_limit = std::sqrt(
        2.0 * rotate_braking_decel_ * rotate_error_margin);
      const double rotate_angular_limit =
        std::min(
        rotate_angular_vel_,
        std::max(min_rotate_angular_vel_, rotate_braking_limit));
      angular = std::copysign(
        std::min(rotate_gain_ * std::abs(angle_to_carrot), rotate_angular_limit),
        angle_to_carrot);
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
      // Pure Pursuit already uses the geometric lookahead bearing through
      // curvature. The former future-segment tangent term could point to the
      // opposite side at a replanned corner and reverse the steering command.
      // Keep path_heading_gain in the parameter file for backward-compatible
      // diagnostics, but do not feed a future segment tangent into steering.
      angular = linear * curvature;
    }
  }

  // The motor controller can report low wheel RPM (and therefore make wheel
  // odometry advance) while static friction keeps the physical chassis still.
  // Use the IMU only as a short-term physical-motion witness: after a bounded
  // delay, raise an in-place command to the configured safe maximum until yaw
  // motion starts. This leaves normal translating control and already-moving
  // turns unchanged. The boosted command is applied before trajectory checking
  // so the complete swept footprint is still collision validated.
  bool rotation_breakaway_active = false;
  {
    const double now_seconds = clock_->now().seconds();
    const bool pure_rotation_requested =
      std::abs(linear) < 1.0e-4 && std::abs(angular) > 1.0e-4;
    std::lock_guard<std::mutex> lock(execution_feedback_mutex_);
    const bool imu_fresh = execution_imu_received_seconds_ >= 0.0 &&
      now_seconds - execution_imu_received_seconds_ <= execution_feedback_timeout_;
    const bool imu_reports_rotation = imu_fresh &&
      std::isfinite(execution_imu_angular_) &&
      std::abs(execution_imu_angular_) >= motion_feedback_angular_threshold_;
    if (pure_rotation_requested && imu_fresh && !imu_reports_rotation) {
      if (rotation_still_since_seconds_ < 0.0) {
        rotation_still_since_seconds_ = now_seconds;
      }
      const double still_duration =
        std::max(0.0, now_seconds - rotation_still_since_seconds_);
      if (still_duration >= rotation_breakaway_delay_) {
        angular = std::copysign(
          std::max(std::abs(angular), rotation_breakaway_angular_vel_), angular);
        rotation_breakaway_active = true;
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 1000,
          "LuckController IMU confirms no physical yaw for %.1f s; "
          "applying bounded rotation breakaway w=%.3f rad/s",
          still_duration, angular);
      }
    } else {
      rotation_still_since_seconds_ = -1.0;
    }
  }

  unsigned char pose_cost = nav2_costmap_2d::NO_INFORMATION;
  TrajectoryEvaluation selected_trajectory;
  int avoidance_state = 0;
  int valid_candidate_count = 0;
  double selected_trajectory_cost = 0.0;
  double selected_center_cost = 0.0;
  double blocked_duration = 0.0;
  bool obstacle_failure = false;
  bool corner_unwind = false;
  bool reverse_escape = false;
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

    const double nominal_linear = linear;
    const double nominal_angular = angular;
    if (use_collision_detection_) {
      selected_trajectory = evaluateTrajectory(
        pose, target, nominal_linear, nominal_angular, collision_check_distance,
        nominal_linear, nominal_angular);
      selected_trajectory_cost = selected_trajectory.max_cost;
      selected_center_cost = selected_trajectory.max_center_cost;
      if (selected_trajectory.valid) {
        blocked_since_seconds_ = -1.0;
        had_safe_candidate_last_cycle_ = false;
        last_selected_angular_ = angular;
      } else {
        if (use_local_avoidance_ && linear > 1.0e-4 && !final_heading_phase) {
          selected_trajectory = selectSafeTrajectory(
            pose, target, nominal_linear, nominal_angular, collision_check_distance);
          selected_trajectory_cost = selected_trajectory.max_cost;
          selected_center_cost = selected_trajectory.max_center_cost;
        }

        // If every positive-v arc is invalid at a sharp corner, evaluate a
        // low-speed in-place rotation toward the actual lookahead point. This
        // can change the geometry of the next forward arc without blind backup.
        // It is bounded below and is never considered translational progress.
        const double corner_unwind_elapsed = blocked_since_seconds_ < 0.0 ? 0.0 :
          std::max(0.0, clock_->now().seconds() - blocked_since_seconds_);
        if (!selected_trajectory.valid && use_local_avoidance_ &&
          !final_heading_phase && reverse_escape_used_seconds_ <= 1.0e-6 &&
          corner_unwind_elapsed < kCornerUnwindTimeout)
        {
          const double preferred_direction = angle_to_carrot >= 0.0 ? 1.0 : -1.0;
          for (int attempt = 0; attempt < 2; ++attempt) {
            const double direction = attempt == 0 ?
              preferred_direction : -preferred_direction;
            auto rotation_candidate = evaluateTrajectory(
              pose, target, 0.0,
              direction * kCornerUnwindAngularVelocity,
              collision_check_distance, nominal_linear, nominal_angular);
            if (rotation_candidate.valid) {
              selected_trajectory = std::move(rotation_candidate);
              selected_trajectory_cost = selected_trajectory.max_cost;
              selected_center_cost = selected_trajectory.max_center_cost;
              corner_unwind = true;
              break;
            }
          }
        }

        // If translating arcs and swept in-place turns are both impossible,
        // allow a very short, slow reverse only when the whole predicted
        // footprint is safe and the trajectory ends at a lower center cost.
        // This is a bounded geometric escape, not an unconditional BT backup.
        if (!selected_trajectory.valid && use_local_avoidance_ &&
          !final_heading_phase &&
          reverse_escape_used_seconds_ < reverse_escape_timeout_)
        {
          auto reverse_candidate = selectSafeReverseTrajectory(
            pose, target, nominal_linear, nominal_angular, collision_check_distance);
          if (reverse_candidate.valid) {
            selected_trajectory = std::move(reverse_candidate);
            selected_trajectory_cost = selected_trajectory.max_cost;
            selected_center_cost = selected_trajectory.max_center_cost;
            reverse_escape = true;
          }
        }

        if (selected_trajectory.valid) {
          linear = selected_trajectory.linear;
          angular = selected_trajectory.angular;
          avoidance_state = reverse_escape ? 7 : (corner_unwind ? 4 : 1);
          valid_candidate_count = selected_trajectory.valid_candidate_count;
          selected_trajectory_cost = selected_trajectory.max_cost;
          const double now_seconds = clock_->now().seconds();
          if (reverse_escape) {
            reverse_escape_used_seconds_ = std::min(
              reverse_escape_timeout_,
              reverse_escape_used_seconds_ + control_period_);
            blocked_since_seconds_ = -1.0;
            blocked_duration = reverse_escape_used_seconds_;
          } else if (corner_unwind) {
            if (blocked_since_seconds_ < 0.0) {
              blocked_since_seconds_ = now_seconds;
            }
            blocked_duration = std::max(0.0, now_seconds - blocked_since_seconds_);
          } else {
            const bool robot_is_moving =
              std::abs(velocity.linear.x) > 0.01 || std::abs(velocity.angular.z) > 0.03;
            if (robot_is_moving) {
              blocked_since_seconds_ = -1.0;
            } else {
              if (!had_safe_candidate_last_cycle_ || blocked_since_seconds_ < 0.0) {
                blocked_since_seconds_ = now_seconds;
              }
              blocked_duration = std::max(0.0, now_seconds - blocked_since_seconds_);
              if (blocked_duration >= obstacle_failure_timeout_) {
                avoidance_state = 3;
                obstacle_failure = true;
                linear = 0.0;
                angular = 0.0;
              }
            }
          }
          had_safe_candidate_last_cycle_ = true;
          if (reverse_escape) {
            RCLCPP_WARN_THROTTLE(
              logger_, *clock_, 1000,
              "LuckController safe reverse escape v=%.3f w=%.3f "
              "(center cost %.0f -> %.0f, %.1f/%.1f s budget)",
              linear, angular, selected_trajectory.initial_center_cost,
              selected_trajectory.end_center_cost,
              reverse_escape_used_seconds_, reverse_escape_timeout_);
          } else if (corner_unwind && !obstacle_failure) {
            RCLCPP_WARN_THROTTLE(
              logger_, *clock_, 1000,
              "LuckController all forward arcs blocked; bounded corner unwind "
              "w=%.3f (%.1f/%.1f s)",
              angular, blocked_duration, kCornerUnwindTimeout);
          } else if (obstacle_failure) {
            RCLCPP_WARN_THROTTLE(
              logger_, *clock_, 1000,
              "LuckController has safe candidates but measured no robot motion for %.1f s; "
              "requesting a calm replan",
              blocked_duration);
          } else {
            RCLCPP_INFO_THROTTLE(
              logger_, *clock_, 1000,
              "LuckController local avoidance selected v=%.3f w=%.3f "
              "(nominal v=%.3f w=%.3f, %d safe candidates, "
              "max footprint/center cost %.0f/%.0f)",
              linear, angular, nominal_linear, nominal_angular,
              valid_candidate_count, selected_trajectory_cost, selected_center_cost);
          }
        } else {
          had_safe_candidate_last_cycle_ = false;
          const double now_seconds = clock_->now().seconds();
          if (blocked_since_seconds_ < 0.0) {
            blocked_since_seconds_ = now_seconds;
          }
          blocked_duration = std::max(0.0, now_seconds - blocked_since_seconds_);
          avoidance_state = blocked_duration >= obstacle_failure_timeout_ ? 3 : 2;
          obstacle_failure = avoidance_state == 3;
          linear = 0.0;
          angular = 0.0;
          if (blocked_duration < obstacle_wait_timeout_) {
            RCLCPP_WARN_THROTTLE(
              logger_, *clock_, 1000,
              "LuckController path blocked; waiting briefly for a transient obstacle (%.1f s)",
              blocked_duration);
          } else {
            RCLCPP_WARN_THROTTLE(
              logger_, *clock_, 1000,
              "LuckController path blocked; awaiting a new safe global path (%.1f s)",
              blocked_duration);
          }
        }
      }
    }
  }

  // Detect the failure that Nav2's path/progress logs cannot distinguish:
  // this plugin is producing a valid non-zero command, but the fused odometry
  // reports that the chassis is still.  This also catches a downstream safety
  // layer holding cmd_vel_safe at zero.  Same-goal replanning deliberately
  // does not reset the timer, so a 1 Hz stream of new paths cannot mask the
  // fault indefinitely. A fresh FollowPath attempt gets a clean bounded try.
  if (obstacle_failure) {
    execution_stall_since_seconds_ = -1.0;
  } else {
    const bool commanded_motion =
      std::abs(linear) >= motion_command_linear_threshold_ ||
      std::abs(angular) >= motion_command_angular_threshold_;
    const bool pure_rotation_command =
      std::abs(linear) < motion_command_linear_threshold_ &&
      std::abs(angular) >= motion_command_angular_threshold_;
    bool measured_motion = false;
    // Translation can only be witnessed by odometry on this platform. For a
    // pure in-place turn, however, wheel encoders are not physical proof: the
    // failing bag shows >600 deg of wheel-derived yaw while IMU yaw is exactly
    // zero. Prefer fresh IMU yaw and use filtered odometry only as a fallback
    // when the IMU stream itself is unavailable.
    {
      std::lock_guard<std::mutex> lock(execution_feedback_mutex_);
      const double now_seconds = clock_->now().seconds();
      const bool wheel_fresh = execution_feedback_received_seconds_ >= 0.0 &&
        now_seconds - execution_feedback_received_seconds_ <= execution_feedback_timeout_;
      const bool imu_fresh = execution_imu_received_seconds_ >= 0.0 &&
        now_seconds - execution_imu_received_seconds_ <= execution_feedback_timeout_;
      const bool filtered_linear_motion =
        std::abs(velocity.linear.x) >= motion_feedback_linear_threshold_;
      const bool filtered_angular_motion =
        std::abs(velocity.angular.z) >= motion_feedback_angular_threshold_;
      const bool wheel_linear_motion = wheel_fresh &&
        std::abs(execution_feedback_linear_) >= motion_feedback_linear_threshold_;
      const bool wheel_angular_motion = wheel_fresh &&
        std::abs(execution_feedback_angular_) >= motion_feedback_angular_threshold_;
      const bool imu_angular_motion = imu_fresh &&
        std::isfinite(execution_imu_angular_) &&
        std::abs(execution_imu_angular_) >= motion_feedback_angular_threshold_;
      if (pure_rotation_command) {
        measured_motion = imu_fresh ? imu_angular_motion : filtered_angular_motion;
      } else {
        measured_motion = filtered_linear_motion || wheel_linear_motion ||
          (imu_fresh ? imu_angular_motion : (filtered_angular_motion || wheel_angular_motion));
      }
    }
    if (commanded_motion && !measured_motion) {
      // Startup in-place turns can precede the first non-zero EKF angular
      // velocity. Give them a longer bounded window; translating or mixed
      // commands retain the tighter execution_stall_timeout_.
      const double stall_timeout =
        std::abs(linear) >= motion_command_linear_threshold_ ?
        execution_stall_timeout_ : execution_rotation_stall_timeout_;
      const double now_seconds = clock_->now().seconds();
      if (execution_stall_since_seconds_ < 0.0) {
        execution_stall_since_seconds_ = now_seconds;
      }
      const double execution_stall_duration =
        std::max(0.0, now_seconds - execution_stall_since_seconds_);
      blocked_duration = std::max(blocked_duration, execution_stall_duration);
      if (execution_stall_duration >= stall_timeout) {
        avoidance_state = 6;
        obstacle_failure = true;
        linear = 0.0;
        angular = 0.0;
        RCLCPP_ERROR_THROTTLE(
          logger_, *clock_, 1000,
          "LuckController commanded motion but odometry remained still for %.1f s "
          "(limit %.1f s); "
          "requesting a fresh FollowPath/replan (upstream command or actuator chain stalled)",
          execution_stall_duration, stall_timeout);
      } else if (execution_stall_duration >= 1.0) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 1000,
          "LuckController is commanding v=%.3f w=%.3f but odometry is still "
          "(%.1f/%.1f s)",
          linear, angular, execution_stall_duration, stall_timeout);
      }
    } else {
      execution_stall_since_seconds_ = -1.0;
    }
  }

  if (rotation_breakaway_active && avoidance_state == 0) {
    avoidance_state = 8;
  }

  command.twist.linear.x = linear;
  command.twist.angular.z = angular;
  publishLocalTrajectory(selected_trajectory.path);
  publishDebug(
    target, distance_to_goal, angle_to_carrot, curvature,
    linear, angular, pose_cost, goal_yaw_error, final_heading_phase,
    active_position_tolerance, avoidance_state, valid_candidate_count,
    selected_trajectory_cost, selected_center_cost, blocked_duration,
    effective_lookahead, lookahead_bend);
  if (obstacle_failure) {
    // ROS 2 Humble exposes PlannerException as the controller failure type.
    // Controller Server applies failure_tolerance before propagating it.
    if (avoidance_state == 6) {
      throw nav2_core::PlannerException(
              "LuckController command was not followed by odometry; requesting a calm replan");
    }
    throw nav2_core::PlannerException(
            "LuckController found no safe local trajectory; requesting a calm replan");
  }
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

LuckController::TrajectoryEvaluation LuckController::evaluateTrajectory(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const TrackingTarget & target,
  double linear_velocity,
  double angular_velocity,
  double max_distance,
  double nominal_linear,
  double nominal_angular) const
{
  TrajectoryEvaluation result;
  result.linear = linear_velocity;
  result.angular = angular_velocity;
  result.score = std::numeric_limits<double>::infinity();
  result.path.header = robot_pose.header;

  geometry_msgs::msg::PoseStamped predicted_pose = robot_pose;
  predicted_pose.header = result.path.header;
  result.path.poses.push_back(predicted_pose);

  double x = robot_pose.pose.position.x;
  double y = robot_pose.pose.position.y;
  double yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double initial_footprint_cost = collision_checker_->footprintCostAtPose(
    x, y, yaw, costmap_ros_->getRobotFootprint());
  result.max_cost = initial_footprint_cost;
  result.max_center_cost = static_cast<double>(costAtPose(x, y));
  result.initial_center_cost = result.max_center_cost;
  result.end_center_cost = result.max_center_cost;
  const double initial_center_cost = result.initial_center_cost;
  const bool escaping_inscribed_center =
    initial_center_cost > static_cast<double>(max_trajectory_center_cost_) &&
    initial_center_cost < static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE);
  if (initial_footprint_cost < 0.0 ||
    initial_footprint_cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE) ||
    result.max_center_cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE))
  {
    return result;
  }

  if (std::abs(linear_velocity) < 1.0e-4 && std::abs(angular_velocity) < 1.0e-4) {
    result.valid = true;
    result.valid_candidate_count = 1;
    result.score = 0.0;
    return result;
  }

  const double dt = std::min(0.10, control_period_);
  double elapsed = 0.0;
  double travelled = 0.0;
  while (elapsed < collision_horizon_) {
    const double step = std::min(dt, collision_horizon_ - elapsed);
    const double midpoint_yaw = yaw + 0.5 * angular_velocity * step;
    x += linear_velocity * std::cos(midpoint_yaw) * step;
    y += linear_velocity * std::sin(midpoint_yaw) * step;
    yaw = normalizeAngle(yaw + angular_velocity * step);
    travelled += std::abs(linear_velocity) * step;
    elapsed += step;

    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_->worldToMap(x, y, mx, my)) {
      return result;
    }
    const double footprint_cost = collision_checker_->footprintCostAtPose(
      x, y, yaw, costmap_ros_->getRobotFootprint());
    const double center_cost = static_cast<double>(costAtPose(x, y));
    result.max_cost = std::max(result.max_cost, footprint_cost);
    result.max_center_cost = std::max(result.max_center_cost, center_cost);
    result.end_center_cost = center_cost;
    if (footprint_cost < 0.0 ||
      footprint_cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE) ||
      (!escaping_inscribed_center &&
      center_cost > static_cast<double>(max_trajectory_center_cost_)) ||
      (escaping_inscribed_center &&
      center_cost > initial_center_cost))
    {
      return result;
    }

    predicted_pose.pose.position.x = x;
    predicted_pose.pose.position.y = y;
    predicted_pose.pose.orientation.x = 0.0;
    predicted_pose.pose.orientation.y = 0.0;
    predicted_pose.pose.orientation.z = std::sin(0.5 * yaw);
    predicted_pose.pose.orientation.w = std::cos(0.5 * yaw);
    result.path.poses.push_back(predicted_pose);

    if (max_distance > 0.0 && travelled > max_distance) {
      break;
    }
  }

  // LuckPlanner intentionally permits an already-inscribed start cell to
  // expand into an adjacent traversable cell. Mirror that behavior locally,
  // but accept only a trajectory that actually exits the 253 center band.
  // Pure rotation is still allowed when its swept footprint remains safe.
  if (escaping_inscribed_center && std::abs(linear_velocity) >= 1.0e-4 &&
    static_cast<int>(costAtPose(x, y)) > max_trajectory_center_cost_)
  {
    return result;
  }

  const double target_x = target.pose.pose.position.x;
  const double target_y = target.pose.pose.position.y;
  const double start_distance = std::hypot(
    target_x - robot_pose.pose.position.x, target_y - robot_pose.pose.position.y);
  const double end_distance = std::hypot(target_x - x, target_y - y);
  const double target_bearing = std::atan2(target_y - y, target_x - x);
  const double heading_error = std::abs(normalizeAngle(target_bearing - yaw));
  const double cost_ratio = clamp(
    result.max_center_cost / static_cast<double>(nav2_costmap_2d::MAX_NON_OBSTACLE),
    0.0, 1.0);
  const double clearance_delta = clamp(
    (result.end_center_cost - result.initial_center_cost) /
    static_cast<double>(nav2_costmap_2d::MAX_NON_OBSTACLE),
    -1.0, 1.0);
  const double speed_loss = std::max(0.0, nominal_linear - linear_velocity) /
    std::max(std::abs(nominal_linear), min_linear_vel_);
  const double angular_change = std::abs(angular_velocity - nominal_angular) /
    std::max(max_angular_vel_, 1.0e-3);
  const double angular_smoothness = std::abs(angular_velocity - last_selected_angular_) /
    std::max(max_angular_vel_, 1.0e-3);

  result.score =
    avoidance_path_weight_ * end_distance +
    avoidance_heading_weight_ * heading_error +
    avoidance_cost_weight_ * cost_ratio +
    avoidance_smoothness_weight_ * angular_smoothness +
    avoidance_speed_weight_ * speed_loss +
    avoidance_clearance_weight_ * clearance_delta +
    0.25 * angular_change;
  if (end_distance > start_distance + 0.03) {
    result.score += 5.0 * (end_distance - start_distance);
  }
  result.valid = true;
  result.valid_candidate_count = 1;
  return result;
}

LuckController::TrajectoryEvaluation LuckController::selectSafeTrajectory(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const TrackingTarget & target,
  double nominal_linear,
  double nominal_angular,
  double max_distance)
{
  TrajectoryEvaluation best;
  best.score = std::numeric_limits<double>::infinity();
  best.max_cost = collision_checker_->footprintCostAtPose(
    robot_pose.pose.position.x, robot_pose.pose.position.y,
    tf2::getYaw(robot_pose.pose.orientation), costmap_ros_->getRobotFootprint());
  best.max_center_cost = static_cast<double>(costAtPose(
      robot_pose.pose.position.x, robot_pose.pose.position.y));
  double rejected_max_cost = best.max_cost;
  double rejected_max_center_cost = best.max_center_cost;
  int valid_candidates = 0;
  const double now_seconds = clock_->now().seconds();
  if (avoidance_side_until_seconds_ >= 0.0 &&
    now_seconds >= avoidance_side_until_seconds_)
  {
    avoidance_side_ = 0;
    avoidance_side_until_seconds_ = -1.0;
  }

  constexpr int linear_samples = 3;
  const int angular_half = avoidance_angular_samples_ / 2;
  for (int linear_index = 0; linear_index < linear_samples; ++linear_index) {
    const double ratio = static_cast<double>(linear_index) /
      static_cast<double>(linear_samples - 1);
    const double linear_scale =
      1.0 - ratio * (1.0 - avoidance_min_linear_scale_);
    const double candidate_linear = std::max(
      min_linear_vel_, nominal_linear * linear_scale);

    for (int angular_index = -angular_half; angular_index <= angular_half; ++angular_index) {
      const double normalized_offset = angular_half > 0 ?
        static_cast<double>(angular_index) / static_cast<double>(angular_half) : 0.0;
      const double angular_offset = avoidance_angular_span_ * normalized_offset;
      const double candidate_angular = clamp(
        nominal_angular + angular_offset, -max_angular_vel_, max_angular_vel_);

      auto candidate = evaluateTrajectory(
        robot_pose, target, candidate_linear, candidate_angular,
        max_distance, nominal_linear, nominal_angular);
      if (!candidate.valid) {
        rejected_max_cost = std::max(rejected_max_cost, candidate.max_cost);
        rejected_max_center_cost = std::max(
          rejected_max_center_cost, candidate.max_center_cost);
        continue;
      }
      ++valid_candidates;

      const int candidate_side = angular_offset > 0.05 ? 1 :
        (angular_offset < -0.05 ? -1 : 0);
      if (avoidance_side_ != 0 && candidate_side != 0 &&
        candidate_side != avoidance_side_ &&
        candidate.end_center_cost >=
        candidate.initial_center_cost - reverse_escape_min_cost_drop_)
      {
        candidate.score += avoidance_switch_penalty_;
      }
      if (candidate.score < best.score) {
        best = std::move(candidate);
      }
    }
  }

  if (!best.valid) {
    best.max_cost = rejected_max_cost;
    best.max_center_cost = rejected_max_center_cost;
    return best;
  }
  best.avoidance = true;
  best.valid_candidate_count = valid_candidates;
  const double chosen_offset = best.angular - nominal_angular;
  if (std::abs(chosen_offset) > 0.05) {
    avoidance_side_ = chosen_offset > 0.0 ? 1 : -1;
    avoidance_side_until_seconds_ = now_seconds + avoidance_side_hold_time_;
  }
  last_selected_angular_ = best.angular;
  return best;
}

LuckController::TrajectoryEvaluation LuckController::selectSafeReverseTrajectory(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const TrackingTarget & target,
  double nominal_linear,
  double nominal_angular,
  double max_distance) const
{
  TrajectoryEvaluation best;
  best.score = std::numeric_limits<double>::infinity();
  double rejected_max_cost = collision_checker_->footprintCostAtPose(
    robot_pose.pose.position.x, robot_pose.pose.position.y,
    tf2::getYaw(robot_pose.pose.orientation), costmap_ros_->getRobotFootprint());
  double rejected_max_center_cost = static_cast<double>(costAtPose(
      robot_pose.pose.position.x, robot_pose.pose.position.y));
  int valid_candidates = 0;

  // Straight reverse is tested first, then two shallow arcs. Selection is still
  // score based, but every accepted candidate must end at a measurably lower
  // center cost than it started and is limited to at most 8 cm of prediction.
  const double angular_candidates[] = {
    0.0, reverse_escape_angular_vel_, -reverse_escape_angular_vel_};
  const double reverse_prediction_distance = std::min(max_distance, 0.08);
  for (const double candidate_angular : angular_candidates) {
    auto candidate = evaluateTrajectory(
      robot_pose, target, -reverse_escape_linear_vel_, candidate_angular,
      reverse_prediction_distance, nominal_linear, nominal_angular);
    if (!candidate.valid ||
      candidate.end_center_cost >
      candidate.initial_center_cost - reverse_escape_min_cost_drop_)
    {
      rejected_max_cost = std::max(rejected_max_cost, candidate.max_cost);
      rejected_max_center_cost = std::max(
        rejected_max_center_cost, candidate.max_center_cost);
      continue;
    }
    ++valid_candidates;
    if (candidate.score < best.score) {
      best = std::move(candidate);
    }
  }

  if (!best.valid) {
    best.max_cost = rejected_max_cost;
    best.max_center_cost = rejected_max_center_cost;
    return best;
  }
  best.avoidance = true;
  best.valid_candidate_count = valid_candidates;
  return best;
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

void LuckController::resetObstacleState()
{
  blocked_since_seconds_ = -1.0;
  had_safe_candidate_last_cycle_ = false;
  avoidance_side_until_seconds_ = -1.0;
  last_selected_angular_ = 0.0;
  avoidance_side_ = 0;
}

void LuckController::publishLocalTrajectory(const nav_msgs::msg::Path & path)
{
  if (local_trajectory_pub_ && !path.poses.empty()) {
    local_trajectory_pub_->publish(path);
  }
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
  double active_position_tolerance,
  int avoidance_state,
  int valid_candidate_count,
  double selected_trajectory_cost,
  double selected_center_cost,
  double blocked_duration,
  double effective_lookahead,
  double lookahead_bend)
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
    active_position_tolerance,
    static_cast<double>(avoidance_state),
    static_cast<double>(valid_candidate_count),
    selected_trajectory_cost,
    selected_center_cost,
    blocked_duration,
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    effective_lookahead,
    lookahead_bend};
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
