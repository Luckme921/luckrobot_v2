#ifndef LUCKROBOT_NAV2_PLUGINS__LUCK_CONTROLLER_HPP_
#define LUCKROBOT_NAV2_PLUGINS__LUCK_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/buffer.h"

namespace luckrobot_nav2_plugins
{

class LuckController : public nav2_core::Controller
{
public:
  LuckController() = default;
  ~LuckController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  struct TrackingTarget
  {
    geometry_msgs::msg::PoseStamped pose;
    double tangent_yaw{0.0};
    double cross_track_error{0.0};
    double remaining_path{0.0};
  };

  struct TrajectoryEvaluation
  {
    bool valid{false};
    bool avoidance{false};
    double linear{0.0};
    double angular{0.0};
    double score{0.0};
    double max_cost{0.0};
    double max_center_cost{0.0};
    double initial_center_cost{0.0};
    double end_center_cost{0.0};
    int valid_candidate_count{0};
    nav_msgs::msg::Path path;
  };

  nav_msgs::msg::Path transformPlan(
    const geometry_msgs::msg::PoseStamped & robot_pose) const;
  TrackingTarget selectTrackingTarget(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double lookahead) const;
  TrajectoryEvaluation evaluateTrajectory(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const TrackingTarget & target,
    double linear_velocity,
    double angular_velocity,
    double max_distance,
    double nominal_linear,
    double nominal_angular) const;
  TrajectoryEvaluation selectSafeTrajectory(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const TrackingTarget & target,
    double nominal_linear,
    double nominal_angular,
    double max_distance);
  TrajectoryEvaluation selectSafeReverseTrajectory(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const TrackingTarget & target,
    double nominal_linear,
    double nominal_angular,
    double max_distance) const;
  bool footprintInCollision(double x, double y, double yaw) const;
  unsigned char costAtPose(double x, double y) const;
  double getParameterCostScale(unsigned char cost) const;
  void resetObstacleState();
  void publishLocalTrajectory(const nav_msgs::msg::Path & path);
  void publishDebug(
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
    double lookahead_bend);

  static double normalizeAngle(double angle);
  static double clamp(double value, double low, double high);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<
      nav2_costmap_2d::Costmap2D *>> collision_checker_;

  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("LuckController")};
  rclcpp::Clock::SharedPtr clock_;
  nav_msgs::msg::Path global_plan_;
  mutable std::mutex plan_mutex_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr
    lookahead_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    tracking_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
    local_trajectory_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    execution_feedback_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr
    execution_imu_sub_;
  std::mutex execution_feedback_mutex_;
  std::string execution_feedback_odom_topic_{"/wheel/odom"};
  std::string execution_feedback_imu_topic_{"/imu/data"};
  double execution_feedback_timeout_{0.50};
  double execution_feedback_linear_{0.0};
  double execution_feedback_angular_{0.0};
  double execution_feedback_received_seconds_{-1.0};
  double execution_imu_angular_{0.0};
  double execution_imu_received_seconds_{-1.0};

  double desired_linear_vel_{0.15};
  double base_desired_linear_vel_{0.15};
  double min_linear_vel_{0.03};
  double max_angular_vel_{0.35};
  double rotate_angular_vel_{0.25};
  double min_rotate_angular_vel_{0.08};
  double lookahead_dist_{0.35};
  // A long fixed lookahead cuts across the inside of short planner corners.
  // Blend down to this near-field target only when the near and far path
  // bearings diverge; straight-path behavior remains unchanged.
  double corner_lookahead_dist_{0.12};
  double corner_lookahead_start_angle_{0.20};
  double corner_lookahead_full_angle_{0.45};
  double approach_dist_{0.35};
  double goal_rotate_dist_{0.08};
  double goal_position_tolerance_{0.08};
  double goal_yaw_tolerance_{0.10};
  double rotate_enter_angle_{0.45};
  double rotate_exit_angle_{0.15};
  double hard_rotate_angle_{0.80};
  double rotate_gain_{1.0};
  // Conservative physical deceleration used to start slowing an in-place turn
  // before rotate_exit_angle.  This is intentionally separate from the command
  // rate limiter because velocity_smoother and the chassis both add lag.
  double rotate_braking_decel_{0.20};
  double path_heading_gain_{0.60};
  double curvature_speed_gain_{1.0};
  double cost_speed_gain_{0.55};
  double min_cost_speed_scale_{0.40};
  // Zero disables the controller-local limiter. velocity_smoother remains the
  // single acceleration/deceleration authority in the deployed pipeline.
  double max_angular_accel_{0.0};
  double max_angular_decel_{0.80};
  double transform_tolerance_{1.0};
  double collision_horizon_{1.0};
  double avoidance_angular_span_{0.35};
  double avoidance_min_linear_scale_{0.35};
  double avoidance_path_weight_{2.5};
  double avoidance_heading_weight_{0.60};
  double avoidance_cost_weight_{0.80};
  double avoidance_smoothness_weight_{0.35};
  double avoidance_speed_weight_{0.50};
  double avoidance_clearance_weight_{2.0};
  double avoidance_switch_penalty_{0.40};
  double avoidance_side_hold_time_{2.0};
  // Last-resort escape after both positive-v arcs and swept in-place turns are
  // invalid. The reverse command is footprint checked, must lower center cost,
  // and has a strict per-navigation-goal time budget.
  double reverse_escape_linear_vel_{0.04};
  double reverse_escape_angular_vel_{0.12};
  double reverse_escape_timeout_{2.0};
  double reverse_escape_min_cost_drop_{2.0};
  double obstacle_wait_timeout_{1.5};
  double obstacle_failure_timeout_{5.0};
  // Independent command-to-odometry watchdog.  Nav2's PoseProgressChecker can
  // be restarted by FollowPath action retries, so it is not sufficient to
  // detect a healthy controller that keeps commanding a stalled actuator.
  double execution_stall_timeout_{3.0};
  // In-place heading alignment can legitimately take longer than a translating
  // command while filtered odometry catches up at startup. Keep a separate,
  // still bounded window so a healthy initial turn is not aborted prematurely.
  double execution_rotation_stall_timeout_{6.0};
  // A low-RPM differential turn can move the motor encoders without rotating
  // the chassis. If the IMU confirms that state, briefly request the already
  // configured maximum safe yaw rate to overcome static friction.
  double rotation_breakaway_delay_{0.60};
  double rotation_breakaway_angular_vel_{0.35};
  double motion_command_linear_threshold_{0.03};
  double motion_command_angular_threshold_{0.10};
  double motion_feedback_linear_threshold_{0.01};
  double motion_feedback_angular_threshold_{0.03};
  double control_period_{0.10};
  bool use_collision_detection_{true};
  bool use_local_avoidance_{true};
  int avoidance_angular_samples_{7};
  // Keep the robot center out of Nav2's 253 inscribed band so a subsequent
  // global replan can still start. Footprint collision remains >= 254.
  int max_trajectory_center_cost_{252};
  bool rotate_mode_{false};
  bool had_safe_candidate_last_cycle_{false};
  // Used only to distinguish a 1 Hz in-action path refresh from a new
  // FollowPath attempt after the behavior tree's Wait node.
  double last_compute_seconds_{-1.0};
  // Unlike obstacle state this timer is deliberately not reset by ordinary
  // same-goal path changes.  Otherwise 1 Hz replanning could hide a permanent
  // downstream stop forever.
  double execution_stall_since_seconds_{-1.0};
  double rotation_still_since_seconds_{-1.0};
  double reverse_escape_used_seconds_{0.0};
  double blocked_since_seconds_{-1.0};
  double avoidance_side_until_seconds_{-1.0};
  double last_selected_angular_{0.0};
  int avoidance_side_{0};
};

}  // namespace luckrobot_nav2_plugins

#endif  // LUCKROBOT_NAV2_PLUGINS__LUCK_CONTROLLER_HPP_
