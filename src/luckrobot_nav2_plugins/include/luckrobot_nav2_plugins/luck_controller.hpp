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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
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

  nav_msgs::msg::Path transformPlan(
    const geometry_msgs::msg::PoseStamped & robot_pose) const;
  TrackingTarget selectTrackingTarget(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double lookahead) const;
  bool collisionAhead(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    double linear_velocity,
    double angular_velocity,
    double max_distance);
  bool footprintInCollision(double x, double y, double yaw) const;
  unsigned char costAtPose(double x, double y) const;
  double getParameterCostScale(unsigned char cost) const;
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
    double active_position_tolerance);

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

  double desired_linear_vel_{0.15};
  double base_desired_linear_vel_{0.15};
  double min_linear_vel_{0.03};
  double max_angular_vel_{0.35};
  double rotate_angular_vel_{0.25};
  double lookahead_dist_{0.35};
  double approach_dist_{0.35};
  double goal_rotate_dist_{0.08};
  double goal_position_tolerance_{0.08};
  double goal_yaw_tolerance_{0.10};
  double rotate_enter_angle_{0.45};
  double rotate_exit_angle_{0.15};
  double hard_rotate_angle_{0.80};
  double rotate_gain_{1.0};
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
  double control_period_{0.10};
  bool use_collision_detection_{true};
  bool rotate_mode_{false};
};

}  // namespace luckrobot_nav2_plugins

#endif  // LUCKROBOT_NAV2_PLUGINS__LUCK_CONTROLLER_HPP_
