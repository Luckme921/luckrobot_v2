#include "luckrobot_nav2_plugins/luck_controller.hpp"
#include <nav2_costmap_2d/cost_values.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace luckrobot_nav2_plugins
{
struct LuckControllerTestAccess
{
  static void run(const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & ros)
  {
    auto require = [](bool ok, const char * msg) {
        if (!ok) {throw std::runtime_error(msg);}
      };
    LuckController c;
    c.costmap_ros_ = ros;
    c.costmap_ = ros->getCostmap();
    c.costmap_->resizeMap(200, 200, .01, -1.0, -1.0);
    c.costmap_->resetMap(0, 0, 200, 200);
    // Explicitly free synthetic map, irrespective of track_unknown_space default.
    for (unsigned int y = 0; y < 200; ++y) {
      for (unsigned int x = 0; x < 200; ++x) {c.costmap_->setCost(x, y, 0);}
    }
    std::vector<geometry_msgs::msg::Point> footprint(16);
    for (int i = 0; i < 16; ++i) {
      footprint[i].x = .175 * std::cos(i * 2.0 * M_PI / 16);
      footprint[i].y = .175 * std::sin(i * 2.0 * M_PI / 16);
    }
    ros->setRobotFootprint(footprint);
    c.collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<
      nav2_costmap_2d::Costmap2D *>>(c.costmap_);
    c.collision_horizon_ = 1.5;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.pose.orientation.w = 1.0;
    nav_msgs::msg::Path sparse;
    sparse.poses.assign(3, pose);
    sparse.poses[1].pose.position.x = 1.0;
    sparse.poses[2].pose.position.x = 2.0;
    pose.pose.position.x = .49;
    const auto before = c.selectTrackingTarget(sparse, pose, .35);
    pose.pose.position.x = .51;
    const auto after = c.selectTrackingTarget(sparse, pose, .35);
    require(std::abs(before.pose.pose.position.x - .84) < 1e-9,
      "carrot starts at segment projection");
    require(std::abs(after.pose.pose.position.x - before.pose.pose.position.x - .02) < 1e-9,
      "carrot continuous across nearest-vertex boundary");
    require(std::abs(after.remaining_path - 1.49) < 1e-9,
      "remaining distance excludes traversed segment");
    pose.pose.position.x = 0;
    LuckController::TrackingTarget target;
    target.pose = pose;
    target.pose.pose.position.x = .5;
    // A wall 7.5 cm ahead of the footprint. In-place rotation is geometrically
    // safe at rest; moving at .3 m/s it is NOT safe to assume instant stopping.
    for (unsigned int y = 0; y < 200; ++y) {
      c.costmap_->setCost(125, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
    require(c.evaluateTrajectory(pose, target, 0, .15, .35, 0, .15).valid,
      "stationary rotation should fit");
    c.measured_linear_ = .3;
    require(!c.evaluateTrajectory(pose, target, 0, .15, .35, 0, .15).valid,
      "moving rotation must include forward coast");
    require(!c.evaluateTrajectory(pose, target, -.04, 0, .35, -.04, 0).valid,
      "reverse must not erase forward coast");
    require(!c.evaluateTrajectory(pose, target, 0, 0, .35, 0, 0).valid,
      "stop prediction must include braking distance");
    c.measured_linear_ = 0;
    require(!c.evaluateTrajectory(pose, target, .3, 0, .35, .3, 0).valid,
      "slower execution must not mask ideal arc collision");
    for (unsigned int y = 0; y < 200; ++y) {c.costmap_->setCost(125, y, 0);}
    c.measured_linear_ = .15;
    c.measured_angular_ = -.25;
    require(c.evaluateTrajectory(pose, target, .15, .25, .35, .15, .25).valid,
      "safe reversal in open space");
    const auto begin = std::chrono::steady_clock::now();
    for (int i = 0; i < 100; ++i) {
      for (int j = -10; j <= 10; ++j) {
        c.evaluateTrajectory(pose, target, .15, j * .035, .35, .15, 0);
      }
    }
    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - begin).count() / 100;
    std::cout << "PASS actual controller: coast, reverse, stop, ideal arc and open-space checks; "
              << "21-candidate average " << ms << " ms (synthetic, not full-stack load)\n";

    auto tf = std::make_shared<tf2_ros::Buffer>(ros->get_clock());
    c.configure(ros, "TestController", tf, ros);
    c.activate();
    // Isolate command algebra here; collision acceptance was tested above.
    c.use_collision_detection_ = false;
    auto command = [&](double gx, double gy, double goal_yaw, double actual_yaw_rate,
      bool force_alignment = false) {
        nav_msgs::msg::Path plan;
        plan.header.frame_id = "odom";
        plan.poses = {pose, pose};
        plan.poses.back().pose.position.x = gx;
        plan.poses.back().pose.position.y = gy;
        plan.poses.back().pose.orientation.z = std::sin(goal_yaw / 2);
        plan.poses.back().pose.orientation.w = std::cos(goal_yaw / 2);
        c.setPlan(plan);
        if (force_alignment) {c.rotate_mode_ = true;}
        pose.header.stamp = ros->now();
        geometry_msgs::msg::Twist velocity;
        velocity.linear.x = .15;
        velocity.angular.z = actual_yaw_rate;
        return c.computeVelocityCommands(pose, velocity, nullptr).twist;
      };
    auto stopped = command(.06, 0, 0, 0);
    require(stopped.linear.x == 0 && stopped.angular.z == 0,
      "goal tolerance must not restart creep");
    c.desired_linear_vel_ = .40;
    c.approach_dist_ = .75;
    auto terminal_request = command(.15, 0, 0, 0);
    const double old_terminal = c.min_linear_vel_ +
      (.40-c.min_linear_vel_)*(.15-.08)/(.75-.08);
    require(terminal_request.linear.x > old_terminal &&
      terminal_request.linear.x <= old_terminal+.020001,
      "terminal request boost is small and applied before safety bounds");
    auto outside_terminal = command(.30, 0, 0, 0);
    const double old_outside = c.min_linear_vel_ +
      (.40-c.min_linear_vel_)*(.30-.08)/(.75-.08);
    require(std::abs(outside_terminal.linear.x-old_outside)<1e-8,
      "terminal boost must not change the earlier approach segment");
    c.costmap_->setCost(100, 100, 240);
    auto crawl = command(.081, 0, 0, 0);
    require(crawl.linear.x > 0 && crawl.linear.x < c.min_linear_vel_,
      "cost and terminal limits must not be raised to minimum speed");
    c.costmap_->setCost(100, 100, 0);
    auto reference = command(.5, .02, 0, 0);
    auto corrected = command(.5, .02, 0, .2);
    require(std::abs(corrected.linear.x - reference.linear.x) < 1e-8,
      "small yaw correction preserves forward request");
    require(corrected.angular.z < reference.angular.z,
      "measured positive yaw unwinds steering early");
    auto settling = command(.5, .035, 0, .20, true);
    require(settling.linear.x == 0 && settling.angular.z == 0,
      "aligned heading must brake residual yaw before translating");
    auto aligned = command(.5, .035, 0, .05, true);
    require(aligned.linear.x > 0,
      "aligned and slow yaw releases forward tracking");
    auto still_aligning = command(.5, .07, 0, .05, true);
    require(still_aligning.linear.x == 0 && still_aligning.angular.z > 0,
      "eight degree residual must remain in alignment");
    // setPlan may reset the still timer; trigger breakaway via zero delay.
    c.rotation_breakaway_delay_ = 0;
    c.execution_imu_received_seconds_ = ros->now().seconds();
    c.execution_imu_angular_ = 0;
    auto finish_turn = command(.06, 0, .11, 0);
    require(finish_turn.angular.z > 0 && finish_turn.angular.z < .12,
      "breakaway cannot override terminal angular braking");
    c.deactivate();
    c.cleanup();
    // Exercise configure() invariants for the explicit fast candidate, without
    // starting a chassis, TF broadcaster or publishing any velocity command.
    ros->declare_parameter("FastTest.desired_linear_vel", .40);
    ros->declare_parameter("FastTest.max_angular_vel", .80);
    ros->declare_parameter("FastTest.rotate_angular_vel", 4.0/7.0);
    ros->declare_parameter("FastTest.rotation_breakaway_angular_vel", 4.0/7.0);
    ros->declare_parameter("FastTest.approach_dist", .75);
    LuckController fast;
    fast.configure(ros, "FastTest", tf, ros);
    require(fast.max_angular_vel_ == .80 && fast.desired_linear_vel_ == .40,
      "fast configuration must load without parameter invariant failures");
    fast.cleanup();
    std::cout << "PASS actual command generation: goal hold, no floor override, "
              << "same-v yaw correction and bounded breakaway\n";
  }
};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("prediction_test_costmap");
  ros->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
  ros->on_configure(rclcpp_lifecycle::State());
  luckrobot_nav2_plugins::LuckControllerTestAccess::run(ros);
  ros->on_cleanup(rclcpp_lifecycle::State());
  ros.reset();
  rclcpp::shutdown();
}
