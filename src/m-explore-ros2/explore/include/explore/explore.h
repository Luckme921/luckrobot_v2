#include "explore/coverage_policy.hpp"
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include "explore/observation_progress.hpp"
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  Copyright (c) 2021, Carlos Alvarez, Juan Galvis.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#ifndef NAV_EXPLORE_H_
#define NAV_EXPLORE_H_

#include <explore/costmap_client.h>
#include <explore/frontier_search.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <explore_lite_msgs/msg/explore_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "explore/global_regions.hpp"

using namespace std::placeholders;
#ifdef ELOQUENT
#define ACTION_NAME "NavigateToPose"
#elif DASHING
#define ACTION_NAME "NavigateToPose"
#else
#define ACTION_NAME "navigate_to_pose"
#endif
namespace explore
{
/**
 * @class Explore
 * @brief A class adhering to the robot_actions::Action interface that moves the
 * robot base to explore its environment.
 */
class Explore : public rclcpp::Node
{
public:
  Explore();
  ~Explore();

  void start();
  void stop(bool finished_exploring = false, const std::string& reason = "");
  void resume();

  using NavigationGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using PlannerGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>;

  struct FrontierCandidate
  {
    geometry_msgs::msg::Point frontier;
    geometry_msgs::msg::Point staging;
    geometry_msgs::msg::Point unknown_centroid;
    double distance;
    double turn;
    double heading;
    double branch_distance;
    double frontier_length;
    double unknown_gain;
    double unknown_heading;
    std::vector<geometry_msgs::msg::Point> topology_path;
  };

  struct BranchChoice
  {
    geometry_msgs::msg::Point anchor;
    geometry_msgs::msg::Point frontier;
    geometry_msgs::msg::Point unknown_centroid;
    double heading;
    double frontier_length;
    double unknown_gain;
    std::uint64_t id;
  };

private:
  /**
   * @brief  Make a global plan
   */
  void makePlan();
  void makeGlobalPlan(const geometry_msgs::msg::Pose& pose, bool handoff);
  bool global_regions_enabled_{false};
  bool global_shadow_only_{false};
  GlobalRegions global_regions_;
  ObservationProgress global_observation_progress_;
  geometry_msgs::msg::Point global_pending_staging_;
  bool global_view_latched_{false};
  std::uint64_t global_view_region_{0};
  struct VisitedGlobalView {
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Point frontier;
    double area;
  };
  std::vector<VisitedGlobalView> global_visited_views_;
  std::vector<std::pair<geometry_msgs::msg::Point, double>> global_failed_views_;
  double global_last_gain_time_{-1.0};
  double global_known_area_{0.0};

  // /**
  //  * @brief  Publish a frontiers as markers
  //  */
  void visualizeFrontiers(
      const std::vector<frontier_exploration::Frontier>& frontiers);
  void visualizeSelectedGoal(
      const geometry_msgs::msg::Point& staging_goal,
      bool backtracking);

  bool goalOnBlacklist(const geometry_msgs::msg::Point& goal);
  bool findSafeStagingPoint(
      const frontier_exploration::Frontier& frontier,
      const geometry_msgs::msg::Point& robot_position,
      geometry_msgs::msg::Point& staging_point,
      std::vector<geometry_msgs::msg::Point>* alternatives = nullptr);
  double estimateUnknownGain(
      const frontier_exploration::Frontier& frontier,
      const geometry_msgs::msg::Point& observation_point,
      geometry_msgs::msg::Point& unknown_centroid);
  bool knownFreeWithClearance(double x, double y);
  bool findKnownFreePath(
      const geometry_msgs::msg::Point& start,
      const geometry_msgs::msg::Point& goal,
      std::vector<geometry_msgs::msg::Point>& path);
  int navigationCostAt(double x, double y);
  bool selectLocalGoal(
      const nav_msgs::msg::Path& path,
      const geometry_msgs::msg::Point& requested_goal,
      const geometry_msgs::msg::Point& robot_position,
      bool enforce_forward_progress,
      double goal_step_limit,
      geometry_msgs::msg::Point& local_goal,
      bool& reaches_frontier,
      int& maximum_cost);
  void checkPathToFrontier(
      const geometry_msgs::msg::Point& frontier_goal,
      const geometry_msgs::msg::Point& staging_goal,
      const geometry_msgs::msg::Point& robot_position,
      double robot_yaw,
      bool continuing_branch,
      bool backtracking = false,
      bool continuous_handoff = false);
  void sendNavigationGoal(
      const geometry_msgs::msg::Point& frontier_goal,
      const geometry_msgs::msg::Point& navigation_goal,
      const geometry_msgs::msg::Point& robot_position,
      double target_yaw,
      double robot_yaw,
      bool reaches_frontier,
      bool continuing_branch,
      bool backtracking,
      int maximum_path_cost,
      bool soft_escape,
      bool continuous_handoff);
  void rejectFrontier(const geometry_msgs::msg::Point& frontier_goal,
                      const char* reason,
                      bool keep_branch = false);
  void deferFrontier(
      const geometry_msgs::msg::Point& frontier_goal,
      double cooldown = -1.0,
      double radius = -1.0);
  void rememberAlternativeBranches(
      const std::vector<FrontierCandidate>& candidates,
      const FrontierCandidate& selected,
      const geometry_msgs::msg::Point& anchor);
  bool branchChoiceKnown(const BranchChoice& choice) const;
  bool branchChoiceCompleted(const BranchChoice& choice) const;
  bool candidateMatchesChoice(
      const FrontierCandidate& candidate,
      const BranchChoice& choice) const;
  void completeCurrentBranch(
      const geometry_msgs::msg::Point& robot_position,
      const char* reason,
      bool remember_completed = true);
  void abandonResumeChoice(const char* reason);

  NavigationGoalHandle::SharedPtr navigation_goal_handle_;
  PlannerGoalHandle::SharedPtr planner_goal_handle_;
  // void
  // goal_response_callback(std::shared_future<NavigationGoalHandle::SharedPtr>
  // future);
  void reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                   const geometry_msgs::msg::Point& frontier_goal);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      navigation_costmap_subscription_;
  nav_msgs::msg::OccupancyGrid::SharedPtr navigation_costmap_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr navigation_updates_subscription_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_publisher_;

  /**
    * @brief Publisher for exploration status updates (see ExploreStatus.msg for status values)
    */
  rclcpp::Publisher<explore_lite_msgs::msg::ExploreStatus>::SharedPtr status_pub_;

  rclcpp::Logger logger_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  Costmap2DClient costmap_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      move_base_client_;
  rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr
      planner_client_;
  frontier_exploration::FrontierSearch search_;
  rclcpp::TimerBase::SharedPtr exploring_timer_;
  // rclcpp::TimerBase::SharedPtr oneshot_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_subscription_;
  void resumeCallback(const std_msgs::msg::Bool::SharedPtr msg);

  std::vector<geometry_msgs::msg::Point> frontier_blacklist_;
  struct DeferredFrontier
  {
    geometry_msgs::msg::Point point;
    rclcpp::Time retry_after;
    double radius;

    DeferredFrontier(
        const geometry_msgs::msg::Point& deferred_point,
        const rclcpp::Time& deferred_retry_after,
        double deferred_radius)
      : point(deferred_point), retry_after(deferred_retry_after),
        radius(deferred_radius)
    {
    }
  };
  std::vector<DeferredFrontier> deferred_frontiers_;
  geometry_msgs::msg::Point prev_goal_;
  geometry_msgs::msg::Point last_progress_pose_;
  geometry_msgs::msg::Point pending_frontier_goal_;
  geometry_msgs::msg::Point pending_unknown_centroid_;
  geometry_msgs::msg::Point active_navigation_goal_;
  geometry_msgs::msg::Point branch_frontier_;
  geometry_msgs::msg::Point branch_anchor_;
  geometry_msgs::msg::Point branch_unknown_centroid_;
  geometry_msgs::msg::Point last_branch_memory_anchor_;
  std::vector<BranchChoice> branch_stack_;
  std::vector<BranchChoice> completed_branches_;
  BranchChoice resume_choice_;
  double prev_distance_;
  double last_progress_yaw_{0.0};
  double branch_heading_{0.0};
  double branch_unknown_gain_{0.0};
  double pending_unknown_gain_{0.0};
  rclcpp::Time last_progress_;
  size_t last_markers_count_;

  geometry_msgs::msg::Pose initial_pose_;
  void returnToInitialPose(void);

  // parameters
  double planner_frequency_;
  double potential_scale_, orientation_scale_, gain_scale_;
  double progress_timeout_;
  double progress_distance_;
  double progress_angle_;
  double blacklist_radius_;
  double goal_clearance_;
  double staging_search_radius_;
  double staging_retry_cooldown_;
  double path_check_timeout_;
  double heading_weight_;
  double staging_cost_weight_;
  double min_goal_distance_;
  double goal_settle_time_;
  double max_goal_step_;
  double soft_escape_goal_step_;
  double branch_weight_;
  double branch_match_radius_;
  double branch_direction_weight_;
  double branch_forward_angle_;
  double min_forward_progress_;
  double junction_angle_;
  double branch_choice_radius_;
  double branch_anchor_tolerance_;
  double junction_memory_min_travel_;
  double junction_cluster_radius_;
  double resume_anchor_match_radius_;
  double resume_heading_tolerance_;
  double completed_branch_radius_;
  double frontier_size_weight_;
  double unknown_gain_radius_;
  double unknown_visibility_half_angle_;
  double unknown_gain_weight_;
  double unknown_gain_cap_;
  double min_unknown_gain_;
  double branch_memory_min_unknown_gain_;
  double completed_branch_reopen_gain_delta_;
  double unknown_basin_match_radius_;
  double unknown_basin_weight_;
  double frontier_refresh_delay_;
  double branch_end_timeout_;
  double retry_cooldown_;
  double stalled_retry_cooldown_;
  double stalled_region_radius_;
  double no_candidate_timeout_;
  double hard_cost_wait_timeout_;
  double continuous_handoff_distance_;
  double continuous_handoff_min_goal_age_;
  double continuous_handoff_cooldown_;
  double continuous_handoff_heading_tolerance_;
  double continuous_handoff_min_advance_;
  int max_staging_cost_;
  int max_path_cost_;
  int max_robot_cost_;
  int hard_stop_cost_;
  int max_branch_stack_size_;
  int max_siblings_per_junction_;
  bool visualize_;
  bool return_to_init_;
  std::string robot_base_frame_;
  std::string behavior_tree_;
  std::string navigation_costmap_topic_;
  bool near_first_;
  CoverageConfirmation coverage_confirmation_;
  bool resuming_ = false;
  bool goal_active_{false};
  bool goal_reaches_frontier_{true};
  bool branch_active_{false};
  bool branch_memory_anchor_valid_{false};
  bool resume_choice_active_{false};
  bool backtracking_{false};
  bool active_goal_backtracking_{false};
  bool path_check_active_{false};
  bool path_check_backtracking_{false};
  bool path_check_continuing_{false};
  bool path_check_continuous_handoff_{false};
  bool active_goal_soft_escape_{false};
  bool continuous_handoff_enabled_{true};
  bool exploring_{true};
  std::uint64_t path_request_id_{0};
  std::uint64_t next_branch_id_{1};
  std::uint64_t active_branch_id_{0};
  rclcpp::Time path_check_started_;
  rclcpp::Time last_goal_completed_;
  rclcpp::Time no_candidate_since_;
  rclcpp::Time hard_cost_since_;
  rclcpp::Time active_goal_started_;
  rclcpp::Time last_continuous_handoff_;
  rclcpp_action::GoalUUID active_goal_id_;
};
}  // namespace explore

#endif
