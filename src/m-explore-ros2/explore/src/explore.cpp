#include "explore/rolling_handoff.hpp"
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

#include <explore/explore.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <mutex>
#include <queue>
#include <thread>

#include "nav2_costmap_2d/cost_values.hpp"

inline static double yaw_from_quaternion(
    const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline static double angular_distance(double one, double two)
{
  return std::abs(std::atan2(std::sin(one - two), std::cos(one - two)));
}

namespace explore
{
Explore::Explore()
  : Node("explore_node")
  , logger_(this->get_logger())
  , tf_buffer_(this->get_clock())
  , tf_listener_(tf_buffer_)
  , costmap_client_(*this, &tf_buffer_)
  , prev_distance_(0)
  , last_markers_count_(0)
{
  double timeout;
  double min_frontier_size;
  this->declare_parameter<float>("planner_frequency", 1.0);
  this->declare_parameter<float>("progress_timeout", 30.0);
  this->declare_parameter<float>("progress_distance", 0.05);
  this->declare_parameter<float>("progress_angle", 0.15);
  this->declare_parameter<float>("blacklist_radius", 0.40);
  this->declare_parameter<float>("goal_clearance", 0.25);
  this->declare_parameter<float>("staging_search_radius", 1.00);
  this->declare_parameter<float>("staging_retry_cooldown", 5.0);
  this->declare_parameter<float>("path_check_timeout", 6.0);
  this->declare_parameter<float>("heading_weight", 0.45);
  this->declare_parameter<float>("staging_cost_weight", 0.50);
  this->declare_parameter<float>("min_goal_distance", 0.25);
  this->declare_parameter<float>("goal_settle_time", 0.35);
  this->declare_parameter<float>("max_goal_step", 0.80);
  this->declare_parameter<float>("soft_escape_goal_step", 0.30);
  this->declare_parameter<float>("branch_weight", 0.75);
  this->declare_parameter<float>("branch_match_radius", 1.00);
  this->declare_parameter<float>("branch_direction_weight", 0.80);
  this->declare_parameter<float>("branch_forward_angle", 1.22);
  this->declare_parameter<float>("min_forward_progress", 0.10);
  this->declare_parameter<float>("junction_angle", 0.70);
  this->declare_parameter<float>("branch_choice_radius", 0.85);
  this->declare_parameter<float>("branch_anchor_tolerance", 0.30);
  this->declare_parameter<float>("junction_memory_min_travel", 1.20);
  this->declare_parameter<float>("junction_cluster_radius", 0.90);
  this->declare_parameter<float>("resume_anchor_match_radius", 0.75);
  this->declare_parameter<float>("resume_heading_tolerance", 1.05);
  this->declare_parameter<float>("completed_branch_radius", 0.85);
  this->declare_parameter<float>("frontier_size_weight", 0.35);
  this->declare_parameter<float>("unknown_gain_radius", 1.50);
  this->declare_parameter<float>("unknown_visibility_half_angle", 1.05);
  this->declare_parameter<float>("unknown_gain_weight", 0.80);
  this->declare_parameter<float>("unknown_gain_cap", 4.00);
  this->declare_parameter<float>("min_unknown_gain", 0.50);
  this->declare_parameter<float>("branch_memory_min_unknown_gain", 0.75);
  this->declare_parameter<float>(
      "completed_branch_reopen_gain_delta", 0.75);
  this->declare_parameter<float>("unknown_basin_match_radius", 1.50);
  this->declare_parameter<float>("unknown_basin_weight", 0.35);
  this->declare_parameter<float>("frontier_refresh_delay", 1.50);
  this->declare_parameter<float>("branch_end_timeout", 1.5);
  this->declare_parameter<int>("max_branch_stack_size", 12);
  this->declare_parameter<int>("max_siblings_per_junction", 3);
  this->declare_parameter<float>("retry_cooldown", 20.0);
  this->declare_parameter<float>("stalled_retry_cooldown", 60.0);
  this->declare_parameter<float>("stalled_region_radius", 0.85);
  this->declare_parameter<float>("no_candidate_timeout", 15.0);
  this->declare_parameter<float>("hard_cost_wait_timeout", 15.0);
  this->declare_parameter<bool>("continuous_handoff_enabled", true);
  this->declare_parameter<float>("continuous_handoff_distance", 0.45);
  this->declare_parameter<float>("continuous_handoff_min_goal_age", 2.0);
  this->declare_parameter<float>("continuous_handoff_cooldown", 1.5);
  this->declare_parameter<float>("continuous_handoff_heading_tolerance", 0.52);
  this->declare_parameter<float>("continuous_handoff_min_advance", 0.20);
  this->declare_parameter<int>("max_staging_cost", 20);
  this->declare_parameter<int>("max_path_cost", 80);
  this->declare_parameter<int>("max_robot_cost", 40);
  this->declare_parameter<int>("hard_stop_cost", 99);
  this->declare_parameter<std::string>(
      "navigation_costmap_topic", "/global_costmap/costmap");
  this->declare_parameter<std::string>("behavior_tree", "");
  this->declare_parameter<bool>("near_first", true);
  this->declare_parameter<bool>("visualize", false);
  this->declare_parameter<bool>("global_regions_enabled", false);
  this->declare_parameter<double>("global_max_goal_step", 1.60);
  this->declare_parameter<bool>("global_shadow_only", false);
  this->declare_parameter<float>("potential_scale", 1e-3);
  this->declare_parameter<float>("orientation_scale", 0.0);
  this->declare_parameter<float>("gain_scale", 1.0);
  this->declare_parameter<float>("min_frontier_size", 0.5);
  this->declare_parameter<bool>("return_to_init", false);

  this->get_parameter("planner_frequency", planner_frequency_);
  this->get_parameter("progress_timeout", timeout);
  this->get_parameter("progress_distance", progress_distance_);
  this->get_parameter("progress_angle", progress_angle_);
  this->get_parameter("blacklist_radius", blacklist_radius_);
  this->get_parameter("goal_clearance", goal_clearance_);
  this->get_parameter("staging_search_radius", staging_search_radius_);
  this->get_parameter("staging_retry_cooldown", staging_retry_cooldown_);
  this->get_parameter("path_check_timeout", path_check_timeout_);
  this->get_parameter("heading_weight", heading_weight_);
  this->get_parameter("staging_cost_weight", staging_cost_weight_);
  this->get_parameter("min_goal_distance", min_goal_distance_);
  this->get_parameter("goal_settle_time", goal_settle_time_);
  this->get_parameter("max_goal_step", max_goal_step_);
  this->get_parameter("soft_escape_goal_step", soft_escape_goal_step_);
  this->get_parameter("branch_weight", branch_weight_);
  this->get_parameter("branch_match_radius", branch_match_radius_);
  this->get_parameter("branch_direction_weight", branch_direction_weight_);
  this->get_parameter("branch_forward_angle", branch_forward_angle_);
  this->get_parameter("min_forward_progress", min_forward_progress_);
  this->get_parameter("junction_angle", junction_angle_);
  this->get_parameter("branch_choice_radius", branch_choice_radius_);
  this->get_parameter("branch_anchor_tolerance", branch_anchor_tolerance_);
  this->get_parameter(
      "junction_memory_min_travel", junction_memory_min_travel_);
  this->get_parameter("junction_cluster_radius", junction_cluster_radius_);
  this->get_parameter(
      "resume_anchor_match_radius", resume_anchor_match_radius_);
  this->get_parameter(
      "resume_heading_tolerance", resume_heading_tolerance_);
  this->get_parameter("completed_branch_radius", completed_branch_radius_);
  this->get_parameter("frontier_size_weight", frontier_size_weight_);
  this->get_parameter("unknown_gain_radius", unknown_gain_radius_);
  this->get_parameter(
      "unknown_visibility_half_angle", unknown_visibility_half_angle_);
  this->get_parameter("unknown_gain_weight", unknown_gain_weight_);
  this->get_parameter("unknown_gain_cap", unknown_gain_cap_);
  this->get_parameter("min_unknown_gain", min_unknown_gain_);
  this->get_parameter(
      "branch_memory_min_unknown_gain", branch_memory_min_unknown_gain_);
  this->get_parameter(
      "completed_branch_reopen_gain_delta",
      completed_branch_reopen_gain_delta_);
  this->get_parameter(
      "unknown_basin_match_radius", unknown_basin_match_radius_);
  this->get_parameter("unknown_basin_weight", unknown_basin_weight_);
  this->get_parameter("frontier_refresh_delay", frontier_refresh_delay_);
  this->get_parameter("branch_end_timeout", branch_end_timeout_);
  this->get_parameter("max_branch_stack_size", max_branch_stack_size_);
  this->get_parameter(
      "max_siblings_per_junction", max_siblings_per_junction_);
  this->get_parameter("retry_cooldown", retry_cooldown_);
  this->get_parameter(
      "stalled_retry_cooldown", stalled_retry_cooldown_);
  this->get_parameter("stalled_region_radius", stalled_region_radius_);
  this->get_parameter("no_candidate_timeout", no_candidate_timeout_);
  this->get_parameter("hard_cost_wait_timeout", hard_cost_wait_timeout_);
  this->get_parameter(
      "continuous_handoff_enabled", continuous_handoff_enabled_);
  this->get_parameter(
      "continuous_handoff_distance", continuous_handoff_distance_);
  this->get_parameter(
      "continuous_handoff_min_goal_age", continuous_handoff_min_goal_age_);
  this->get_parameter(
      "continuous_handoff_cooldown", continuous_handoff_cooldown_);
  this->get_parameter(
      "continuous_handoff_heading_tolerance",
      continuous_handoff_heading_tolerance_);
  this->get_parameter(
      "continuous_handoff_min_advance", continuous_handoff_min_advance_);
  this->get_parameter("max_staging_cost", max_staging_cost_);
  this->get_parameter("max_path_cost", max_path_cost_);
  this->get_parameter("max_robot_cost", max_robot_cost_);
  this->get_parameter("hard_stop_cost", hard_stop_cost_);
  this->get_parameter("navigation_costmap_topic", navigation_costmap_topic_);
  this->get_parameter("behavior_tree", behavior_tree_);
  this->get_parameter("near_first", near_first_);
  this->get_parameter("visualize", visualize_);
  this->get_parameter("global_regions_enabled", global_regions_enabled_);
  // Longer execution on verified known-space routes; legacy remains unchanged.
  if (global_regions_enabled_) this->get_parameter("global_max_goal_step", max_goal_step_);
  this->get_parameter("global_shadow_only", global_shadow_only_);
  this->get_parameter("potential_scale", potential_scale_);
  this->get_parameter("orientation_scale", orientation_scale_);
  this->get_parameter("gain_scale", gain_scale_);
  this->get_parameter("min_frontier_size", min_frontier_size);
  this->get_parameter("return_to_init", return_to_init_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  progress_timeout_ = timeout;
  move_base_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, ACTION_NAME);
  planner_client_ =
      rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
          this, "compute_path_to_pose");

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size, logger_);

  if (visualize_) {
    marker_array_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("explore/"
                                                                     "frontier"
                                                                     "s",
                                                                     10);
  }

  // Publisher for exploration status
  rclcpp::QoS status_qos(10);
  status_qos.transient_local();
  status_pub_ = this->create_publisher<explore_lite_msgs::msg::ExploreStatus>("explore/status", status_qos);

  // Subscription to resume or stop exploration
  resume_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&Explore::resumeCallback, this, std::placeholders::_1));

  rclcpp::QoS navigation_costmap_qos(rclcpp::KeepLast(1));
  navigation_costmap_qos.reliable();
  navigation_costmap_qos.transient_local();
  navigation_costmap_subscription_ =
      this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          navigation_costmap_topic_, navigation_costmap_qos,
          [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            navigation_costmap_ = msg;
          });

  navigation_updates_subscription_ =
      this->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
          navigation_costmap_topic_ + "_updates", rclcpp::QoS(10).reliable(),
          [this](const map_msgs::msg::OccupancyGridUpdate::SharedPtr update) {
            auto grid = navigation_costmap_;
            if (!grid || update->header.frame_id != grid->header.frame_id ||
                update->x < 0 || update->y < 0 ||
                static_cast<unsigned>(update->x) > grid->info.width ||
                static_cast<unsigned>(update->y) > grid->info.height ||
                update->width > grid->info.width - update->x ||
                update->height > grid->info.height - update->y ||
                update->data.size() != static_cast<size_t>(update->width) * update->height ||
                grid->data.size() != static_cast<size_t>(grid->info.width) * grid->info.height ||
                rclcpp::Time(update->header.stamp) < rclcpp::Time(grid->header.stamp)) return;
            // Copy-on-write keeps a coherent snapshot for current readers.
            auto next = std::make_shared<nav_msgs::msg::OccupancyGrid>(*grid);
            for (size_t y = 0; y < update->height; ++y) {
              std::copy_n(update->data.begin() + y * update->width, update->width,
                  next->data.begin() + (y + update->y) * next->info.width + update->x);
            }
            next->header.stamp = update->header.stamp;
            navigation_costmap_ = next;
          });

  RCLCPP_INFO(logger_, "Waiting to connect to move_base nav2 server");
  move_base_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Waiting to connect to planner nav2 server");
  planner_client_->wait_for_action_server();
  RCLCPP_INFO(logger_, "Connected to move_base and planner nav2 servers");
  RCLCPP_INFO(
      logger_,
      "Rolling-horizon UNKNOWN-branch exploration V10.7: step <= %.2f m, "
      "clearance %.2f m, "
      "target/path/soft/hard costs <= %d/%d/%d/%d, escape step <= %.2f m, "
      "visible UNKNOWN radius/cone %.2f m/%.0f deg, "
      "goal/memory gains >= %.2f/%.2f m2, reopen delta %.2f m2, "
      "basin match %.2f m, junction spacing/cluster %.2f/%.2f m, "
      "resume route %.2f m/%.0f deg, stalled region %.2f m/%.0f s, "
      "hard-cost wait %.0f s, handoff %.2f m/%.0f deg, "
      "stack <= %d, siblings <= %d, tree %s",
      max_goal_step_,
      goal_clearance_, max_staging_cost_, max_path_cost_, max_robot_cost_,
      hard_stop_cost_, soft_escape_goal_step_,
      unknown_gain_radius_,
      unknown_visibility_half_angle_ * 180.0 / 3.14159265358979323846,
      min_unknown_gain_, branch_memory_min_unknown_gain_,
      completed_branch_reopen_gain_delta_, unknown_basin_match_radius_,
      junction_memory_min_travel_, junction_cluster_radius_,
      resume_anchor_match_radius_,
      resume_heading_tolerance_ * 180.0 / 3.14159265358979323846,
      stalled_region_radius_, stalled_retry_cooldown_, hard_cost_wait_timeout_,
      continuous_handoff_distance_,
      continuous_handoff_heading_tolerance_ * 180.0 / 3.14159265358979323846,
      max_branch_stack_size_, max_siblings_per_junction_,
      behavior_tree_.empty() ? "<Nav2 default>" : behavior_tree_.c_str());

  if (return_to_init_) {
    RCLCPP_INFO(logger_, "Getting initial pose of the robot");
    geometry_msgs::msg::TransformStamped transformStamped;
    std::string map_frame = costmap_client_.getGlobalFrameID();
    try {
      transformStamped = tf_buffer_.lookupTransform(
          map_frame, robot_base_frame_, tf2::TimePointZero);
      initial_pose_.position.x = transformStamped.transform.translation.x;
      initial_pose_.position.y = transformStamped.transform.translation.y;
      initial_pose_.orientation = transformStamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(logger_, "Couldn't find transform from %s to %s: %s",
                   map_frame.c_str(), robot_base_frame_.c_str(), ex.what());
      return_to_init_ = false;
    }
  }

  exploring_timer_ = this->create_wall_timer(
      std::chrono::milliseconds((uint16_t)(1000.0 / planner_frequency_)),
      [this]() { makePlan(); });
  // Start after the first timer tick. This gives the transient-local Nav2
  // global costmap subscription time to deliver its initial snapshot.
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
  RCLCPP_INFO(
      logger_, "Waiting for navigation costmap on %s before selecting goals",
      navigation_costmap_topic_.c_str());
}

Explore::~Explore()
{
  stop();
}

void Explore::resumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    resume();
  } else {
    stop();
  }
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  const auto blue = std_msgs::msg::ColorRGBA().set__b(1.0).set__a(0.5);
  const auto red = std_msgs::msg::ColorRGBA().set__r(1.0).set__a(0.5);

  RCLCPP_DEBUG(logger_, "visualising %lu frontiers", frontiers.size());
  visualization_msgs::msg::MarkerArray markers_msg;
  std::vector<visualization_msgs::msg::Marker>& markers = markers_msg.markers;
  visualization_msgs::msg::Marker m;

  // V3 displayed every UNKNOWN frontier centroid as a green sphere. Clear
  // RViz's persistent marker cache once so those old, misleading targets do
  // not remain after upgrading while RViz itself stays open.
  if (last_markers_count_ == 0) {
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.push_back(clear);
  }

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = this->now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // m.lifetime defaults to 0, means lives forever
  m.frame_locked = true;

  m.action = visualization_msgs::msg::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.id = int(id);
    m.pose.position.x = 0.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.0;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = id;

  // delete previous markers, which are now unused
  m.action = visualization_msgs::msg::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_->publish(markers_msg);
}

void Explore::visualizeSelectedGoal(
    const geometry_msgs::msg::Point& staging_goal,
    bool backtracking)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = costmap_client_.getGlobalFrameID();
  marker.header.stamp = this->now();
  marker.ns = "exploration_safe_goal";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position = staging_goal;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.18;
  marker.scale.y = 0.18;
  marker.scale.z = 0.05;
  marker.color.r = backtracking ? 1.0 : 0.0;
  marker.color.g = backtracking ? 0.55 : 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.frame_locked = true;

  visualization_msgs::msg::MarkerArray markers;
  markers.markers.push_back(marker);

  // Yellow points are remembered known-free junctions. Magenta points are
  // the corresponding frontier directions saved at those junctions. They
  // visualize DFS memory only; the single green/orange sphere above remains
  // the only pose actually sent to Nav2.
  visualization_msgs::msg::Marker junctions;
  junctions.header = marker.header;
  junctions.ns = "exploration_dfs_junctions";
  junctions.id = 0;
  junctions.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  junctions.action = visualization_msgs::msg::Marker::ADD;
  junctions.pose.orientation.w = 1.0;
  junctions.scale.x = 0.12;
  junctions.scale.y = 0.12;
  junctions.scale.z = 0.05;
  junctions.color.r = 1.0;
  junctions.color.g = 0.85;
  junctions.color.b = 0.0;
  junctions.color.a = 0.9;
  junctions.frame_locked = true;

  visualization_msgs::msg::Marker remembered_frontiers = junctions;
  remembered_frontiers.ns = "exploration_dfs_children";
  remembered_frontiers.type = visualization_msgs::msg::Marker::POINTS;
  remembered_frontiers.scale.x = 0.10;
  remembered_frontiers.scale.y = 0.10;
  remembered_frontiers.color.r = 0.85;
  remembered_frontiers.color.g = 0.0;
  remembered_frontiers.color.b = 1.0;

  for (const auto& choice : branch_stack_) {
    junctions.points.push_back(choice.anchor);
    remembered_frontiers.points.push_back(choice.frontier);
  }
  if (resume_choice_active_) {
    junctions.points.push_back(resume_choice_.anchor);
    remembered_frontiers.points.push_back(resume_choice_.frontier);
  }
  markers.markers.push_back(junctions);
  markers.markers.push_back(remembered_frontiers);
  marker_array_publisher_->publish(markers);
}

void Explore::makePlan()
{
  if (!exploring_) {
    return;
  }

  // The SLAM map only says whether a cell is known/free. Nav2's global
  // costmap additionally contains the inflated robot footprint and current
  // obstacle observations. Selecting a goal before that map arrives can put
  // a nominally-white target beside a chair leg or inside inflated cost.
  if (!navigation_costmap_) {
    RCLCPP_INFO_THROTTLE(
        logger_, *this->get_clock(), 5000,
        "Waiting for navigation costmap on %s before selecting goals",
        navigation_costmap_topic_.c_str());
    return;
  }

  auto pose = costmap_client_.getRobotPose();

  // Let SLAM and the global obstacle layer publish a fresh map after arrival.
  // Immediately selecting again previously produced a duplicate 14 cm target
  // followed by a distant target only 0.165 seconds later.
  if (last_goal_completed_.nanoseconds() != 0 &&
      this->now() - last_goal_completed_ <
          tf2::durationFromSec(goal_settle_time_)) {
    return;
  }

  // A planner request should normally return quickly. Never let a lost action
  // response freeze exploration forever.
  if (path_check_active_) {
    if (this->now() - path_check_started_ <=
        tf2::durationFromSec(path_check_timeout_)) {
      return;
    }

    RCLCPP_WARN(
        logger_,
        "Path precheck timed out after %.1f seconds for frontier (%.2f, %.2f); "
        "trying another region",
        path_check_timeout_, pending_frontier_goal_.x,
        pending_frontier_goal_.y);
    const bool was_continuous_handoff = path_check_continuous_handoff_;
    path_check_active_ = false;
    path_check_continuous_handoff_ = false;
    last_goal_completed_ = this->now();
    ++path_request_id_;
    planner_client_->async_cancel_all_goals();
    if (was_continuous_handoff) {
      // The old navigation goal is still active while a rolling successor is
      // only being validated. A failed precheck must never cancel or poison
      // that healthy motion.
      last_continuous_handoff_ = this->now();
    } else if (path_check_backtracking_) {
      abandonResumeChoice("path precheck to junction timed out");
    } else {
      deferFrontier(pending_frontier_goal_);
      if (!path_check_continuing_) {
        branch_active_ = false;
        active_branch_id_ = 0;
      }
    }
    return;
  }

  // Hold an accepted Nav2 goal while the robot is translating or deliberately
  // rotating toward the path. V5 counted translation only, so its first
  // 0.35 m target was canceled after 15 seconds while the robot was still
  // making a healthy 93 degree heading correction.
  bool continuous_handoff_cycle = false;
  if (goal_active_ && !resuming_) {
    const double moved = std::hypot(
        pose.position.x - last_progress_pose_.x,
        pose.position.y - last_progress_pose_.y);
    const double robot_yaw = yaw_from_quaternion(pose.orientation);
    const double turned = angular_distance(robot_yaw, last_progress_yaw_);

    if (moved >= progress_distance_ || turned >= progress_angle_) {
      last_progress_pose_ = pose.position;
      last_progress_yaw_ = robot_yaw;
      last_progress_ = this->now();
    }

    if (this->now() - last_progress_ >
        tf2::durationFromSec(progress_timeout_)) {
      RCLCPP_WARN(
          logger_,
          "No translation or rotation progress for %.1f seconds; deferring "
          "locked %s (%.2f, %.2f)",
          progress_timeout_,
          active_goal_backtracking_ ? "DFS backtrack" : "frontier",
          prev_goal_.x, prev_goal_.y);
      goal_active_ = false;
      active_goal_id_ = rclcpp_action::GoalUUID{};
      move_base_client_->async_cancel_all_goals();
      last_goal_completed_ = this->now();
      if (global_regions_enabled_) {
        global_view_latched_=false;
        global_failed_views_.emplace_back(global_pending_staging_,
            this->now().seconds()+stalled_retry_cooldown_);
        return;  // Blocked destination remains a pending global region.
      }
      if (active_goal_backtracking_) {
        abandonResumeChoice("no motion while returning to junction");
      } else {
        deferFrontier(
            prev_goal_, stalled_retry_cooldown_, stalled_region_radius_);
        completeCurrentBranch(
            pose.position, "temporarily blocked with no motion", false);
      }
      return;
    }

    const double remaining = std::hypot(
        active_navigation_goal_.x - pose.position.x,
        active_navigation_goal_.y - pose.position.y);
    const bool old_enough =
        active_goal_started_.nanoseconds() != 0 &&
        this->now() - active_goal_started_ >=
            tf2::durationFromSec(continuous_handoff_min_goal_age_);
    const bool cooldown_elapsed =
        last_continuous_handoff_.nanoseconds() == 0 ||
        this->now() - last_continuous_handoff_ >=
            tf2::durationFromSec(continuous_handoff_cooldown_);
    continuous_handoff_cycle =
        continuous_handoff_enabled_ && old_enough &&
        (global_regions_enabled_ ? (global_view_latched_ && !goal_reaches_frontier_) : branch_active_) &&
        cooldown_elapsed && !active_goal_backtracking_ &&
        !active_goal_soft_escape_ &&
        remaining <= continuous_handoff_distance_;
    if (!continuous_handoff_cycle) {
      return;
    }

    RCLCPP_DEBUG_THROTTLE(
        logger_, *this->get_clock(), 1000,
        "Preparing a rolling successor with %.2f m left on branch #%llu",
        remaining, static_cast<unsigned long long>(active_branch_id_));
  }

  // Published Nav2 costs 1..98 are inflation gradients, not collisions. V10
  // treated every value above the comfort threshold as an invalid start and
  // could strand a stopped robot in cost 67 forever. Only unknown/outside and
  // inscribed/lethal cells remain hard stops. A soft-cost start is allowed to
  // request one short, planner-checked connector ending at cost <= 20; RPP
  // and collision_monitor still have final authority over the motion.
  const int robot_cost = navigationCostAt(pose.position.x, pose.position.y);
  if (robot_cost < 0 || robot_cost >= hard_stop_cost_) {
    if (hard_cost_since_.nanoseconds() == 0) {
      hard_cost_since_ = this->now();
    }
    if (this->now() - hard_cost_since_ >
        tf2::durationFromSec(hard_cost_wait_timeout_)) {
      RCLCPP_ERROR(
          logger_,
          "Robot remained in unknown/inscribed/lethal global cost %d for "
          "%.1f seconds after navigation stopped. Safety-pausing exploration "
          "instead of waiting forever; move the robot to clear white space "
          "or correct localization, then restart/resume exploration",
          robot_cost, hard_cost_wait_timeout_);
      stop(false, "exploration_blocked:unsafe_start");
      return;
    }
    RCLCPP_WARN_THROTTLE(
        logger_, *this->get_clock(), 5000,
        "Robot start cell is unknown, inscribed, or lethal (global cost %d, "
        "hard stop >= %d); waiting up to %.1f seconds for costmap clearance",
        robot_cost, hard_stop_cost_, hard_cost_wait_timeout_);
    return;
  }
  hard_cost_since_ =
      rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  if (robot_cost > max_robot_cost_) {
    RCLCPP_WARN_THROTTLE(
        logger_, *this->get_clock(), 5000,
        "Robot is in soft inflation (global cost %d > comfort %d); allowing "
        "only a <= %.2f m planner-checked escape step to a cost <= %d white "
        "cell",
        robot_cost, max_robot_cost_, soft_escape_goal_step_,
        max_staging_cost_);
  }

  if (global_regions_enabled_) {
    makeGlobalPlan(pose, continuous_handoff_cycle);
    return;
  }

  // V8/V9 physically replayed every connector back to an old junction before
  // opening its remembered child. On a live map that produced many short
  // reverse goals, and the saved junction could later lie in inflated cost.
  // V10 keeps the DFS stack as logical memory, then lets Nav2 compute the
  // current shortest safe route directly to the remembered live branch. The
  // route still passes through a real bottleneck when the geometry requires
  // it, but open rooms no longer retrace obsolete breadcrumbs.
  if (resume_choice_active_ && backtracking_) {
    backtracking_ = false;
    RCLCPP_INFO(
        logger_,
        "DFS direct transit to remembered branch #%llu; Nav2 will use the "
        "current shortest safe path instead of replaying junction (%.2f, "
        "%.2f)",
        static_cast<unsigned long long>(resume_choice_.id),
        resume_choice_.anchor.x, resume_choice_.anchor.y);
  }

  if (!branch_active_ && !resume_choice_active_ &&
      !branch_stack_.empty()) {
    while (!branch_stack_.empty()) {
      const BranchChoice next = branch_stack_.back();
      branch_stack_.pop_back();
      if (branchChoiceCompleted(next)) {
        continue;
      }
      resume_choice_ = next;
      resume_choice_active_ = true;
      backtracking_ = true;
      RCLCPP_INFO(
          logger_,
          "DFS selected pending branch #%llu; preparing direct transit from "
          "current pose (saved junction %.2f, %.2f)",
          static_cast<unsigned long long>(next.id),
          next.anchor.x, next.anchor.y);
      return;
    }
  }

  // Find frontiers. Upstream sorts by a cost that mixes distance and frontier
  // size. Real robots are safer and more efficient when nearby reachable
  // regions are exhausted before long cross-room trips.
  auto frontiers = search_.searchFrom(pose.position);
  if (near_first_) {
    std::stable_sort(
        frontiers.begin(), frontiers.end(),
        [](const frontier_exploration::Frontier& one,
           const frontier_exploration::Frontier& two) {
          return one.min_distance < two.min_distance;
        });
  }
  RCLCPP_DEBUG(logger_, "found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    RCLCPP_DEBUG(logger_, "frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    // A single empty SLAM snapshot is not proof that the whole map is done.
    // Let the normal DFS close the active branch, discard stale remembered
    // children, and then require no_candidate_timeout_ of stable emptiness.
    RCLCPP_INFO_THROTTLE(
        logger_, *this->get_clock(), 5000,
        "No frontiers in this map snapshot; validating DFS stack before "
        "declaring exploration complete");
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // Select a low-cost known-free staging point, not the UNKNOWN frontier
  // centroid. V10 copies the candidate data because branch choices persist
  // across map updates; pointers into this one frontier snapshot would be
  // invalid on the next planning cycle.
  std::vector<FrontierCandidate> candidates;
  const double robot_yaw = yaw_from_quaternion(pose.orientation);
  const double map_resolution = costmap_client_.getCostmap()->getResolution();
  for (const auto& frontier : frontiers) {
    if (goalOnBlacklist(frontier.centroid)) {
      continue;
    }
    geometry_msgs::msg::Point candidate;
    if (findSafeStagingPoint(frontier, pose.position, candidate)) {
      std::vector<geometry_msgs::msg::Point> topology_path;
      if (!findKnownFreePath(pose.position, candidate, topology_path)) {
        RCLCPP_DEBUG(
            logger_,
            "Skipping frontier (%.2f, %.2f): staging point is not connected "
            "to the robot through the current known-free map",
            frontier.centroid.x, frontier.centroid.y);
        continue;
      }

      geometry_msgs::msg::Point unknown_centroid;
      const double unknown_gain = estimateUnknownGain(
          frontier, candidate, unknown_centroid);
      if (unknown_gain < min_unknown_gain_) {
        RCLCPP_DEBUG(
            logger_,
            "Ignoring gray region at (%.2f, %.2f): only %.2f m2 of UNKNOWN "
            "is directly observable from reachable white staging point "
            "(minimum %.2f m2)",
            frontier.centroid.x, frontier.centroid.y,
            unknown_gain, min_unknown_gain_);
        continue;
      }

      double distance = 0.0;
      double heading = robot_yaw;
      for (size_t index = 1; index < topology_path.size(); ++index) {
        const double segment = std::hypot(
            topology_path[index].x - topology_path[index - 1].x,
            topology_path[index].y - topology_path[index - 1].y);
        distance += segment;
        if (distance <= 0.35 || index == 1) {
          heading = std::atan2(
              topology_path[index].y - pose.position.y,
              topology_path[index].x - pose.position.x);
        }
      }
      if (distance < min_goal_distance_) {
        RCLCPP_INFO(
            logger_,
            "Frontier (%.2f, %.2f) is already observed from its safe target "
            "(%.2f m < %.2f m); waiting %.1f seconds for SLAM to advance "
            "the UNKNOWN boundary (gain %.2f m2)",
            frontier.centroid.x, frontier.centroid.y, distance,
            min_goal_distance_, retry_cooldown_, unknown_gain);
        // Reaching a white staging point does not mean the whole branch has
        // been explored. The old policy permanently blacklisted it here and
        // immediately walked back from large openings. Give SLAM time to move
        // the boundary; only the unchanged local centroid is cooled down.
        deferFrontier(frontier.centroid);
        continue;
      }
      const double turn = angular_distance(heading, robot_yaw);
      const double branch_distance = branch_active_
          ? std::hypot(frontier.centroid.x - branch_frontier_.x,
                       frontier.centroid.y - branch_frontier_.y)
          : 0.0;
      BranchChoice live_choice{
          pose.position, frontier.centroid, unknown_centroid, heading,
          static_cast<double>(frontier.size) * map_resolution,
          unknown_gain, 0};
      if (branchChoiceCompleted(live_choice)) {
        RCLCPP_DEBUG(
            logger_, "Skipping completed branch region (%.2f, %.2f)",
            frontier.centroid.x, frontier.centroid.y);
        continue;
      }
      candidates.push_back(FrontierCandidate{
          frontier.centroid, candidate, unknown_centroid,
          distance, turn, heading,
          branch_distance,
          static_cast<double>(frontier.size) * map_resolution,
          unknown_gain,
          std::atan2(
              unknown_centroid.y - pose.position.y,
              unknown_centroid.x - pose.position.x),
          topology_path});
      continue;
    }

    RCLCPP_INFO(
        logger_,
        "Skipping frontier (%.2f, %.2f): no staging point that is known-free, "
        "has %.2f m map clearance, and global cost <= %d; retrying in %.1f "
        "seconds",
        frontier.centroid.x, frontier.centroid.y, goal_clearance_,
        max_staging_cost_, staging_retry_cooldown_);
    deferFrontier(frontier.centroid, staging_retry_cooldown_);
    // This is not a permanent failure. As the robot advances along another
    // branch, SLAM and obstacle clearing can create a valid white staging cell
    // here. V5 blacklisted every such frontier immediately and could declare
    // exploration complete after a single map snapshot.
  }

  if (candidates.empty()) {
    if (continuous_handoff_cycle) {
      // Keep driving the current goal. Normal arrival handling will retry from
      // a fresher SLAM boundary if no safe forward successor exists yet.
      return;
    }
    if (branch_active_) {
      if (no_candidate_since_.nanoseconds() == 0) {
        no_candidate_since_ = this->now();
        RCLCPP_INFO(
            logger_,
            "Current DFS branch has no safe forward candidate; waiting %.1f "
            "seconds for one more map update before closing it",
            branch_end_timeout_);
        return;
      }
      if (this->now() - no_candidate_since_ <
          tf2::durationFromSec(branch_end_timeout_)) {
        return;
      }
      completeCurrentBranch(pose.position, "no safe frontier remains");
      no_candidate_since_ =
          rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
      return;
    }

    if (resume_choice_active_) {
      RCLCPP_INFO(
          logger_,
          "Remembered child at junction #%llu disappeared after map update; "
          "marking it complete",
          static_cast<unsigned long long>(resume_choice_.id));
      completed_branches_.push_back(resume_choice_);
      resume_choice_active_ = false;
      backtracking_ = false;
      no_candidate_since_ =
          rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
      return;
    }

    if (no_candidate_since_.nanoseconds() == 0) {
      no_candidate_since_ = this->now();
      RCLCPP_WARN(
          logger_,
          "No safe local branch candidate in the current map; waiting up to "
          "%.1f seconds for mapping or obstacle updates",
          no_candidate_timeout_);
      return;
    }
    if (this->now() - no_candidate_since_ <
        tf2::durationFromSec(no_candidate_timeout_)) {
      return;
    }
    // No admissible candidate is not the same as no unexplored frontier.
    // In the recorded failure the 15 s completion timer expired before the
    // rejected connector's 20 s retry cooldown. Give bounded retries a full
    // chance, then pause explicitly instead of reporting false completion.
    if (!frontiers.empty()) {
      const double retry_window = std::max(
          no_candidate_timeout_,
          std::max(retry_cooldown_, stalled_retry_cooldown_) +
          frontier_refresh_delay_ + 1.0);
      if ((this->now() - no_candidate_since_).seconds() < retry_window) {
        RCLCPP_WARN_THROTTLE(
            logger_, *this->get_clock(), 5000,
            "Unresolved frontiers remain (%zu); waiting for safe staging or "
            "cooldown expiry, bounded to %.1f s; map is not complete",
            frontiers.size(), retry_window);
        return;
      }
      RCLCPP_WARN(logger_,
          "Exploration paused: %zu unresolved frontiers have no safe candidate "
          "after bounded retries; inspect obstacles or resume after map changes",
          frontiers.size());
      stop(false, "exploration_blocked:no_safe_viewpoint");
      return;
    }
    RCLCPP_WARN(
        logger_,
        "No frontiers and no pending branch for %.1f seconds; exploration complete",
        no_candidate_timeout_);
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status_msg);
    stop(true);
    return;
  }

  const FrontierCandidate* selected = nullptr;
  double selected_score = std::numeric_limits<double>::infinity();
  bool continuing_branch = false;

  if (resume_choice_active_) {
    // Bind the remembered child to its current spatial UNKNOWN sector. Its
    // old heading was measured at the saved junction and is not comparable
    // with a path heading measured from the robot's new pose. Requiring those
    // two angles to match rejected valid long transit paths in V9.
    for (const auto& candidate : candidates) {
      if (!candidateMatchesChoice(candidate, resume_choice_)) {
        continue;
      }
      const double basin_distance = std::hypot(
          candidate.unknown_centroid.x - resume_choice_.unknown_centroid.x,
          candidate.unknown_centroid.y - resume_choice_.unknown_centroid.y);
      const double frontier_distance = std::hypot(
          candidate.frontier.x - resume_choice_.frontier.x,
          candidate.frontier.y - resume_choice_.frontier.y);
      const double score =
          candidate.distance +
          branch_weight_ * std::min(
              frontier_distance, 2.0 * branch_choice_radius_) -
          frontier_size_weight_ * candidate.frontier_length +
          unknown_basin_weight_ * std::min(
              basin_distance, unknown_basin_match_radius_) -
          unknown_gain_weight_ * std::min(
              candidate.unknown_gain, unknown_gain_cap_);
      if (score < selected_score) {
        selected_score = score;
        selected = &candidate;
      }
    }

    if (selected == nullptr) {
      RCLCPP_INFO(
          logger_,
          "No live frontier matches remembered child at junction #%llu; "
          "discarding that completed child",
          static_cast<unsigned long long>(resume_choice_.id));
      completed_branches_.push_back(resume_choice_);
      resume_choice_active_ = false;
      return;
    }
  } else if (branch_active_) {
    // Staying on a branch is now a geometric invariant, not merely a bonus
    // for a nearby frontier centroid. The next staging point must be ahead of
    // the last path heading and remain inside a wide cone so curved corridors
    // are supported without permitting a silent U-turn.
    for (const auto& candidate : candidates) {
      const double direction_error =
          angular_distance(candidate.heading, branch_heading_);
      const double basin_distance = std::hypot(
          candidate.unknown_centroid.x - branch_unknown_centroid_.x,
          candidate.unknown_centroid.y - branch_unknown_centroid_.y);
      const double projection =
          std::min(candidate.distance, 0.35) * std::cos(direction_error);
      const bool same_unknown_basin =
          basin_distance <= unknown_basin_match_radius_;
      // UNKNOWN outside a partially mapped room can remain one connected
      // component around several doors. It is useful for gain ranking, but it
      // must not relax direction enough to jump between opposite sides of the
      // room. Curvature is followed incrementally as branch_heading_ updates.
      const double allowed_direction_error = continuous_handoff_cycle
          ? std::min(
                branch_forward_angle_,
                continuous_handoff_heading_tolerance_)
          : branch_forward_angle_;
      if (direction_error > allowed_direction_error ||
          (!same_unknown_basin && projection < min_forward_progress_)) {
        continue;
      }
      const double score =
          candidate.distance + heading_weight_ * candidate.turn +
          branch_direction_weight_ * direction_error +
          branch_weight_ * std::min(candidate.branch_distance,
                                    branch_match_radius_) -
          frontier_size_weight_ * candidate.frontier_length +
          unknown_basin_weight_ * std::min(
              basin_distance, unknown_basin_match_radius_) -
          unknown_gain_weight_ * std::min(
              candidate.unknown_gain, unknown_gain_cap_);
      if (score < selected_score) {
        selected_score = score;
        selected = &candidate;
      }
    }

    if (selected == nullptr) {
      if (continuous_handoff_cycle) {
        return;
      }
      if (no_candidate_since_.nanoseconds() == 0) {
        no_candidate_since_ = this->now();
        RCLCPP_INFO(
            logger_,
            "No frontier lies ahead of the current DFS branch (heading %.0f "
            "deg); waiting %.1f seconds before switching branch",
            branch_heading_ * 180.0 / 3.14159265358979323846,
            branch_end_timeout_);
        return;
      }
      if (this->now() - no_candidate_since_ <
          tf2::durationFromSec(branch_end_timeout_)) {
        return;
      }
      completeCurrentBranch(pose.position, "no forward child remains");
      no_candidate_since_ =
          rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
      return;
    }
    continuing_branch = true;
  } else {
    // Root selection: combine nearby travel cost with information gain. The
    // frontier length is a stable proxy for the mouth/extent of a currently
    // visible branch; it is not claimed to be the final unknown-room area.
    for (const auto& candidate : candidates) {
      const double score =
          candidate.distance + heading_weight_ * candidate.turn -
          frontier_size_weight_ * candidate.frontier_length -
          unknown_gain_weight_ * std::min(
              candidate.unknown_gain, unknown_gain_cap_);
      if (score < selected_score) {
        selected_score = score;
        selected = &candidate;
      }
    }
  }

  if (selected == nullptr) {
    RCLCPP_ERROR(logger_, "Internal error: no candidate survived selection");
    return;
  }

  no_candidate_since_ =
      rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  const geometry_msgs::msg::Point frontier_position = selected->frontier;
  const geometry_msgs::msg::Point staging_position = selected->staging;

  // A remembered branch is a topological exit, not every frontier visible
  // from every 0.85 m connector. In particular, never create new siblings
  // while routing to an already remembered child: V10.4 did that at the old
  // branch endpoint and filled the stack with aliases of the same two edges.
  if (!resume_choice_active_ && !continuous_handoff_cycle) {
    rememberAlternativeBranches(candidates, *selected, pose.position);
  }
  const bool resuming_choice = resume_choice_active_;
  if (resuming_choice) {
    // The stored child direction is also checked against Nav2's real path.
    // Matching a drifting frontier centroid alone is not enough if the
    // planner would physically leave the junction through the wrong edge.
    branch_heading_ = resume_choice_.heading;
    resume_choice_active_ = false;
    backtracking_ = false;
  }

  // ensure only first call of makePlan was set resuming to true
  if (resuming_) {
    resuming_ = false;
  }

  RCLCPP_INFO(
      logger_,
      "V10.7 precheck frontier (%.2f, %.2f) via safe white target "
      "(%.2f, %.2f), distance %.2f m, turn %.0f deg, frontier %.2f m, "
      "UNKNOWN gain %.2f m2 toward (%.2f, %.2f), score %.2f, stack %lu%s",
      frontier_position.x, frontier_position.y,
      staging_position.x, staging_position.y,
      selected->distance,
      selected->turn * 180.0 / 3.14159265358979323846,
      selected->frontier_length, selected->unknown_gain,
      selected->unknown_centroid.x, selected->unknown_centroid.y,
      selected_score, branch_stack_.size(),
      continuing_branch ? ", continuing forward" :
      (resuming_choice ? ", opening remembered child" :
       (continuous_handoff_cycle ? ", rolling handoff" : ", opening root")));
  pending_unknown_centroid_ = selected->unknown_centroid;
  pending_unknown_gain_ = selected->unknown_gain;
  checkPathToFrontier(
      frontier_position, staging_position, pose.position, robot_yaw,
      continuing_branch || resuming_choice, false,
      continuous_handoff_cycle);
}

void Explore::makeGlobalPlan(const geometry_msgs::msg::Pose& pose, bool handoff)
{
  // Global planning bypasses the legacy branch-selection resume path below.
  // Consume resume once here; otherwise every timer tick preempts a live goal.
  resuming_ = false;
  // Extend only the same verified observation route. Do not change region or
  // count the preempted intermediate endpoint as a completed observation.
  if (handoff) {
    if (!global_view_latched_ || goal_reaches_frontier_) return;
    checkPathToFrontier(pending_unknown_centroid_, global_pending_staging_,
        pose.position, yaw_from_quaternion(pose.orientation), true, false, true);
    return;
  }

  const auto evaluation_start=std::chrono::steady_clock::now();
  auto raw = search_.searchFrom(pose.position);
  // Split oversized connected frontiers in map space. Otherwise one outside
  // UNKNOWN rim can alias several different entrances. This is a frontier
  // region approximation, not a claim that unknown room topology is known.
  std::vector<frontier_exploration::Frontier> frontiers;
  for (auto f : raw) {
    std::vector<std::vector<geometry_msgs::msg::Point>> queue{f.points};
    while (!queue.empty()) {
      auto points = std::move(queue.back()); queue.pop_back();
      if (points.empty()) continue;
      double minx=points[0].x,maxx=minx,miny=points[0].y,maxy=miny;
      for (const auto& p:points) {minx=std::min(minx,p.x);maxx=std::max(maxx,p.x);
        miny=std::min(miny,p.y);maxy=std::max(maxy,p.y);}
      if (std::max(maxx-minx,maxy-miny)>1.5 && points.size()>6) {
        bool x_axis=maxx-minx>=maxy-miny;
        std::sort(points.begin(),points.end(),[&](const auto& a,const auto& b){
          return x_axis?a.x<b.x:a.y<b.y;});
        auto mid=points.begin()+points.size()/2;
        queue.emplace_back(points.begin(),mid);queue.emplace_back(mid,points.end());
        continue;
      }
      f.points=points; f.size=points.size(); f.centroid.x=0;f.centroid.y=0;
      f.middle=points.front();f.min_distance=std::numeric_limits<double>::infinity();
      for (const auto& p:points) {
        f.centroid.x+=p.x; f.centroid.y+=p.y;
        double d=std::hypot(p.x-pose.position.x,p.y-pose.position.y);
        if(d<f.min_distance){f.min_distance=d;f.middle=p;}
      }
      f.centroid.x/=points.size();f.centroid.y/=points.size();
      frontiers.push_back(f);
    }
  }
  auto* map=costmap_client_.getCostmap();
  const double resolution=map->getResolution();
  std::vector<std::vector<RegionPoint>> observations;
  for (const auto& f:frontiers) {
    std::vector<RegionPoint> points;
    for(const auto& p:f.points) points.push_back({p.x,p.y});
    observations.push_back(std::move(points));
  }
  std::vector<std::uint64_t> ids;
  std::vector<double> area(frontiers.size(),0.0);
  std::vector<bool> narrow(frontiers.size(),false);
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(map->getMutex()));
    const size_t count=map->getSizeInCellsX()*map->getSizeInCellsY();
    const double known_area=std::count(map->getCharMap(),map->getCharMap()+count,
        nav2_costmap_2d::FREE_SPACE)*resolution*resolution;
    if(global_last_gain_time_<0 || known_area>=global_known_area_+0.25) {
      global_known_area_=known_area;
      global_last_gain_time_=this->now().seconds();
    }
    const auto revision=costmap_client_.getFullMapRevision();
    ids=global_regions_.update(observations,revision,[&](const RegionPoint& p){
      unsigned x,y;
      return map->worldToMap(p.x,p.y,x,y) && map->getCost(x,y)!=nav2_costmap_2d::NO_INFORMATION;
    });
    // Potential unseen area attached to each GLOBAL frontier segment. The
    // depth limits computational/area attribution, not distance from robot.
    // It is a frontier-fringe estimate, not the true size of an unknown room.
    const int width=map->getSizeInCellsX(),height=map->getSizeInCellsY();
    const int depth_limit=std::max(1,static_cast<int>(std::ceil(1.5/resolution)));
    const auto accessible=coverageReachable(map->getCharMap(),width,height,
        resolution,goal_clearance_);
    for(size_t i=0;i<frontiers.size();++i) {
      std::vector<int> depth(count,-1);std::queue<int> cells;
      for(const auto& p:frontiers[i].points) {
        unsigned x,y;if(!map->worldToMap(p.x,p.y,x,y))continue;
        int index=y*width+x;
        if(depth[index]<0 && accessible[index] && map->getCharMap()[index]==nav2_costmap_2d::NO_INFORMATION){
          depth[index]=0;cells.push(index);
        }
      }
      size_t unknown=0;
      while(!cells.empty()) {
        int index=cells.front();cells.pop();++unknown;
        if(depth[index]>=depth_limit)continue;
        int x=index%width,y=index/width;
        for(const auto& delta:std::vector<std::pair<int,int>>{{1,0},{-1,0},{0,1},{0,-1}}){
          int nx=x+delta.first,ny=y+delta.second;
          if(nx<0||ny<0||nx>=width||ny>=height)continue;
          int next=ny*width+nx;
          if(depth[next]<0 && accessible[next] && map->getCharMap()[next]==nav2_costmap_2d::NO_INFORMATION){
            depth[next]=depth[index]+1;cells.push(next);
          }
        }
      }
      area[i]=unknown*resolution*resolution;
      narrow[i]=!frontiers[i].points.empty() && std::all_of(
          frontiers[i].points.begin(),frontiers[i].points.end(),[&](const auto& p){
            unsigned x,y;if(!map->worldToMap(p.x,p.y,x,y))return false;
            return !accessible[y*width+x];
          });
    }
  }
  size_t actionable=0,ignored_small=0,ignored_narrow=0;
  for(size_t i=0;i<frontiers.size();++i) {
    if(narrow[i])++ignored_narrow;
    else if(area[i]<min_unknown_gain_)++ignored_small;
    else ++actionable;
  }
  unsigned rx,ry;
  const bool robot_in_map=map->worldToMap(pose.position.x,pose.position.y,rx,ry);
  const bool ready=coverage_confirmation_.update(robot_in_map && actionable==0,
      costmap_client_.getFullMapRevision(),this->now().seconds());
  RCLCPP_INFO_THROTTLE(logger_,*this->get_clock(),5000,
      "COVERAGE accessible scope: pending=%lu narrow=%lu small=%lu; completion requires 30 s / 3 map updates",
      actionable,ignored_narrow,ignored_small);
  if(ready && !global_shadow_only_) {
    RCLCPP_INFO(logger_,"Accessible exploration complete; residual narrow/small unknown retained in map");
    auto status=explore_lite_msgs::msg::ExploreStatus();
    status.status=explore_lite_msgs::msg::ExploreStatus::EXPLORATION_COMPLETE;
    status_pub_->publish(status);stop(true);return;
  }
  if(visualize_)visualizeFrontiers(frontiers);
  if(global_regions_.regions.size()>512) {
    RCLCPP_ERROR(logger_,"GLOBAL registry exceeded 512 retained regions; pausing for diagnosis instead of dropping branches");
    stop(false, "exploration_error:registry_limit");return;
  }
  const double now=this->now().seconds();
  global_failed_views_.erase(std::remove_if(global_failed_views_.begin(),global_failed_views_.end(),
      [&](const auto& v){return v.second<=now;}),global_failed_views_.end());
  struct Option {std::uint64_t id;size_t frontier;geometry_msgs::msg::Point view;double distance;double score;};
  std::vector<Option> options;
  std::vector<std::pair<std::uint64_t,double>> costs;
  for(size_t i=0;i<frontiers.size();++i) {
    if(narrow[i] || area[i]<min_unknown_gain_)continue;
    if(!global_observation_progress_.eligible(ids[i],frontiers[i].centroid.x,
        frontiers[i].centroid.y,area[i])) {
      RCLCPP_INFO_THROTTLE(logger_,*this->get_clock(),5000,
          "GLOBAL region %llu deferred: two observation attempts without meaningful frontier progress; NOT covered",
          static_cast<unsigned long long>(ids[i]));
      continue;
    }
    std::vector<geometry_msgs::msg::Point> views;geometry_msgs::msg::Point first;
    if(!findSafeStagingPoint(frontiers[i],pose.position,first,&views))continue;
    // Continue the same observation destination after an intermediate step.
    // Revalidate its known-space route and Nav2 collision checks every time.
    if(global_view_latched_ && ids[i]==global_view_region_)
      views.insert(views.begin(),global_pending_staging_);
    for(const auto& view:views) {
      if(std::any_of(global_visited_views_.begin(),global_visited_views_.end(),[&](const auto& v){
        return ObservationProgress::repeat(
            std::hypot(view.x-v.point.x,view.y-v.point.y),
            std::hypot(frontiers[i].centroid.x-v.frontier.x,
                       frontiers[i].centroid.y-v.frontier.y),v.area,area[i]);
      }))continue;
      if(std::any_of(global_failed_views_.begin(),global_failed_views_.end(),[&](const auto& v){
        return std::hypot(view.x-v.first.x,view.y-v.first.y)<0.20;}))continue;
      std::vector<geometry_msgs::msg::Point> route;
      if(!findKnownFreePath(pose.position,view,route))continue;
      double distance=0;
      for(size_t k=1;k<route.size();++k)distance+=std::hypot(route[k].x-route[k-1].x,route[k].y-route[k-1].y);
      if(distance<min_goal_distance_)continue;
      double edge_distance=std::numeric_limits<double>::infinity();
      for(const auto& edge:frontiers[i].points) {
        edge_distance=std::min(edge_distance,std::hypot(view.x-edge.x,view.y-edge.y));
      }
      // Do not let the cheapest nearby stopping point defeat progress toward
      // the globally chosen entrance. Distance alone reproduced origin loops.
      // Estimate visible frontier support, not unknown area behind a wall.
      // This is a viewpoint utility estimate; global branch discovery remains global.
      size_t visible=0,sampled=0;
      {
        auto* map=costmap_client_.getCostmap();
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(map->getMutex()));
        const auto stride=std::max<size_t>(1,(frontiers[i].points.size()+63)/64);
        for(size_t n=0;n<frontiers[i].points.size();n+=stride) {
          ++sampled;const auto& edge=frontiers[i].points[n];
          const double length=std::hypot(edge.x-view.x,edge.y-view.y);
          if(length>2.5)continue;
          bool clear=true;
          const int steps=std::max(1,static_cast<int>(std::ceil(length/(map->getResolution()*0.5))));
          for(int k=0;k<steps;++k) {
            unsigned mx,my;const double t=static_cast<double>(k)/steps;
            if(!map->worldToMap(view.x+t*(edge.x-view.x),view.y+t*(edge.y-view.y),mx,my)) {clear=false;break;}
            const auto cost=map->getCost(mx,my);
            if(cost!=nav2_costmap_2d::FREE_SPACE) {
              // Permit only the endpoint frontier cell to be unknown.
              unsigned ex,ey;
              if(!(cost==nav2_costmap_2d::NO_INFORMATION && map->worldToMap(edge.x,edge.y,ex,ey) && ex==mx && ey==my))clear=false;
              break;
            }
          }
          if(clear)++visible;
        }
      }
      const bool continuing=global_view_latched_ && ids[i]==global_view_region_ &&
          std::hypot(view.x-global_pending_staging_.x,view.y-global_pending_staging_.y)<0.01;
      if(!visible && !continuing)continue;
      const double visible_area=area[i]*static_cast<double>(visible)/std::max<size_t>(1,sampled);
      // Fixed per-goal overhead discourages many cheap moves with little information.
      const double score=(distance+0.80)/(0.10+visible_area)+0.5*edge_distance;
      options.push_back({ids[i],i,view,distance,score});
      costs.emplace_back(ids[i],score);
    }
  }
  auto latched=options.end();
  if(global_view_latched_) {
    latched=std::find_if(options.begin(),options.end(),[&](const auto& o){
      return o.id==global_view_region_ &&
          std::hypot(o.view.x-global_pending_staging_.x,o.view.y-global_pending_staging_.y)<0.01;
    });
    if(latched==options.end())global_view_latched_=false;
  }
  auto selected=global_view_latched_ ? global_view_region_ : global_regions_.choose(costs);
  const double evaluation_ms=std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-evaluation_start).count();
  RCLCPP_INFO_THROTTLE(logger_,*this->get_clock(),5000,
      "GLOBAL evaluation: %.1f ms, %lu live segments, %lu retained regions, %lu viewpoints",
      evaluation_ms,frontiers.size(),global_regions_.regions.size(),options.size());
  if(!selected) {
    if(handoff)return;
    if(no_candidate_since_.nanoseconds()==0)no_candidate_since_=this->now();
    RCLCPP_WARN_THROTTLE(logger_,*this->get_clock(),5000,
        "GLOBAL pending: %lu frontier regions, %lu retained identities, no safe viewpoint; NOT complete",
        frontiers.size(),global_regions_.regions.size());
    if((this->now()-no_candidate_since_).seconds()>62.5) {
      stop(false, "exploration_blocked:no_safe_viewpoint");
    }
    return;
  }
  if(this->now().seconds()-global_last_gain_time_>120.0) {
    RCLCPP_WARN(logger_,"GLOBAL no mapped free-area growth >= 0.25 m2 for 120 s; pausing possible known-area loop, NOT complete");
    stop(false, "exploration_blocked:no_map_growth");return;
  }
  // Selecting an unverified candidate is not progress. Repeated planner
  // rejection must not restart the blocked deadline on every cooldown cycle.
  if(no_candidate_since_.nanoseconds()==0)no_candidate_since_=this->now();
  if((this->now()-no_candidate_since_).seconds()>62.5) {
    stop(false, "exploration_blocked:no_validated_route");
    return;
  }
  auto chosen=options.end();
  for(auto it=options.begin();it!=options.end();++it) {
    if(it->id==selected && (chosen==options.end()||it->score<chosen->score))chosen=it;
  }
  if(global_view_latched_)chosen=latched;
  if(chosen==options.end())return;
  global_view_region_=selected;
  global_view_latched_=!global_shadow_only_;
  const auto& f=frontiers[chosen->frontier];
  global_pending_staging_=chosen->view;
  pending_unknown_centroid_=f.centroid;
  pending_unknown_gain_=area[chosen->frontier];
  RCLCPP_INFO(logger_,"GLOBAL region %llu: %lu alternatives, staging (%.2f, %.2f), route %.2f m, potential area %.2f m2, step <= %.2f m",
      static_cast<unsigned long long>(selected),options.size(),chosen->view.x,chosen->view.y,
      chosen->distance,pending_unknown_gain_,max_goal_step_);
  if(global_shadow_only_)return;  // No planner/navigation action in replay.
  checkPathToFrontier(f.centroid,chosen->view,pose.position,
      yaw_from_quaternion(pose.orientation),true,false,handoff);
}

bool Explore::branchChoiceKnown(const BranchChoice& choice) const
{
  const auto same_choice = [this](
      const BranchChoice& one, const BranchChoice& two) {
    const double frontier_distance = std::hypot(
        one.frontier.x - two.frontier.x,
        one.frontier.y - two.frontier.y);
    const double anchor_distance = std::hypot(
        one.anchor.x - two.anchor.x, one.anchor.y - two.anchor.y);
    const double basin_distance = std::hypot(
        one.unknown_centroid.x - two.unknown_centroid.x,
        one.unknown_centroid.y - two.unknown_centroid.y);
    const double direction_error =
        angular_distance(one.heading, two.heading);
    return (basin_distance < unknown_basin_match_radius_ &&
            direction_error < branch_forward_angle_) ||
           (frontier_distance < branch_choice_radius_ &&
            direction_error < junction_angle_) ||
           (anchor_distance < 2.0 * branch_anchor_tolerance_ &&
            direction_error < 0.5 * junction_angle_);
  };

  if (resume_choice_active_ && same_choice(choice, resume_choice_)) {
    return true;
  }
  if (branch_active_) {
    const BranchChoice active{
        branch_anchor_, branch_frontier_, branch_unknown_centroid_,
        branch_heading_, 0.0, branch_unknown_gain_, active_branch_id_};
    if (same_choice(choice, active)) {
      return true;
    }
  }
  return std::any_of(
      branch_stack_.begin(), branch_stack_.end(),
      [&choice, &same_choice](const BranchChoice& stored) {
        return same_choice(choice, stored);
      });
}

bool Explore::branchChoiceCompleted(const BranchChoice& choice) const
{
  return std::any_of(
      completed_branches_.begin(), completed_branches_.end(),
      [this, &choice](const BranchChoice& completed) {
        const double frontier_distance = std::hypot(
            choice.frontier.x - completed.frontier.x,
            choice.frontier.y - completed.frontier.y);
        const double anchor_distance = std::hypot(
            choice.anchor.x - completed.anchor.x,
            choice.anchor.y - completed.anchor.y);
        const double basin_distance = std::hypot(
            choice.unknown_centroid.x - completed.unknown_centroid.x,
            choice.unknown_centroid.y - completed.unknown_centroid.y);
        const double direction_error =
            angular_distance(choice.heading, completed.heading);
        // A previously completed basin is reopened if mapping later reveals
        // materially more UNKNOWN space there. This prevents a temporary
        // obstacle or short scan from deleting a whole room from the DFS.
        if (choice.unknown_gain >
            completed.unknown_gain + completed_branch_reopen_gain_delta_) {
          return false;
        }
        return (basin_distance < unknown_basin_match_radius_ &&
                direction_error < branch_forward_angle_) ||
               (frontier_distance < completed_branch_radius_ &&
                direction_error < junction_angle_) ||
               (frontier_distance < 2.0 * completed_branch_radius_ &&
                anchor_distance < 2.0 * branch_anchor_tolerance_ &&
                direction_error < 0.5 * junction_angle_);
      });
}

bool Explore::candidateMatchesChoice(
    const FrontierCandidate& candidate,
    const BranchChoice& choice) const
{
  const double frontier_distance = std::hypot(
      candidate.frontier.x - choice.frontier.x,
      candidate.frontier.y - choice.frontier.y);
  const double basin_distance = std::hypot(
      candidate.unknown_centroid.x - choice.unknown_centroid.x,
      candidate.unknown_centroid.y - choice.unknown_centroid.y);
  if (!(frontier_distance < branch_choice_radius_ ||
        (basin_distance < unknown_basin_match_radius_ &&
         frontier_distance < 2.0 * branch_choice_radius_))) {
    return false;
  }

  // Spatially close frontiers can still be opposite exits of the same large
  // connected UNKNOWN component. Rebind a saved child only when the current
  // known-free route actually passes its saved junction and leaves it in the
  // saved direction. If an open-room shortcut no longer passes the anchor,
  // discard the stale memory and let root scoring rediscover the live region.
  if (candidate.topology_path.size() < 2) {
    return false;
  }
  size_t anchor_index = 0;
  double anchor_error = std::numeric_limits<double>::infinity();
  for (size_t index = 0; index < candidate.topology_path.size(); ++index) {
    const double error = std::hypot(
        candidate.topology_path[index].x - choice.anchor.x,
        candidate.topology_path[index].y - choice.anchor.y);
    if (error < anchor_error) {
      anchor_error = error;
      anchor_index = index;
    }
  }
  if (anchor_error > resume_anchor_match_radius_) {
    return false;
  }

  size_t exit_index = anchor_index;
  while (exit_index + 1 < candidate.topology_path.size() &&
         std::hypot(
             candidate.topology_path[exit_index].x -
                 candidate.topology_path[anchor_index].x,
             candidate.topology_path[exit_index].y -
                 candidate.topology_path[anchor_index].y) < 0.30) {
    ++exit_index;
  }
  if (exit_index == anchor_index) {
    return false;
  }
  const double route_heading = std::atan2(
      candidate.topology_path[exit_index].y -
          candidate.topology_path[anchor_index].y,
      candidate.topology_path[exit_index].x -
          candidate.topology_path[anchor_index].x);
  return angular_distance(route_heading, choice.heading) <=
         resume_heading_tolerance_;
}

void Explore::rememberAlternativeBranches(
    const std::vector<FrontierCandidate>& candidates,
    const FrontierCandidate& selected,
    const geometry_msgs::msg::Point& anchor)
{
  // In a cluttered open room, shortest paths toward many frontiers diverge at
  // the robot on every cycle. Treating every such pose as a new junction made
  // V10.4 grow from one to nine saved siblings while following one corridor.
  // Record a deeper macro junction only after meaningful travel. Root choices
  // are always eligible, so a centre start still captures its initial A/B/C
  // alternatives.
  if (branch_active_ && branch_memory_anchor_valid_ &&
      std::hypot(
          anchor.x - last_branch_memory_anchor_.x,
          anchor.y - last_branch_memory_anchor_.y) <
          junction_memory_min_travel_) {
    return;
  }

  struct RankedChoice
  {
    BranchChoice choice;
    double depth;
    double score;
  };
  std::vector<RankedChoice> alternatives;
  alternatives.reserve(candidates.size());

  for (const auto& candidate : candidates) {
    // A tiny frontier can still be used as the next local observation goal,
    // but it must not become a persistent DFS child. V10.3 stored map-edge
    // fragments down to 0.20 m2; those stale entries kept a completed room
    // alive and delayed return to the real root branches.
    if (candidate.unknown_gain < branch_memory_min_unknown_gain_) {
      continue;
    }

    if (std::hypot(candidate.frontier.x - selected.frontier.x,
                   candidate.frontier.y - selected.frontier.y) <
        branch_choice_radius_) {
      continue;
    }

    // While already inside a child branch, do not remember the direct parent
    // edge as if it were a new sibling. Genuine side branches remain eligible;
    // an approximately backwards path is traversed only by explicit DFS
    // backtracking to an older stack entry.
    if (branch_active_ &&
        angular_distance(candidate.heading, branch_heading_) >
            branch_forward_angle_ + junction_angle_) {
      continue;
    }

    size_t common_count = 0;
    const size_t path_count = std::min(
        candidate.topology_path.size(), selected.topology_path.size());
    while (common_count < path_count &&
           std::hypot(
               candidate.topology_path[common_count].x -
                   selected.topology_path[common_count].x,
               candidate.topology_path[common_count].y -
                   selected.topology_path[common_count].y) < 0.04) {
      ++common_count;
    }
    if (common_count == 0 ||
        common_count >= candidate.topology_path.size() ||
        common_count >= selected.topology_path.size()) {
      continue;
    }

    size_t junction_index = common_count - 1;
    geometry_msgs::msg::Point junction = anchor;
    // Store a goal-safe point on the common trunk, not an unknown frontier
    // cell. If the exact divergence is tight beside a chair/wall, step
    // backwards along the shared path until the robot footprint has clearance.
    for (size_t offset = 0; offset <= junction_index; ++offset) {
      const size_t index = junction_index - offset;
      const auto& point = candidate.topology_path[index];
      if (knownFreeWithClearance(point.x, point.y)) {
        junction = point;
        junction_index = index;
        break;
      }
    }

    const auto child_heading = [&junction](
        const std::vector<geometry_msgs::msg::Point>& path,
        size_t first_child) {
      size_t lookahead = std::min(first_child, path.size() - 1);
      while (lookahead + 1 < path.size() &&
             std::hypot(path[lookahead].x - junction.x,
                        path[lookahead].y - junction.y) < 0.25) {
        ++lookahead;
      }
      return std::atan2(
          path[lookahead].y - junction.y,
          path[lookahead].x - junction.x);
    };
    const double candidate_child_heading = child_heading(
        candidate.topology_path,
        std::min(common_count, candidate.topology_path.size() - 1));
    const double selected_child_heading = child_heading(
        selected.topology_path,
        std::min(common_count, selected.topology_path.size() - 1));
    if (angular_distance(candidate_child_heading, selected_child_heading) <
        junction_angle_) {
      continue;
    }

    BranchChoice choice{
        junction, candidate.frontier, candidate.unknown_centroid,
        candidate_child_heading, candidate.frontier_length,
        candidate.unknown_gain, next_branch_id_++};
    if (branchChoiceKnown(choice) || branchChoiceCompleted(choice)) {
      continue;
    }

    double junction_depth = 0.0;
    for (size_t index = 1; index <= junction_index; ++index) {
      junction_depth += std::hypot(
          selected.topology_path[index].x -
              selected.topology_path[index - 1].x,
          selected.topology_path[index].y -
              selected.topology_path[index - 1].y);
    }
    alternatives.push_back(RankedChoice{
        choice, junction_depth,
        candidate.distance + heading_weight_ * candidate.turn -
            frontier_size_weight_ * candidate.frontier_length -
            unknown_gain_weight_ * std::min(
                candidate.unknown_gain, unknown_gain_cap_)});
  }

  // Keep only the strongest siblings around each physical junction. Choices
  // that diverge at different trunk depths belong to different parent nodes
  // and must not compete for one global quota. Any omitted real child remains
  // a live frontier and can be rediscovered after the remembered stack drains.
  if (max_siblings_per_junction_ > 0) {
    std::stable_sort(
        alternatives.begin(), alternatives.end(),
        [](const RankedChoice& one, const RankedChoice& two) {
          if (one.score != two.score) {
            return one.score < two.score;
          }
          return one.choice.unknown_gain > two.choice.unknown_gain;
        });
    std::vector<RankedChoice> strongest;
    strongest.reserve(alternatives.size());
    for (const auto& alternative : alternatives) {
      const size_t newly_kept = static_cast<size_t>(std::count_if(
          strongest.begin(), strongest.end(),
          [this, &alternative](const RankedChoice& kept) {
            return std::hypot(
                       kept.choice.anchor.x - alternative.choice.anchor.x,
                       kept.choice.anchor.y - alternative.choice.anchor.y) <
                   junction_cluster_radius_;
          }));
      const size_t already_stored = static_cast<size_t>(std::count_if(
          branch_stack_.begin(), branch_stack_.end(),
          [this, &alternative](const BranchChoice& stored) {
            return std::hypot(
                       stored.anchor.x - alternative.choice.anchor.x,
                       stored.anchor.y - alternative.choice.anchor.y) <
                   junction_cluster_radius_;
          }));
      const size_t resuming_here =
          resume_choice_active_ &&
          std::hypot(
              resume_choice_.anchor.x - alternative.choice.anchor.x,
              resume_choice_.anchor.y - alternative.choice.anchor.y) <
              junction_cluster_radius_
              ? 1u : 0u;
      if (already_stored + resuming_here + newly_kept <
          static_cast<size_t>(max_siblings_per_junction_)) {
        strongest.push_back(alternative);
      }
    }
    alternatives.swap(strongest);
  }

  // A stack is LIFO. Shallow choices are pushed first and deep junctions last,
  // so an A2 sibling is exhausted before returning to the root B sibling.
  // Within the same junction depth, push worse alternatives first so the best
  // sibling is on top. This ordering is what makes traversal genuinely DFS,
  // rather than a flat frontier priority queue decorated with branch labels.
  std::sort(
      alternatives.begin(), alternatives.end(),
      [](const RankedChoice& one, const RankedChoice& two) {
        if (one.depth != two.depth) {
          return one.depth < two.depth;
        }
        return one.score > two.score;
      });

  size_t remembered = 0;
  for (const auto& alternative : alternatives) {
    if (static_cast<int>(branch_stack_.size()) >= max_branch_stack_size_) {
      RCLCPP_WARN_THROTTLE(
          logger_, *this->get_clock(), 10000,
          "DFS branch stack reached its limit of %d; ignoring additional "
          "low-priority branches",
          max_branch_stack_size_);
      break;
    }
    if (branchChoiceKnown(alternative.choice) ||
        branchChoiceCompleted(alternative.choice)) {
      continue;
    }
    branch_stack_.push_back(alternative.choice);
    ++remembered;
  }

  if (remembered > 0) {
    last_branch_memory_anchor_ = anchor;
    branch_memory_anchor_valid_ = true;
    RCLCPP_INFO(
        logger_,
        "DFS remembered %lu sibling branch(es) from search origin "
        "(%.2f, %.2f); stack depth is now %lu",
        remembered, anchor.x, anchor.y, branch_stack_.size());
  }
}

void Explore::completeCurrentBranch(
    const geometry_msgs::msg::Point& robot_position,
    const char* reason,
    bool remember_completed)
{
  if (branch_active_) {
    BranchChoice completed{
        branch_anchor_, branch_frontier_, branch_unknown_centroid_,
        branch_heading_, 0.0, branch_unknown_gain_, active_branch_id_};
    if (remember_completed) {
      completed_branches_.push_back(completed);
    }
    RCLCPP_INFO(
        logger_,
        "DFS branch #%llu %s at (%.2f, %.2f): %s; %lu sibling branch(es) "
        "remain",
        static_cast<unsigned long long>(active_branch_id_),
        remember_completed ? "complete" : "paused",
        robot_position.x, robot_position.y, reason, branch_stack_.size());
  }

  branch_active_ = false;
  active_branch_id_ = 0;

  while (!branch_stack_.empty()) {
    const BranchChoice next = branch_stack_.back();
    branch_stack_.pop_back();
    if (branchChoiceCompleted(next)) {
      continue;
    }
    resume_choice_ = next;
    resume_choice_active_ = true;
    backtracking_ = true;
    RCLCPP_INFO(
        logger_,
        "DFS selected remembered branch #%llu; next cycle will route "
        "directly to its live frontier (saved junction %.2f, %.2f)",
        static_cast<unsigned long long>(next.id),
        next.anchor.x, next.anchor.y);
    return;
  }

  resume_choice_active_ = false;
  backtracking_ = false;
}

double Explore::estimateUnknownGain(
    const frontier_exploration::Frontier& frontier,
    const geometry_msgs::msg::Point& observation_point,
    geometry_msgs::msg::Point& unknown_centroid)
{
  auto* costmap = costmap_client_.getCostmap();
  if (costmap == nullptr || costmap->getResolution() <= 0.0) {
    unknown_centroid = frontier.centroid;
    return 0.0;
  }

  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(costmap->getMutex()));
  const unsigned int width = costmap->getSizeInCellsX();
  const unsigned int height = costmap->getSizeInCellsY();
  const double resolution = costmap->getResolution();
  const int maximum_depth = std::max(
      1, static_cast<int>(std::ceil(unknown_gain_radius_ / resolution)));
  const auto* data = costmap->getCharMap();
  unsigned int observation_x = 0;
  unsigned int observation_y = 0;
  if (!costmap->worldToMap(
          observation_point.x, observation_point.y,
          observation_x, observation_y)) {
    unknown_centroid = frontier.centroid;
    return 0.0;
  }
  auto visible_edge = frontier.middle;
  double edge_distance = std::hypot(visible_edge.x-observation_point.x,
                                    visible_edge.y-observation_point.y);
  for (const auto& point : frontier.points) {
    const double d = std::hypot(point.x-observation_point.x, point.y-observation_point.y);
    if (d < edge_distance) { visible_edge = point; edge_distance = d; }
  }
  const double observation_heading = std::atan2(
      visible_edge.y - observation_point.y,
      visible_edge.x - observation_point.x);

  // Count only gray cells that a range sensor at the reachable white staging
  // point could actually see. A connected UNKNOWN component can wrap around
  // the outside of a closed wall and make a tiny wall gap look like a whole
  // unexplored room. The forward cone plus Bresenham occlusion test removes
  // those wall-behind/outside cells while retaining real doors and corridors.
  const auto directly_observable = [&](unsigned int target_x,
                                       unsigned int target_y) {
    double target_wx = 0.0;
    double target_wy = 0.0;
    costmap->mapToWorld(target_x, target_y, target_wx, target_wy);
    const double target_heading = std::atan2(
        target_wy - observation_point.y,
        target_wx - observation_point.x);
    if (angular_distance(target_heading, observation_heading) >
        unknown_visibility_half_angle_) {
      return false;
    }

    int x0 = static_cast<int>(observation_x);
    int y0 = static_cast<int>(observation_y);
    const int x1 = static_cast<int>(target_x);
    const int y1 = static_cast<int>(target_y);
    const int dx = std::abs(x1 - x0);
    const int sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0);
    const int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    while (x0 != x1 || y0 != y1) {
      const int twice_error = 2 * error;
      if (twice_error >= dy) {
        error += dy;
        x0 += sx;
      }
      if (twice_error <= dx) {
        error += dx;
        y0 += sy;
      }
      if (x0 == x1 && y0 == y1) {
        break;
      }
      if (x0 < 0 || y0 < 0 || x0 >= static_cast<int>(width) ||
          y0 >= static_cast<int>(height)) {
        return false;
      }
      const unsigned char ray_cost = data[
          static_cast<size_t>(y0) * width + static_cast<size_t>(x0)];
      if (ray_cost != nav2_costmap_2d::FREE_SPACE &&
          ray_cost != nav2_costmap_2d::NO_INFORMATION) {
        return false;
      }
    }
    return true;
  };

  struct CellDepth
  {
    unsigned int x;
    unsigned int y;
    int depth;
  };
  std::queue<CellDepth> queue;
  std::vector<unsigned char> visited(
      static_cast<size_t>(width) * height, 0);

  const auto seed = [&](double wx, double wy) {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap->worldToMap(wx, wy, mx, my)) {
      return;
    }
    const size_t index = static_cast<size_t>(my) * width + mx;
    if (data[index] != nav2_costmap_2d::NO_INFORMATION || visited[index]) {
      return;
    }
    visited[index] = 1;
    queue.push(CellDepth{mx, my, 0});
  };

  seed(frontier.middle.x, frontier.middle.y);
  for (const auto& point : frontier.points) {
    seed(point.x, point.y);
  }

  std::uint64_t count = 0;
  double sum_x = 0.0;
  double sum_y = 0.0;
  while (!queue.empty()) {
    const CellDepth cell = queue.front();
    queue.pop();
    double wx = 0.0;
    double wy = 0.0;
    costmap->mapToWorld(cell.x, cell.y, wx, wy);
    if (directly_observable(cell.x, cell.y)) {
      sum_x += wx;
      sum_y += wy;
      ++count;
    }

    if (cell.depth >= maximum_depth) {
      continue;
    }
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const int nx = static_cast<int>(cell.x) + dx;
        const int ny = static_cast<int>(cell.y) + dy;
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(width) ||
            ny >= static_cast<int>(height)) {
          continue;
        }
        const size_t index = static_cast<size_t>(ny) * width + nx;
        if (visited[index] ||
            data[index] != nav2_costmap_2d::NO_INFORMATION) {
          continue;
        }
        visited[index] = 1;
        queue.push(CellDepth{
            static_cast<unsigned int>(nx), static_cast<unsigned int>(ny),
            cell.depth + 1});
      }
    }
  }

  if (count == 0) {
    unknown_centroid = frontier.centroid;
    return 0.0;
  }
  unknown_centroid.x = sum_x / static_cast<double>(count);
  unknown_centroid.y = sum_y / static_cast<double>(count);
  unknown_centroid.z = 0.0;
  return static_cast<double>(count) * resolution * resolution;
}

bool Explore::findSafeStagingPoint(
    const frontier_exploration::Frontier& frontier,
    const geometry_msgs::msg::Point& robot_position,
    geometry_msgs::msg::Point& staging_point,
    std::vector<geometry_msgs::msg::Point>* alternatives)
{
  const auto navigation_costmap = navigation_costmap_;
  if (!navigation_costmap || navigation_costmap->info.resolution <= 0.0 ||
      navigation_costmap->info.width == 0 ||
      navigation_costmap->info.height == 0 ||
      navigation_costmap->header.frame_id !=
          costmap_client_.getGlobalFrameID()) {
    return false;
  }

  auto* costmap = costmap_client_.getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(costmap->getMutex()));

  unsigned int frontier_mx;
  unsigned int frontier_my;
  if (!costmap->worldToMap(
          frontier.middle.x, frontier.middle.y, frontier_mx, frontier_my)) {
    return false;
  }

  const double resolution = costmap->getResolution();
  const int search_cells =
      std::max(1, static_cast<int>(std::ceil(staging_search_radius_ /
                                            resolution)));
  const int clearance_cells =
      std::max(1, static_cast<int>(std::ceil(goal_clearance_ / resolution)));
  const int size_x = static_cast<int>(costmap->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap->getSizeInCellsY());

  double best_score = std::numeric_limits<double>::infinity();
  bool found = false;
  std::vector<std::pair<double, geometry_msgs::msg::Point>> ranked;

  // A long/curved frontier can have an unusable representative point while
  // another end opens onto a safe corridor. Cover its extent, retaining all
  // existing known-free and navigation-cost checks. Spatially thin anchors
  // and cap their count to bound work in the exploration timer.
  std::vector<geometry_msgs::msg::Point> anchors{frontier.middle};
  for (const auto& point : frontier.points) {
    const bool separated = std::all_of(anchors.begin(), anchors.end(),
        [&](const auto& a) {return std::hypot(a.x-point.x,a.y-point.y) >= 0.40;});
    if (separated) anchors.push_back(point);
    if (anchors.size() >= 24) break;
  }
  for (const auto& anchor : anchors) {
  if (!costmap->worldToMap(anchor.x, anchor.y, frontier_mx, frontier_my)) continue;
  for (int dy = -search_cells; dy <= search_cells; ++dy) {
    for (int dx = -search_cells; dx <= search_cells; ++dx) {
      if (dx * dx + dy * dy > search_cells * search_cells) {
        continue;
      }

      const int mx = static_cast<int>(frontier_mx) + dx;
      const int my = static_cast<int>(frontier_my) + dy;
      if (mx < 0 || my < 0 || mx >= size_x || my >= size_y ||
          costmap->getCost(static_cast<unsigned int>(mx),
                           static_cast<unsigned int>(my)) !=
              nav2_costmap_2d::FREE_SPACE) {
        continue;
      }

      bool clear = true;
      for (int cy = -clearance_cells;
           cy <= clearance_cells && clear; ++cy) {
        for (int cx = -clearance_cells; cx <= clearance_cells; ++cx) {
          if (cx * cx + cy * cy > clearance_cells * clearance_cells) {
            continue;
          }
          const int check_x = mx + cx;
          const int check_y = my + cy;
          if (check_x < 0 || check_y < 0 ||
              check_x >= size_x || check_y >= size_y ||
              costmap->getCost(static_cast<unsigned int>(check_x),
                               static_cast<unsigned int>(check_y)) !=
                  nav2_costmap_2d::FREE_SPACE) {
            clear = false;
            break;
          }
        }
      }
      if (!clear) {
        continue;
      }

      double wx;
      double wy;
      costmap->mapToWorld(
          static_cast<unsigned int>(mx), static_cast<unsigned int>(my),
          wx, wy);
      const int nav_cost = navigationCostAt(wx, wy);
      if (nav_cost < 0 || nav_cost > max_staging_cost_) {
        continue;
      }
      const double frontier_distance =
          std::hypot(wx - anchor.x, wy - anchor.y);
      const double robot_distance =
          std::hypot(wx - robot_position.x, wy - robot_position.y);
      // Soft preference only: retain the full circular clearance test above.
      // Probe additional room in eight directions, capped at 0.40 m, so safe
      // stopping points prefer corridor interiors over the minimum-clearance edge.
      double extra_room = 0.0;
      const int preferred_cells = static_cast<int>(std::ceil(0.40 / resolution));
      for (int radius = clearance_cells + 1; radius <= preferred_cells; ++radius) {
        bool roomy = true;
        for (int direction = 0; direction < 8; ++direction) {
          const double angle = direction * 3.14159265358979323846 / 4.0;
          const int px = mx + static_cast<int>(std::round(radius * std::cos(angle)));
          const int py = my + static_cast<int>(std::round(radius * std::sin(angle)));
          if (px < 0 || py < 0 || px >= size_x || py >= size_y ||
              costmap->getCost(px, py) != nav2_costmap_2d::FREE_SPACE) {
            roomy = false;
            break;
          }
        }
        if (!roomy) break;
        extra_room = (radius-clearance_cells)*resolution;
      }
      const double score =
          frontier_distance + 0.05 * robot_distance +
          staging_cost_weight_ * static_cast<double>(nav_cost) / 100.0 - extra_room;
      if (alternatives) {
        geometry_msgs::msg::Point p;
        p.x=wx; p.y=wy;
        ranked.emplace_back(score,p);
      }
      if (score < best_score) {
        best_score = score;
        staging_point.x = wx;
        staging_point.y = wy;
        staging_point.z = 0.0;
        found = true;
      }
    }
  }
  }

  if (alternatives) {
    alternatives->clear();
    std::stable_sort(ranked.begin(), ranked.end(),
        [](const auto& a, const auto& b) {return a.first < b.first;});
    for (const auto& item : ranked) {
      if (std::all_of(alternatives->begin(), alternatives->end(),
          [&](const auto& p) {return std::hypot(p.x-item.second.x,p.y-item.second.y)>=0.25;})) {
        alternatives->push_back(item.second);
        if (alternatives->size() >= 8) break;
      }
    }
  }
  return found;
}

bool Explore::knownFreeWithClearance(double x, double y)
{
  auto* costmap = costmap_client_.getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(costmap->getMutex()));

  unsigned int mx;
  unsigned int my;
  if (!costmap->worldToMap(x, y, mx, my) ||
      costmap->getCost(mx, my) != nav2_costmap_2d::FREE_SPACE) {
    return false;
  }

  const int clearance_cells = std::max(
      1, static_cast<int>(std::ceil(
             goal_clearance_ / costmap->getResolution())));
  const int size_x = static_cast<int>(costmap->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap->getSizeInCellsY());
  for (int dy = -clearance_cells; dy <= clearance_cells; ++dy) {
    for (int dx = -clearance_cells; dx <= clearance_cells; ++dx) {
      if (dx * dx + dy * dy > clearance_cells * clearance_cells) {
        continue;
      }
      const int check_x = static_cast<int>(mx) + dx;
      const int check_y = static_cast<int>(my) + dy;
      if (check_x < 0 || check_y < 0 || check_x >= size_x ||
          check_y >= size_y ||
          costmap->getCost(static_cast<unsigned int>(check_x),
                           static_cast<unsigned int>(check_y)) !=
              nav2_costmap_2d::FREE_SPACE) {
        return false;
      }
    }
  }
  return true;
}

bool Explore::findKnownFreePath(
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& goal,
    std::vector<geometry_msgs::msg::Point>& path)
{
  path.clear();
  auto* costmap = costmap_client_.getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(costmap->getMutex()));

  unsigned int start_x;
  unsigned int start_y;
  unsigned int goal_x;
  unsigned int goal_y;
  if (!costmap->worldToMap(start.x, start.y, start_x, start_y) ||
      !costmap->worldToMap(goal.x, goal.y, goal_x, goal_y)) {
    return false;
  }

  const unsigned int size_x = costmap->getSizeInCellsX();
  const unsigned int size_y = costmap->getSizeInCellsY();
  const unsigned int start_index = costmap->getIndex(start_x, start_y);
  const unsigned int goal_index = costmap->getIndex(goal_x, goal_y);
  const size_t cell_count = static_cast<size_t>(size_x) * size_y;
  std::vector<int> parent(cell_count, -1);
  std::queue<unsigned int> open;

  if (costmap->getCost(goal_x, goal_y) != nav2_costmap_2d::FREE_SPACE) {
    return false;
  }
  parent[start_index] = static_cast<int>(start_index);
  open.push(start_index);

  while (!open.empty() && parent[goal_index] < 0) {
    const unsigned int index = open.front();
    open.pop();
    unsigned int cell_x;
    unsigned int cell_y;
    costmap->indexToCells(index, cell_x, cell_y);

    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const int next_x = static_cast<int>(cell_x) + dx;
        const int next_y = static_cast<int>(cell_y) + dy;
        if (next_x < 0 || next_y < 0 ||
            next_x >= static_cast<int>(size_x) ||
            next_y >= static_cast<int>(size_y)) {
          continue;
        }
        if (costmap->getCost(
                static_cast<unsigned int>(next_x),
                static_cast<unsigned int>(next_y)) !=
            nav2_costmap_2d::FREE_SPACE) {
          continue;
        }
        // Do not cut diagonally through two obstacle corners. The topological
        // path is intentionally conservative; Nav2 performs the final metric
        // planning and cost checks afterwards.
        if (dx != 0 && dy != 0 &&
            (costmap->getCost(
                 static_cast<unsigned int>(static_cast<int>(cell_x) + dx),
                 cell_y) != nav2_costmap_2d::FREE_SPACE ||
             costmap->getCost(
                 cell_x,
                 static_cast<unsigned int>(static_cast<int>(cell_y) + dy)) !=
                 nav2_costmap_2d::FREE_SPACE)) {
          continue;
        }

        const unsigned int next_index = costmap->getIndex(
            static_cast<unsigned int>(next_x),
            static_cast<unsigned int>(next_y));
        if (parent[next_index] >= 0) {
          continue;
        }
        parent[next_index] = static_cast<int>(index);
        open.push(next_index);
      }
    }
  }

  if (parent[goal_index] < 0) {
    return false;
  }

  std::vector<unsigned int> reversed;
  for (unsigned int index = goal_index;;) {
    reversed.push_back(index);
    if (index == start_index) {
      break;
    }
    index = static_cast<unsigned int>(parent[index]);
  }
  path.reserve(reversed.size());
  for (auto iterator = reversed.rbegin(); iterator != reversed.rend();
       ++iterator) {
    unsigned int cell_x;
    unsigned int cell_y;
    geometry_msgs::msg::Point point;
    costmap->indexToCells(*iterator, cell_x, cell_y);
    costmap->mapToWorld(cell_x, cell_y, point.x, point.y);
    point.z = 0.0;
    path.push_back(point);
  }
  return true;
}

int Explore::navigationCostAt(double x, double y)
{
  const auto grid = navigation_costmap_;
  if (!grid || grid->info.resolution <= 0.0 || grid->info.width == 0 ||
      grid->info.height == 0 ||
      grid->header.frame_id != costmap_client_.getGlobalFrameID()) {
    return -1;
  }
  const int mx = static_cast<int>(std::floor(
      (x - grid->info.origin.position.x) / grid->info.resolution));
  const int my = static_cast<int>(std::floor(
      (y - grid->info.origin.position.y) / grid->info.resolution));
  if (mx < 0 || my < 0 || mx >= static_cast<int>(grid->info.width) ||
      my >= static_cast<int>(grid->info.height)) {
    return -1;
  }
  return static_cast<int>(grid->data[
      static_cast<size_t>(my) * grid->info.width +
      static_cast<size_t>(mx)]);
}

bool Explore::selectLocalGoal(
    const nav_msgs::msg::Path& path,
    const geometry_msgs::msg::Point& requested_goal,
    const geometry_msgs::msg::Point& robot_position,
    bool enforce_forward_progress,
    double goal_step_limit,
    geometry_msgs::msg::Point& local_goal,
    bool& reaches_frontier,
    int& maximum_cost)
{
  // Macro-branch continuity is enforced before the Nav2 request. Once Nav2
  // returns a collision-checked global path, its first segment may legitimately
  // point sideways or briefly away from the destination to pass furniture.
  // V9 compared that detour with the destination's straight-line bearing and
  // rejected correct paths even when their maximum cost was only 16.
  (void)enforce_forward_progress;
  maximum_cost = 0;
  reaches_frontier = false;
  if (path.poses.empty()) {
    return false;
  }

  // Planner output may be sparse after rounding/simplification. Sample the
  // actual polyline before clearance checks; never shortcut between bends.
  auto sampled = path;
  sampled.poses.clear();
  sampled.poses.push_back(path.poses.front());
  const double spacing = std::max(0.005, std::min(0.025,
      costmap_client_.getCostmap()->getResolution() * 0.5));
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto& a = path.poses[i-1].pose.position;
    const auto& b = path.poses[i].pose.position;
    const double length = std::hypot(b.x-a.x, b.y-a.y);
    if (!std::isfinite(length) || length / spacing > 100000) return false;
    const int steps = std::max(1, static_cast<int>(std::ceil(length / spacing)));
    for (int j = 1; j <= steps; ++j) {
      auto pose = path.poses[i];
      const double t = static_cast<double>(j) / steps;
      pose.pose.position.x = a.x + t * (b.x-a.x);
      pose.pose.position.y = a.y + t * (b.y-a.y);
      sampled.poses.push_back(pose);
    }
  }

  double path_distance = std::hypot(
      sampled.poses.front().pose.position.x - robot_position.x,
      sampled.poses.front().pose.position.y - robot_position.y);
  bool found = false;
  bool processed_entire_path = true;
  for (size_t index = 0; index < sampled.poses.size(); ++index) {
    const auto& stamped_pose = sampled.poses[index];
    if (index > 0) {
      const auto& previous = sampled.poses[index - 1].pose.position;
      path_distance += std::hypot(
          stamped_pose.pose.position.x - previous.x,
          stamped_pose.pose.position.y - previous.y);
    }

    const int cost = navigationCostAt(
        stamped_pose.pose.position.x, stamped_pose.pose.position.y);
    if (global_regions_enabled_) {
      auto* map=costmap_client_.getCostmap();
      std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(map->getMutex()));
      unsigned x,y;
      if(!map->worldToMap(stamped_pose.pose.position.x,
                         stamped_pose.pose.position.y,x,y) ||
          map->getCost(x,y)!=nav2_costmap_2d::FREE_SPACE) {
        maximum_cost=-1;processed_entire_path=false;break;
      }
    }
    if (cost < 0) {
      maximum_cost = -1;
      processed_entire_path = false;
      break;
    }
    maximum_cost = std::max(maximum_cost, cost);
    if (cost > max_path_cost_) {
      processed_entire_path = false;
      break;
    }
    if (path_distance > goal_step_limit) {
      processed_entire_path = false;
      break;
    }

    // The connector itself must end in the same kind of known-white,
    // low-inflation cell required for a real frontier staging point. Walking
    // only a bounded prefix of Nav2's path means a distant or partly blocked
    // frontier can never command a long cross-room motion.
    if (path_distance >= min_goal_distance_ &&
        cost <= max_staging_cost_ &&
        knownFreeWithClearance(
            stamped_pose.pose.position.x,
            stamped_pose.pose.position.y)) {
      local_goal = stamped_pose.pose.position;
      found = true;
    }
  }

  if (!found) {
    return false;
  }

  const auto& endpoint = sampled.poses.back().pose.position;
  reaches_frontier =
      processed_entire_path &&
      std::hypot(local_goal.x - endpoint.x, local_goal.y - endpoint.y) < 0.08 &&
      std::hypot(endpoint.x - requested_goal.x,
                 endpoint.y - requested_goal.y) < 0.12;
  return true;
}

void Explore::checkPathToFrontier(
    const geometry_msgs::msg::Point& frontier_goal,
    const geometry_msgs::msg::Point& staging_goal,
    const geometry_msgs::msg::Point& robot_position,
    double robot_yaw,
    bool continuing_branch,
    bool backtracking,
    bool continuous_handoff)
{
  pending_frontier_goal_ = frontier_goal;
  path_check_active_ = true;
  path_check_backtracking_ = backtracking;
  path_check_continuing_ = continuing_branch;
  path_check_continuous_handoff_ = continuous_handoff;
  path_check_started_ = this->now();
  const std::uint64_t request_id = ++path_request_id_;

  auto goal = nav2_msgs::action::ComputePathToPose::Goal();
  goal.goal.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.goal.header.stamp = this->now();
  goal.goal.pose.position = staging_goal;
  // Bound the planning request itself, not only the returned global route.
  // Otherwise a distant goal can produce an unknown-space shortcut whose
  // prefix is rejected even though the known-free detour is connected.
  std::vector<geometry_msgs::msg::Point> known_route;
  if (findKnownFreePath(robot_position, staging_goal, known_route)) {
    double distance = 0.0;
    bool have_step = false;
    geometry_msgs::msg::Point step;
    for (size_t i = 1; i < known_route.size(); ++i) {
      distance += std::hypot(known_route[i].x - known_route[i - 1].x,
                             known_route[i].y - known_route[i - 1].y);
      if (distance > max_goal_step_) {
        break;
      }
      const int cost = navigationCostAt(known_route[i].x, known_route[i].y);
      if (distance >= min_goal_distance_ && cost >= 0 &&
          cost <= max_staging_cost_ &&
          knownFreeWithClearance(known_route[i].x, known_route[i].y)) {
        step = known_route[i];
        have_step = true;
      }
    }
    if (have_step && std::hypot(step.x - staging_goal.x,
                               step.y - staging_goal.y) > 0.08) {
      goal.goal.pose.position = step;
      RCLCPP_INFO(logger_,
          "Pre-segmenting known-free route: planning to (%.2f, %.2f), "
          "horizon %.2f m; retaining distant branch (%.2f, %.2f)",
          step.x, step.y, max_goal_step_, staging_goal.x, staging_goal.y);
    }
  }
  const double target_yaw = std::atan2(
      goal.goal.pose.position.y - robot_position.y,
      goal.goal.pose.position.x - robot_position.x);
  goal.goal.pose.orientation.z = std::sin(target_yaw * 0.5);
  goal.goal.pose.orientation.w = std::cos(target_yaw * 0.5);
  // The precheck and the real NavigateToPose action must use the same planner.
  // Mixing Smac GridBased here with LuckGrid in the exploration BT accepted
  // paths that the deployed controller never received unchanged.
  goal.planner_id = "LuckGrid";
  goal.use_start = false;

  auto options = rclcpp_action::Client<
      nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
  options.goal_response_callback =
      [this, request_id, frontier_goal, continuing_branch, backtracking,
       continuous_handoff](
          const PlannerGoalHandle::SharedPtr& goal_handle) {
        if (request_id != path_request_id_ || !exploring_) {
          return;
        }
        if (!goal_handle) {
          path_check_active_ = false;
          path_check_continuous_handoff_ = false;
          if (continuous_handoff) {
            last_continuous_handoff_ = this->now();
            RCLCPP_DEBUG(
                logger_, "Rolling successor precheck was rejected; keeping current goal");
          } else if (backtracking) {
            abandonResumeChoice("planner rejected junction backtrack");
          } else {
            rejectFrontier(
                frontier_goal, "planner rejected path precheck",
                continuing_branch);
          }
          return;
        }
        planner_goal_handle_ = goal_handle;
      };
  options.result_callback =
      [this, request_id, frontier_goal, staging_goal, robot_position,
       robot_yaw, continuing_branch, backtracking, continuous_handoff](
          const PlannerGoalHandle::WrappedResult& result) {
        if (request_id != path_request_id_ || !exploring_) {
          return;
        }
        path_check_active_ = false;
        path_check_continuous_handoff_ = false;
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
            !result.result || result.result->path.poses.empty()) {
          if (continuous_handoff) {
            last_continuous_handoff_ = this->now();
            RCLCPP_DEBUG(
                logger_, "No valid rolling successor path; keeping current goal");
          } else if (backtracking) {
            abandonResumeChoice("no valid path to remembered junction");
          } else {
            rejectFrontier(
                frontier_goal, "no valid Nav2 path", continuing_branch);
          }
          return;
        }

        geometry_msgs::msg::Point local_goal;
        bool reaches_frontier = false;
        int maximum_path_cost = 0;
        const int path_start_cost = navigationCostAt(
            robot_position.x, robot_position.y);
        const bool soft_escape =
            path_start_cost > max_robot_cost_ &&
            path_start_cost < hard_stop_cost_;
        const double goal_step_limit = soft_escape
            ? std::min(max_goal_step_, soft_escape_goal_step_)
            : max_goal_step_;
        if (!selectLocalGoal(
                result.result->path, staging_goal, robot_position,
                continuing_branch && !backtracking, goal_step_limit,
                local_goal,
                reaches_frontier, maximum_path_cost)) {
          if (maximum_path_cost > max_path_cost_) {
            RCLCPP_WARN(
                logger_,
                "Rejecting %s toward (%.2f, %.2f): planned prefix enters "
                "global cost %d above allowed soft-cost limit %d",
                backtracking ? "junction backtrack" : "branch",
                frontier_goal.x, frontier_goal.y,
                maximum_path_cost, max_path_cost_);
          } else {
            RCLCPP_WARN(
                logger_,
                "Rejecting %s toward (%.2f, %.2f): first %.2f m of the "
                "planned path has no known-white stopping cell with %.2f m "
                "clearance and target cost <= %d (observed path cost %d)",
                backtracking ? "junction backtrack" : "branch",
                frontier_goal.x, frontier_goal.y, goal_step_limit,
                goal_clearance_, max_staging_cost_, maximum_path_cost);
          }
          if (continuous_handoff) {
            last_continuous_handoff_ = this->now();
          } else if (backtracking) {
            abandonResumeChoice("no safe connector to remembered junction");
          } else {
            rejectFrontier(
                frontier_goal, "no safe local connector on planned path",
                continuing_branch);
          }
          return;
        }

        if (continuous_handoff) {
          const double old_remaining = std::hypot(
              active_navigation_goal_.x - robot_position.x,
              active_navigation_goal_.y - robot_position.y);
          const double new_distance = std::hypot(
              local_goal.x - robot_position.x,
              local_goal.y - robot_position.y);
          if (!goal_active_ || !rollingSuccessorSafe(
              active_navigation_goal_.x - robot_position.x,
              active_navigation_goal_.y - robot_position.y,
              local_goal.x - robot_position.x, local_goal.y - robot_position.y,
              continuous_handoff_min_advance_, continuous_handoff_heading_tolerance_)) {
            last_continuous_handoff_ = this->now();
            RCLCPP_DEBUG(
                logger_,
                "Rolling successor adds only %.2f m beyond current endpoint "
                "(minimum %.2f m); keeping current goal",
                new_distance - old_remaining,
                continuous_handoff_min_advance_);
            return;
          }
        }

        double target_yaw = std::atan2(
            local_goal.y - robot_position.y,
            local_goal.x - robot_position.x);
        const auto& poses = result.result->path.poses;
        size_t local_index = 0;
        double local_error = std::numeric_limits<double>::infinity();
        for (size_t index = 0; index < poses.size(); ++index) {
          const auto& point = poses[index].pose.position;
          const double error =
              std::hypot(local_goal.x - point.x, local_goal.y - point.y);
          if (error < local_error) {
            local_error = error;
            local_index = index;
          }
        }
        if (local_index > 0) {
          const auto& previous = poses[local_index - 1].pose.position;
          target_yaw = std::atan2(
              local_goal.y - previous.y, local_goal.x - previous.x);
        }

        if (backtracking) {
          RCLCPP_INFO(
              logger_,
              "%s DFS backtrack step at known-free point (%.2f, %.2f)",
              reaches_frontier ? "Final" : "Local",
              local_goal.x, local_goal.y);
        } else if (reaches_frontier) {
          RCLCPP_INFO(
              logger_,
              "Local path reaches safe frontier staging point (%.2f, %.2f)",
              local_goal.x, local_goal.y);
        } else {
          RCLCPP_INFO(
              logger_,
              "Advancing one local branch step to known-free point "
              "(%.2f, %.2f), toward frontier (%.2f, %.2f)",
              local_goal.x, local_goal.y,
              frontier_goal.x, frontier_goal.y);
        }
        sendNavigationGoal(
            frontier_goal, local_goal, robot_position, target_yaw,
            robot_yaw, reaches_frontier, continuing_branch, backtracking,
            maximum_path_cost, soft_escape, continuous_handoff);
      };
  planner_client_->async_send_goal(goal, options);
}

void Explore::sendNavigationGoal(
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
    bool continuous_handoff)
{
  prev_goal_ = backtracking ? navigation_goal : frontier_goal;
  prev_distance_ = std::hypot(
      navigation_goal.x - robot_position.x,
      navigation_goal.y - robot_position.y);
  last_progress_pose_ = robot_position;
  last_progress_yaw_ = robot_yaw;
  last_progress_ = this->now();
  goal_reaches_frontier_ = reaches_frontier;
  active_goal_backtracking_ = backtracking;
  active_goal_soft_escape_ = soft_escape;
  active_navigation_goal_ = navigation_goal;
  active_goal_started_ = this->now();
  if (continuous_handoff) {
    last_continuous_handoff_ = this->now();
  }

  if (!backtracking) {
    if (!continuing_branch || !branch_active_) {
      branch_anchor_ = robot_position;
      branch_heading_ = target_yaw;
      active_branch_id_ = next_branch_id_++;
    } else {
      // Smooth the stored topological edge direction so ordinary corridor
      // curvature is accepted while a sudden near-180-degree reversal is not.
      branch_heading_ = std::atan2(
          0.35 * std::sin(branch_heading_) + 0.65 * std::sin(target_yaw),
          0.35 * std::cos(branch_heading_) + 0.65 * std::cos(target_yaw));
    }
    branch_frontier_ = frontier_goal;
    branch_unknown_centroid_ = pending_unknown_centroid_;
    branch_unknown_gain_ = pending_unknown_gain_;
    branch_active_ = true;
  }

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();
  goal.pose.pose.position = navigation_goal;
  goal.pose.pose.orientation.z = std::sin(target_yaw * 0.5);
  goal.pose.pose.orientation.w = std::cos(target_yaw * 0.5);
  goal.behavior_tree = behavior_tree_;

  if (visualize_) {
    visualizeSelectedGoal(navigation_goal, backtracking);
  }

  const std::uint64_t logged_branch_id =
      backtracking && resume_choice_active_
          ? resume_choice_.id
          : active_branch_id_;
  RCLCPP_INFO(
      logger_,
      "V10.7 goal locked: %s%s%s known-white goal (%.2f, %.2f), step %.2f m, "
      "target/prefix costs %d/%d, frontier (%.2f, %.2f), UNKNOWN gain %.2f m2%s, "
      "branch #%llu, stack %lu",
      backtracking ? "DFS backtrack" : "forward",
      soft_escape ? " soft-cost escape" : "",
      continuous_handoff ? " rolling-handoff" : "",
      navigation_goal.x, navigation_goal.y, prev_distance_,
      navigationCostAt(navigation_goal.x, navigation_goal.y),
      maximum_path_cost,
      frontier_goal.x, frontier_goal.y,
      backtracking ? 0.0 : pending_unknown_gain_,
      reaches_frontier ? ", final staging/anchor" : ", local connector",
      static_cast<unsigned long long>(logged_branch_id),
      branch_stack_.size());

  goal_active_ = true;
  active_goal_id_ = rclcpp_action::GoalUUID{};
  auto options = rclcpp_action::Client<
      nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  options.goal_response_callback =
      [this, frontier_goal, continuing_branch, backtracking, continuous_handoff](
          const NavigationGoalHandle::SharedPtr& goal_handle) {
        if (!exploring_) {
          return;
        }
        if (!goal_handle) {
          // A rejected successor must not leave an untracked previous motion.
          if (continuous_handoff && navigation_goal_handle_) {
            move_base_client_->async_cancel_goal(navigation_goal_handle_);
          }
          goal_active_ = false;
          if (backtracking) {
            abandonResumeChoice("navigation rejected junction backtrack");
          } else {
            rejectFrontier(
                frontier_goal, "navigation goal rejected",
                continuing_branch);
          }
          return;
        }
        no_candidate_since_=rclcpp::Time(0,0,this->get_clock()->get_clock_type());
        navigation_goal_handle_ = goal_handle;
        active_goal_id_ = goal_handle->get_goal_id();
        RCLCPP_DEBUG(
            logger_, "Goal ACCEPTED, uuid: %s",
            rclcpp_action::to_string(active_goal_id_).c_str());
      };
  options.result_callback =
      [this, frontier_goal](
          const NavigationGoalHandle::WrappedResult& result) {
        reachedGoal(result, frontier_goal);
      };
  move_base_client_->async_send_goal(goal, options);
}

void Explore::rejectFrontier(
    const geometry_msgs::msg::Point& frontier_goal, const char* reason,
    bool keep_branch)
{
  if (global_regions_enabled_) {
    global_view_latched_=false;
    global_failed_views_.emplace_back(global_pending_staging_,
        this->now().seconds() + retry_cooldown_);
    RCLCPP_WARN(logger_, "GLOBAL region %llu viewpoint rejected: %s; trying alternatives, not completing region",
        static_cast<unsigned long long>(global_regions_.active), reason);
    path_check_active_ = false;
    goal_active_ = false;
    last_goal_completed_ = this->now();
    return;
  }
  RCLCPP_WARN(
      logger_,
      "Deferring frontier (%.2f, %.2f) for %.1f seconds: %s",
      frontier_goal.x, frontier_goal.y, retry_cooldown_, reason);
  deferFrontier(frontier_goal);
  path_check_active_ = false;
  goal_active_ = false;
  if (!keep_branch) {
    branch_active_ = false;
    active_branch_id_ = 0;
  }
  last_goal_completed_ = this->now();
  // Do not recurse through every frontier in one callback. V5 issued eleven
  // planner actions in 0.5 seconds, then stopped exploration. The regular
  // timer lets costmaps settle and tries a different local branch.
}

void Explore::abandonResumeChoice(const char* reason)
{
  RCLCPP_WARN(
      logger_,
      "Dropping remembered DFS branch #%llu: %s",
      static_cast<unsigned long long>(resume_choice_.id), reason);
  completed_branches_.push_back(resume_choice_);
  resume_choice_active_ = false;
  backtracking_ = false;
  active_goal_backtracking_ = false;
  path_check_active_ = false;
  goal_active_ = false;
  last_goal_completed_ = this->now();
}

void Explore::deferFrontier(
    const geometry_msgs::msg::Point& frontier_goal,
    double cooldown,
    double radius)
{
  if (cooldown < 0.0) {
    cooldown = retry_cooldown_;
  }
  if (radius < 0.0) {
    radius = blacklist_radius_;
  }
  const auto retry_after =
      this->now() + rclcpp::Duration::from_seconds(cooldown);
  for (auto& deferred : deferred_frontiers_) {
    if (std::hypot(frontier_goal.x - deferred.point.x,
                   frontier_goal.y - deferred.point.y) <
        std::max(radius, deferred.radius)) {
      deferred.point = frontier_goal;
      if (retry_after > deferred.retry_after) {
        deferred.retry_after = retry_after;
      }
      deferred.radius = std::max(radius, deferred.radius);
      return;
    }
  }
  deferred_frontiers_.emplace_back(frontier_goal, retry_after, radius);
}

void Explore::returnToInitialPose()
{
  RCLCPP_INFO(logger_, "Returning to initial pose.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNING_TO_ORIGIN;
  status_pub_->publish(status_msg);

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  goal.pose.pose.position = initial_pose_.position;
  goal.pose.pose.orientation = initial_pose_.orientation;
  goal.pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.pose.header.stamp = this->now();

  auto send_goal_options =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
      [this](const NavigationGoalHandle::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          auto status_msg = explore_lite_msgs::msg::ExploreStatus();
          status_msg.status = explore_lite_msgs::msg::ExploreStatus::RETURNED_TO_ORIGIN;
          status_pub_->publish(status_msg);
          RCLCPP_INFO(logger_, "Successfully returned to initial pose.");
        }
      };
  move_base_client_->async_send_goal(goal, send_goal_options);
}
bool Explore::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
{
  // Use a metric area because the centroid of the same inaccessible frontier
  // moves as SLAM fills neighbouring cells.
  for (auto& frontier_goal : frontier_blacklist_) {
    const double distance = std::hypot(
        goal.x - frontier_goal.x, goal.y - frontier_goal.y);
    if (distance < blacklist_radius_) {
      return true;
    }
  }

  const auto now = this->now();
  deferred_frontiers_.erase(
      std::remove_if(
          deferred_frontiers_.begin(), deferred_frontiers_.end(),
          [&now](const DeferredFrontier& deferred) {
            return deferred.retry_after <= now;
          }),
      deferred_frontiers_.end());
  for (const auto& deferred : deferred_frontiers_) {
    if (std::hypot(goal.x - deferred.point.x,
                   goal.y - deferred.point.y) < deferred.radius) {
      return true;
    }
  }
  return false;
}

void Explore::reachedGoal(const NavigationGoalHandle::WrappedResult& result,
                          const geometry_msgs::msg::Point& frontier_goal) {
  if (!exploring_) {
    return;
  }
  // discard stale callbacks from previously preempted goals
  if (result.goal_id != active_goal_id_) {
    return;
  }

  // If the current endpoint completed while its rolling successor was only
  // being prechecked, invalidate that stale precheck. The next timer cycle
  // will plan from the actual reached pose and fresh map.
  if (path_check_active_ && path_check_continuous_handoff_) {
    path_check_active_ = false;
    path_check_continuous_handoff_ = false;
    ++path_request_id_;
    planner_client_->async_cancel_all_goals();
  }

  const bool was_backtracking = active_goal_backtracking_;
  goal_active_ = false;
  active_goal_backtracking_ = false;
  active_goal_soft_escape_ = false;
  if (global_regions_enabled_) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      // An observation attempt is not proof of global region completion.
      // Give other viewpoints a turn while SLAM integrates this observation.
      if (goal_reaches_frontier_) {
        global_view_latched_=false;
        global_observation_progress_.reached(global_view_region_);
        global_visited_views_.push_back({global_pending_staging_,
            pending_unknown_centroid_,pending_unknown_gain_});
      }
      RCLCPP_INFO(logger_, "GLOBAL region %llu step reached; retaining region until map coverage evidence",
          static_cast<unsigned long long>(global_regions_.active));
      branch_active_=true;
    } else {
      global_view_latched_=false;
      global_failed_views_.emplace_back(global_pending_staging_,
          this->now().seconds()+stalled_retry_cooldown_);
      RCLCPP_WARN(logger_, "GLOBAL region %llu navigation attempt ended without success; region remains pending",
          static_cast<unsigned long long>(global_regions_.active));
    }
    last_progress_=this->now();last_goal_completed_=this->now();prev_distance_=0;
    return;
  }
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      if (was_backtracking) {
        if (goal_reaches_frontier_) {
          backtracking_ = false;
          RCLCPP_INFO(
              logger_,
              "Reached parent junction for remembered DFS branch #%llu",
              static_cast<unsigned long long>(resume_choice_.id));
        } else {
          backtracking_ = true;
          RCLCPP_INFO(
              logger_,
              "Reached one safe backtrack connector; continuing toward "
              "junction #%llu",
              static_cast<unsigned long long>(resume_choice_.id));
        }
        last_progress_ = this->now();
        last_goal_completed_ = this->now();
        prev_distance_ = 0.0;
        return;
      }

      if (goal_reaches_frontier_) {
        RCLCPP_INFO(
            logger_,
            "Reached frontier staging point (%.2f, %.2f); keeping its "
            "UNKNOWN basin active and allowing %.1f seconds for the boundary "
            "to advance",
            frontier_goal.x, frontier_goal.y, frontier_refresh_delay_);
        // A staging point is a viewpoint in known white space, not proof that
        // the UNKNOWN room behind it is finished. Cool down only this exact
        // snapshot; a shifted frontier in the same basin remains selectable.
        deferFrontier(frontier_goal, frontier_refresh_delay_);
      } else {
        RCLCPP_INFO(
            logger_,
            "Reached local connector step toward branch (%.2f, %.2f); "
            "recomputing another nearby white step",
            frontier_goal.x, frontier_goal.y);
      }
      branch_frontier_ = frontier_goal;
      branch_active_ = true;
      last_progress_ = this->now();
      last_goal_completed_ = this->now();
      prev_distance_ = 0;
      // Do not recursively choose another target in the same callback. The
      // regular timer provides a short stop and a fresh costmap snapshot.
      return;
    case rclcpp_action::ResultCode::ABORTED:
      if (was_backtracking) {
        abandonResumeChoice("Nav2 aborted return to parent junction");
        return;
      }
      RCLCPP_WARN(
          logger_,
          "Navigation on DFS branch (%.2f, %.2f) aborted; pausing that branch "
          "and trying its nearest remembered sibling",
          frontier_goal.x, frontier_goal.y);
      deferFrontier(
          frontier_goal, stalled_retry_cooldown_, stalled_region_radius_);
      completeCurrentBranch(
          costmap_client_.getRobotPose().position,
          "Nav2 aborted this attempt", false);
      last_goal_completed_ = this->now();
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_DEBUG(logger_, "Goal was canceled");
      // If goal canceled might be because exploration stopped from topic. Don't make new plan.
      return;
    default:
      RCLCPP_WARN(logger_, "Unknown result code from move base nav2");
      branch_active_ = false;
      last_goal_completed_ = this->now();
      return;
  }
}

void Explore::start()
{
  exploring_ = true;
  hard_cost_since_ =
      rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  RCLCPP_INFO(logger_, "Exploration started.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_STARTED;
  status_pub_->publish(status_msg);
}

void Explore::stop(bool finished_exploring, const std::string& reason)
{
  RCLCPP_INFO(logger_, "Exploration stopped.");

  exploring_ = false;
  goal_active_ = false;
  path_check_active_ = false;
  path_check_continuous_handoff_ = false;
  active_goal_soft_escape_ = false;
  active_goal_id_ = rclcpp_action::GoalUUID{};
  ++path_request_id_;
  // Only publish paused status if manually stopped (not finished exploring)
  if (!finished_exploring) {
    auto status_msg = explore_lite_msgs::msg::ExploreStatus();
    status_msg.status = reason.empty() ? explore_lite_msgs::msg::ExploreStatus::EXPLORATION_PAUSED : reason;
    status_pub_->publish(status_msg);
  }

  move_base_client_->async_cancel_all_goals();
  planner_client_->async_cancel_all_goals();
  exploring_timer_->cancel();

  if (return_to_init_ && finished_exploring) {
    returnToInitialPose();
  }
}

void Explore::resume()
{
  global_last_gain_time_=-1.0;
  coverage_confirmation_=CoverageConfirmation{};
  resuming_ = true;
  exploring_ = true;
  hard_cost_since_ =
      rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  no_candidate_since_ =
      rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  RCLCPP_INFO(logger_, "Exploration resuming.");
  auto status_msg = explore_lite_msgs::msg::ExploreStatus();
  status_msg.status = explore_lite_msgs::msg::ExploreStatus::EXPLORATION_IN_PROGRESS;
  status_pub_->publish(status_msg);
  // Reactivate the timer
  exploring_timer_->reset();
  // Resume immediately
  makePlan();
}

}  // namespace explore

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // ROS1 code
  /*
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  } */
  rclcpp::spin(
      std::make_shared<explore::Explore>());  // std::move(std::make_unique)?
  rclcpp::shutdown();
  return 0;
}
