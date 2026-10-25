#include "luckrobot_nav2_plugins/curve_optimizer.hpp"
#include "nav2_constrained_smoother/smoother.hpp"
#include <cmath>

namespace luckrobot_nav2_plugins {
bool optimizeCurve(const nav_msgs::msg::Path & seed,
  nav_msgs::msg::Path & candidate, const nav2_costmap_2d::Costmap2D * map,
  double preferred_radius, double max_time)
{
  if (seed.poses.size()<4 || !map || preferred_radius<=0 || max_time<=0) return false;
  std::vector<Eigen::Vector3d> points;
  // Uniform-ish 10 cm optimization nodes, independent of dense publication.
  double accumulated=0;
  for(size_t i=0;i<seed.poses.size();++i) {
    const auto & p=seed.poses[i].pose.position;
    if(i) {const auto & q=seed.poses[i-1].pose.position;
      accumulated+=std::hypot(p.x-q.x,p.y-q.y);}
    if(i==0 || i+1==seed.poses.size() || accumulated>=.10) {
      if(points.empty() || std::hypot(p.x-points.back()[0],p.y-points.back()[1])>1e-6) {
        points.emplace_back(p.x,p.y,1.0); // forward direction, NOT yaw
      }
      accumulated=0;
    }
  }
  if(points.size()<4 || points.size()>300) return false;
  const Eigen::Vector2d begin=(points[1]-points[0]).head<2>().normalized();
  const Eigen::Vector2d end=(points.back()-points[points.size()-2]).head<2>().normalized();
  nav2_constrained_smoother::OptimizerParams opt;
  opt.linear_solver_type="SPARSE_NORMAL_CHOLESKY";
  opt.max_iterations=50;
  nav2_constrained_smoother::Smoother solver;
  solver.initialize(opt);
  nav2_constrained_smoother::SmootherParams p;
  p.smooth_weight=2000000.; p.costmap_weight=.015; p.curvature_weight=30.;
  p.distance_weight=0.; p.max_curvature=1./preferred_radius; p.max_time=max_time;
  p.path_downsampling_factor=1; p.path_upsampling_factor=4;
  p.reversing_enabled=false; p.keep_start_orientation=false; p.keep_goal_orientation=false;
  try {if(!solver.smooth(points,begin,end,map,p)) return false;}
  catch(const std::exception &) {return false;}
  candidate=seed; candidate.poses.clear();
  for(const auto & v:points) {
    if(!v.allFinite()) return false;
    geometry_msgs::msg::PoseStamped q; q.header=seed.header;
    q.pose.position.x=v[0];q.pose.position.y=v[1];
    q.pose.orientation.z=std::sin(v[2]/2);q.pose.orientation.w=std::cos(v[2]/2);
    candidate.poses.push_back(q);
  }
  if(candidate.poses.size()<2) return false;
  candidate.poses.front().pose.position=seed.poses.front().pose.position;
  candidate.poses.back().pose=seed.poses.back().pose;
  return true;
}
}
