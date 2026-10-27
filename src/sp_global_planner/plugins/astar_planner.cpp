#include "sp_global_planner/astar_core.hpp"
#include "astar_planner.hpp"
#include <queue>
#include <limits>
#include <cmath>
#include <algorithm>
#include <rclcpp/exceptions.hpp>
#include <sstream>
#include <stdexcept>

namespace {

[[noreturn]] void sp_nav_param_error(const rclcpp::Node & node, const std::string & name)
{
  std::ostringstream oss;
  oss << "[参数缺失] 节点 '" << node.get_name() << "' 缺少参数 '" << name
      << "'，请在对应 yaml 的 ros__parameters 中填写。";
  throw std::runtime_error(oss.str());
}

template<typename T>
T require_param(const rclcpp::Node::SharedPtr & node, const std::string & name)
{
  try {
    if (node->has_parameter(name)) {
      return node->get_parameter(name).get_value<T>();
    }
    return node->declare_parameter<T>(name);
  } catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException &) {
    sp_nav_param_error(*node, name);
  }
}

}

namespace sp_global_planner {

void AStarPlanner::configure(const rclcpp::Node::SharedPtr& node, const std::string& plugin_name)
{
  logger_ = node->get_logger();

  lethal_cost_  = require_param<int>(node, plugin_name + ".lethal_cost");
  cost_weight_  = require_param<double>(node, plugin_name + ".cost_weight");

  // Optional smoothing parameters (defaults in astar_core.hpp).
  auto optional=[&](const std::string& key,auto fallback){
    using T=decltype(fallback);
    const auto full=plugin_name+"."+key;
    return node->has_parameter(full)?node->get_parameter(full).get_value<T>():node->declare_parameter<T>(full,fallback);
  };
  smooth_.enabled=optional("smooth_enable",smooth_.enabled);
  const double spacing_m=optional("smooth_spacing",0.10);
  smooth_.iterations=optional("smooth_iterations",smooth_.iterations);
  smooth_.data_weight=optional("smooth_data_weight",smooth_.data_weight);
  smooth_.smooth_weight=optional("smooth_weight",smooth_.smooth_weight);
  smooth_.max_cost=optional("smooth_max_cost",smooth_.max_cost);
  smooth_.curvature_iterations=optional("curvature_iterations",smooth_.curvature_iterations);
  smooth_.curvature_weight=optional("curvature_weight",smooth_.curvature_weight);
  if(!(spacing_m>0)||smooth_.iterations<0||smooth_.data_weight<0||smooth_.smooth_weight<0||
     smooth_.data_weight+2*smooth_.smooth_weight>=1.0||smooth_.max_cost<0||smooth_.max_cost>=lethal_cost_||
     smooth_.curvature_iterations<0||!(smooth_.curvature_weight>0)||smooth_.curvature_weight>=1.0/16)
    throw std::invalid_argument(plugin_name+".smooth_* parameters out of range");
  smooth_spacing_m_=spacing_m;

  RCLCPP_INFO(logger_, "AStarPlanner configured: lethal_cost=%d cost_weight=%.3f smoothing=%s (max_cost=%d)",
              lethal_cost_, cost_weight_, smooth_.enabled?"on":"off", smooth_.max_cost);
}

void AStarPlanner::setMap(const nav_msgs::msg::OccupancyGrid& costmap)
{
  map_ = costmap;
}

bool AStarPlanner::isBlocked(int8_t c) const
{

  if (c < 0) return true;
  return c >= lethal_cost_;
}

double AStarPlanner::cellCostFactor(int8_t c) const
{

  double cc = std::max<int>(0, c);
  return 1.0 + cost_weight_ * (cc / 100.0);
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal)
{
  nav_msgs::msg::Path path;
  if (!map_) {
    RCLCPP_ERROR(logger_, "No map received yet.");
    return path;
  }
  const auto& map = *map_;
  path.header = map.header;

  if (start.header.frame_id != map.header.frame_id || goal.header.frame_id != map.header.frame_id) {
    RCLCPP_WARN(logger_, "Frame mismatch: start=%s goal=%s map=%s",
      start.header.frame_id.c_str(), goal.header.frame_id.c_str(), map.header.frame_id.c_str());
  }

  GridIndex s, g;
  if (!worldToGrid(map, start.pose.position.x, start.pose.position.y, s)) {
    RCLCPP_ERROR(logger_, "Start out of map bounds.");
    return path;
  }
  if (!worldToGrid(map, goal.pose.position.x, goal.pose.position.y, g)) {
    RCLCPP_ERROR(logger_, "Goal out of map bounds.");
    return path;
  }

  auto options=smooth_;
  options.spacing=smooth_spacing_m_/map.info.resolution;
  auto points=sp_astar::plan(map.data, map.info.width, map.info.height,
    {(start.pose.position.x-map.info.origin.position.x)/map.info.resolution,
     (start.pose.position.y-map.info.origin.position.y)/map.info.resolution},
    {(goal.pose.position.x-map.info.origin.position.x)/map.info.resolution,
     (goal.pose.position.y-map.info.origin.position.y)/map.info.resolution},lethal_cost_,cost_weight_,options);
  for(size_t i=0;i<points.size();++i){
    // Orientation = path tangent (useful in RViz; the tracker only uses positions).
    const auto& a=points[i==0?0:i-1];
    const auto& b=points[i+1<points.size()?i+1:i];
    const double yaw=std::atan2(b.y-a.y,b.x-a.x);
    geometry_msgs::msg::PoseStamped ps;
    ps.header=path.header;
    ps.pose.position.x=map.info.origin.position.x+points[i].x*map.info.resolution;
    ps.pose.position.y=map.info.origin.position.y+points[i].y*map.info.resolution;
    ps.pose.orientation.z=std::sin(0.5*yaw);
    ps.pose.orientation.w=std::cos(0.5*yaw);
    path.poses.push_back(ps);
  }
  return path;
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sp_global_planner::AStarPlanner, sp_global_planner::GlobalPlannerPlugin)
