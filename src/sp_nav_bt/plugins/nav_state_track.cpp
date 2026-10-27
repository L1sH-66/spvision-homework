#include "nav_state_track.hpp"

#include <cmath>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "tf2/exceptions.h"

namespace sp_nav_bt
{

NavStateTrack::NavStateTrack(const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  if (!config().blackboard->template get<rclcpp::Node::SharedPtr>("node", node_)) {
    throw std::runtime_error("NavStateTrack: rclcpp::Node::SharedPtr missing in blackboard");
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList NavStateTrack::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal_pose", "Target pose (default: read from blackboard key 'goal_pose')"),
    BT::InputPort<double>(
      "goal_tolerance", 0.2, "Distance threshold to consider goal reached (metres)"),
    BT::InputPort<double>(
      "stop_speed", 0.06, "Speed below which the robot counts as stopped (m/s)"),
    BT::InputPort<double>(
      "stable_time", 0.5, "How long distance and speed must both hold (seconds)"),
    BT::OutputPort<double>("remaining_distance", "Euclidean distance to goal (metres)"),
    BT::OutputPort<double>("elapsed_time",       "Time elapsed since task started (seconds)")
  };
}

BT::NodeStatus NavStateTrack::onStart()
{
  start_time_ = std::chrono::steady_clock::now();
  stable_seconds_=0.0; last_stamp_=0;
  RCLCPP_DEBUG(node_->get_logger(), "[NavStateTrack] Task started, monitoring navigation progress.");
  RCLCPP_DEBUG(node_->get_logger(), "[NavStateTrack] onStart() entered.");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavStateTrack::onRunning()
{
  RCLCPP_DEBUG(node_->get_logger(), "[NavStateTrack] onRunning() ticked.");

  geometry_msgs::msg::PoseStamped goal_pose;
  if (!getInput<geometry_msgs::msg::PoseStamped>("goal_pose", goal_pose)) {

    if (!config().blackboard->template get<geometry_msgs::msg::PoseStamped>("goal_pose", goal_pose)) {
      RCLCPP_WARN(node_->get_logger(), "[NavStateTrack] goal_pose not available yet, RUNNING.");
      return BT::NodeStatus::RUNNING;
    }
  }

  const double elapsed_sec =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();

  double current_x = 0.0;
  double current_y = 0.0;
  int64_t stamp=0;
  try {
    auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    stamp=rclcpp::Time(transform.header.stamp).nanoseconds();
    const auto age=(node_->now()-rclcpp::Time(transform.header.stamp)).seconds();
    if(age>0.3 || age < -0.1) {stable_seconds_=0.;return BT::NodeStatus::RUNNING;}
    current_x = transform.transform.translation.x;
    current_y = transform.transform.translation.y;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(),
      "[NavStateTrack] TF lookup failed: %s. Will retry.", ex.what());

    setOutput("elapsed_time", elapsed_sec);
    config().blackboard->set<double>("elapsed_time", elapsed_sec);
    return BT::NodeStatus::RUNNING;
  }

  const double dx = goal_pose.pose.position.x - current_x;
  const double dy = goal_pose.pose.position.y - current_y;
  const double remaining = std::hypot(dx, dy);

  setOutput("remaining_distance", remaining);
  setOutput("elapsed_time",       elapsed_sec);

  config().blackboard->set<double>("remaining_distance", remaining);
  config().blackboard->set<double>("elapsed_time",       elapsed_sec);

  RCLCPP_DEBUG(node_->get_logger(),
    "[NavStateTrack] remaining=%.3f m  elapsed=%.2f s  pos=(%.2f, %.2f)",
    remaining, elapsed_sec, current_x, current_y);

  double tolerance = 0.2;
  getInput<double>("goal_tolerance", tolerance);

  RCLCPP_DEBUG(node_->get_logger(),
    "[NavStateTrack] goal=(%.2f, %.2f) current=(%.2f, %.2f) remaining=%.3f tol=%.3f",
    goal_pose.pose.position.x, goal_pose.pose.position.y,
    current_x, current_y,
    remaining, tolerance);

  if(stamp>last_stamp_) {
    const double dt=last_stamp_?double(stamp-last_stamp_)*1e-9:0.0;
    const double speed=dt>0?std::hypot(current_x-last_x_,current_y-last_y_)/dt:1e9;
    double stop_speed = 0.06;
    getInput<double>("stop_speed", stop_speed);
    if(dt>0 && dt<0.3 && remaining<tolerance && speed<stop_speed)stable_seconds_+=dt;
    else stable_seconds_=0.0;
    last_stamp_=stamp;last_x_=current_x;last_y_=current_y;
  }
  double stable_time = 0.5;
  getInput<double>("stable_time", stable_time);
  if (stable_seconds_ >= stable_time) {
    // Kept at INFO on purpose: this line is the arrival judgement (time and distance).
    RCLCPP_INFO(node_->get_logger(),
      "[NavStateTrack] Goal reached! remaining=%.3f m, elapsed=%.2f s", remaining, elapsed_sec);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_DEBUG(node_->get_logger(),
    "[NavStateTrack] Goal not reached yet, returning RUNNING.");

  return BT::NodeStatus::RUNNING;
}

void NavStateTrack::onHalted()
{
  setStatus(BT::NodeStatus::IDLE);
  RCLCPP_DEBUG(node_->get_logger(), "[NavStateTrack] Halted externally.");
}

}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sp_nav_bt::NavStateTrack>("nav_NavStateTrack");
}
