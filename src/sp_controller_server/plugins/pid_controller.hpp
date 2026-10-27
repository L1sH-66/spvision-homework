#pragma once
#include <mutex>
#include "sp_controller_server/controller_plugin.hpp"
#include "sp_controller_server/tracking_core.hpp"
namespace pid_controller {
// The plugin keeps the supplied class name (plugin_description.xml / launch files
// load "PidController"), but the algorithm is selected by `controller_mode`:
// "mpc" (default, linear MPC with dead-time predictor) or "pursuit" (fallback).
// All of the control logic lives in the ROS-independent sp_tracking::Tracker.
class PidController : public sp_controller_server::ControllerPlugin {
 public:
  void configure(const rclcpp::Node::SharedPtr&, const std::string&,
    const std::shared_ptr<tf2_ros::Buffer>&) override;
  void setPlan(const nav_msgs::msg::Path&) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped&,const geometry_msgs::msg::Twist&) override;
  void setSpeedLimit(double) override;
 private:
  rclcpp::Node::SharedPtr node_;
  std::string base_frame_,plan_frame_;
  std::mutex mutex_;
  sp_tracking::Tracker tracker_;
  sp_tracking::YawPredictor yaw_;
  double configured_speed_{1.42};
  // RViz: MPC predicted positions and the reference points it tracks (10 Hz).
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr prediction_pub_,reference_pub_;
  unsigned cycle_{0};
  void publishPrediction(const rclcpp::Time& stamp);
};
}
