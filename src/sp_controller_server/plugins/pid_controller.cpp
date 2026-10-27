#include "pid_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace pid_controller {
void PidController::configure(const rclcpp::Node::SharedPtr& node,const std::string& name,
 const std::shared_ptr<tf2_ros::Buffer>&) {
  if(!node)throw std::invalid_argument("null controller node");
  node_=node;
  auto number=[&](const std::string& key,double fallback){
    const auto full=name+"."+key;
    const double v=node_->has_parameter(full)?node_->get_parameter(full).as_double()
                                              :node_->declare_parameter<double>(full,fallback);
    if(!std::isfinite(v))throw std::invalid_argument(full+" must be finite");
    return v;
  };
  auto text=[&](const std::string& key,const std::string& fallback){
    const auto full=name+"."+key;
    return node_->has_parameter(full)?node_->get_parameter(full).as_string()
                                     :node_->declare_parameter<std::string>(full,fallback);
  };
  base_frame_=text("base_frame_id","base_link");
  if(base_frame_.empty())throw std::invalid_argument("empty base frame");
  const std::string mode=text("controller_mode","mpc");
  if(mode!="mpc"&&mode!="pursuit")throw std::invalid_argument(name+".controller_mode must be mpc or pursuit");

  sp_tracking::Config c;  // defaults documented in tracking_core.hpp / nav_params.yaml
  c.use_mpc=(mode=="mpc");
  c.max_speed=number("max_speed",c.max_speed);
  c.lateral_accel=number("lateral_accel",c.lateral_accel);
  c.brake_decel=number("brake_decel",c.brake_decel);
  c.min_corner_speed=number("min_corner_speed",c.min_corner_speed);
  c.curvature_window=number("curvature_window",c.curvature_window);
  c.dead_time=number("dead_time",c.dead_time);
  c.lag_gain=number("lag_gain",c.lag_gain);
  c.max_acceleration=number("max_acceleration",c.max_acceleration);
  const auto horizon_key=name+".mpc_horizon";
  c.mpc_horizon=static_cast<int>(node_->has_parameter(horizon_key)?node_->get_parameter(horizon_key).as_int()
                                    :node_->declare_parameter<int>(horizon_key,c.mpc_horizon));
  c.mpc_step=number("mpc_step",c.mpc_step);
  c.mpc_q_cross=number("mpc_q_cross",c.mpc_q_cross);
  c.mpc_q_along=number("mpc_q_along",c.mpc_q_along);
  c.mpc_q_vel=number("mpc_q_vel",c.mpc_q_vel);
  c.mpc_r_delta=number("mpc_r_delta",c.mpc_r_delta);
  c.mpc_r_ref=number("mpc_r_ref",c.mpc_r_ref);
  c.lookahead=number("lookahead",c.lookahead);
  c.lookahead_time=number("lookahead_time",c.lookahead_time);
  c.position_gain=number("position_gain",c.position_gain);
  c.lateral_damping=number("lateral_damping",c.lateral_damping);
  c.velocity_kp=number("velocity_kp",c.velocity_kp);
  c.velocity_ki=number("velocity_ki",c.velocity_ki);
  c.velocity_integral_limit=number("velocity_integral_limit",c.velocity_integral_limit);
  c.goal_radius=number("goal_radius",c.goal_radius);
  c.goal_gain=number("goal_gain",c.goal_gain);
  c.stop_tolerance=number("stop_tolerance",c.stop_tolerance);
  c.stop_speed=number("stop_speed",c.stop_speed);
  c.command_slew=number("command_slew",c.command_slew);
  c.max_command=number("max_command",c.max_command);
  c.plan_keep_distance=number("plan_keep_distance",c.plan_keep_distance);
  yaw_.latency=number("yaw_latency",yaw_.latency);
  c.validate();
  if(yaw_.latency<0||yaw_.latency>0.2)throw std::invalid_argument(name+".yaw_latency must be in [0, 0.2]");
  tracker_.cfg=c;configured_speed_=c.max_speed;
  prediction_pub_=node_->create_publisher<nav_msgs::msg::Path>("/controller/mpc_prediction",rclcpp::QoS(1));
  reference_pub_=node_->create_publisher<nav_msgs::msg::Path>("/controller/mpc_reference",rclcpp::QoS(1));
  RCLCPP_INFO(node_->get_logger(),"[%s] mode=%s vmax=%.2f a_lat=%.2f dead_time=%.2f horizon=%dx%.2fs",
    name.c_str(),mode.c_str(),c.max_speed,c.lateral_accel,c.dead_time,c.mpc_horizon,c.mpc_step);
}
void PidController::setPlan(const nav_msgs::msg::Path& path){
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<sp_tracking::Vec> pts;
  for(const auto& p:path.poses){
    if(!p.header.frame_id.empty()&&p.header.frame_id!=path.header.frame_id){
      tracker_.clearPlan();throw std::invalid_argument("mixed path frames");
    }
    pts.push_back({p.pose.position.x,p.pose.position.y});
  }
  plan_frame_=path.header.frame_id;tracker_.setPlan(pts);
}
void PidController::setSpeedLimit(double speed){
  std::lock_guard<std::mutex> lock(mutex_);
  if(!std::isfinite(speed)||speed<0)throw std::invalid_argument("invalid speed limit");
  tracker_.cfg.max_speed=speed>0?std::min(configured_speed_,speed):configured_speed_;
}
geometry_msgs::msg::TwistStamped PidController::computeVelocityCommands(
 const geometry_msgs::msg::PoseStamped& pose,const geometry_msgs::msg::Twist& velocity){
  std::lock_guard<std::mutex> lock(mutex_);
  geometry_msgs::msg::TwistStamped out;out.header.stamp=node_->now();out.header.frame_id=base_frame_;
  if(plan_frame_.empty()||pose.header.frame_id!=plan_frame_)return out;
  const auto& q=pose.pose.orientation;
  double qnorm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  if(!std::isfinite(qnorm)||qnorm<1e-8)return out;
  const double t=node_->now().seconds();
  // TF stamp of the pose; fall back to "now" if the stamp is unset.
  double stamp=rclcpp::Time(pose.header.stamp,node_->get_clock()->get_clock_type()).seconds();
  if(stamp<=0.0||stamp>t+0.1)stamp=t;
  yaw_.update(t,stamp,tf2::getYaw(q));
  // Server passes map-frame velocity; the tracker returns a map-frame command.
  auto v=tracker_.compute(t,{pose.pose.position.x,pose.pose.position.y},{velocity.linear.x,velocity.linear.y});
  if(++cycle_%5==0)publishPrediction(node_->now());
  // The simulator consumes base_link-frame commands and base_link (gimbal) keeps turning.
  const double yaw=yaw_.predicted();
  out.twist.linear.x=std::cos(yaw)*v.x+std::sin(yaw)*v.y;
  out.twist.linear.y=-std::sin(yaw)*v.x+std::cos(yaw)*v.y;
  return out;
}
}
namespace pid_controller {
void PidController::publishPrediction(const rclcpp::Time& stamp){
  auto fill=[&](const std::vector<sp_tracking::Vec>& pts){
    nav_msgs::msg::Path path;path.header.frame_id=plan_frame_;path.header.stamp=stamp;
    for(const auto& p:pts){
      geometry_msgs::msg::PoseStamped ps;ps.header=path.header;
      ps.pose.position.x=p.x;ps.pose.position.y=p.y;ps.pose.orientation.w=1.0;
      path.poses.push_back(ps);
    }
    return path;
  };
  if(prediction_pub_->get_subscription_count()>0)prediction_pub_->publish(fill(tracker_.lastPrediction()));
  if(reference_pub_->get_subscription_count()>0)reference_pub_->publish(fill(tracker_.lastReference()));
}
}
PLUGINLIB_EXPORT_CLASS(pid_controller::PidController,sp_controller_server::ControllerPlugin)
