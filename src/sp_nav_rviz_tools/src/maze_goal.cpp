#include <cmath>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
namespace sp_nav_rviz_tools {
// An explicit fixed-goal input tool. Never publishes on startup or changes robot state.
class MazeGoal : public rviz_common::Tool {
 public:
  MazeGoal(){shortcut_key_='m';}
  void onInitialize() override {
    setName("Maze Goal (14.1,14.1)");
    node_=context_->getRosNodeAbstraction().lock()->get_raw_node();
    pub_=node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose",rclcpp::QoS(1).reliable());
  }
  void activate() override {setStatus(sent_?"Goal already sent. Restart for a new trial.":"Click the red goal marker once; snaps to exact map (14.100,14.100).");}
  void deactivate() override {}
  int processMouseEvent(rviz_common::ViewportMouseEvent& event) override {
    if(!event.leftDown())return 0;
    rviz_rendering::ViewportProjectionFinder projection;
    const auto point=projection.getViewportPointProjectionOnXYPlane(event.panel->getRenderWindow(),event.x,event.y);
    if(context_->getFixedFrame()!="map" || !point.first ||
       std::hypot(point.second.x-14.1,point.second.y-14.1)>0.3) {
      setStatus("Click the red goal marker near (14.100,14.100); no goal sent.");
      return 0;
    }
    if(!sent_){
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id="map";goal.header.stamp=node_->now();
      goal.pose.position.x=14.1;goal.pose.position.y=14.1;goal.pose.orientation.w=1.;
      pub_->publish(goal);sent_=true;
      RCLCPP_INFO(node_->get_logger(),"One RViz click: exact maze goal (14.100,14.100)");
    }
    return Finished;
  }
 private:
  bool sent_{false};rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};
}
PLUGINLIB_EXPORT_CLASS(sp_nav_rviz_tools::MazeGoal,rviz_common::Tool)
