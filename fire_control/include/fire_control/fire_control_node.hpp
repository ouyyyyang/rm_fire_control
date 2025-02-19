
#ifndef FIRE_CONTROL_NODE_HPP_
#define FIRE_CONTROL_NODE_HPP_

//  ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/message_filter.h>  
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
// STD
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <optional> 

#include "fire_control/solver.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "fire_control_interfaces/msg/gimbal_cmd.hpp"


namespace rm_fire_control
{
class FireControlNode : public rclcpp::Node
{
public:
  FireControlNode(const rclcpp::NodeOptions &options);
  
private:

  void TargetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg);

  std::unique_ptr<Solver> solver_;
  rclcpp::Publisher<fire_control_interfaces::msg::GimbalCmd>::SharedPtr gimbal_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;  
  message_filters::Subscriber<auto_aim_interfaces::msg::Target> target_sub_;  
  std::shared_ptr<tf2_ros::MessageFilter<auto_aim_interfaces::msg::Target>> tf2_filter_;  
  std::string target_frame_; 
  //bool debug_mode_;
};

  
}  //namespace rm_fire_control

#endif  