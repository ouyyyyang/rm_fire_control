
#ifndef FIRE_CONTROL_NODE_HPP_
#define FIRE_CONTROL_NODE_HPP_

//  ROS
#include <message_filters/subscriber.h>

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
  explicit FireControlNode(const rclcpp::NodeOptions &options);
  
private:

  void TargetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg);

  void timerCallback();

  std::unique_ptr<Solver> solver_;

  message_filters::Subscriber<auto_aim_interfaces::msg::Target> aim_sub_;

  std::optional<auto_aim_interfaces::msg::Target> latest_target_msg_; 

  rclcpp::Publisher<fire_control_interfaces::msg::GimbalCmd>::SharedPtr gimbal_pub_;

  rclcpp::TimerBase::SharedPtr pub_timer_;

  bool debug_mode_;

}

  
}  //namespace rm_fire_control

#endif  