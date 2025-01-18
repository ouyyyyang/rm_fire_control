
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

#include "fire_control/solver.hpp"
#include "auto_aim_interfaces/msg/target.hpp"


namespace rm_fire_control
{
class FireControlNode : public rclcpp::Node
{
public:
  explicit FireControlNode(const rclcpp::NodeOptions &options);
  
private:
  //  Publisher
  rclcpp::Publisher<fire_control_interfaces::msg::GimbalCmd>::SharedPtr gimbal_pub_;
  void timerCallback();

  std::unique_ptr<Solver> solver_;

  message_filters::Subscriber<auto_aim_interfaces::msg::Target> aim_sub_;

}

  
}  //namespace rm_fire_control

#endif  