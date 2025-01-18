
//  C++ system
#include <memory>
#include <string>
#include <vector>
#include <optional> 

#include "fire_control/fire_control_node.hpp"

namespace rm_fire_control
{
FireControlNode::FireControlNode(const rclcpp::NodeOptions & options) 
:Node("fire_control",options), solver_(std::make_unique<Solver>())
{
  RCLCPP_INFO(this->get_logger(), "Starting FireControlNode!");
  
  //  target subscribe
  aim_sub_.subscribe(this, "/tracker/target", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  aim_sub_.registerCallback(std::bind(&FireControlNode::targetCallback, this, std::placeholders::_1));

  gimbal_pub_ = this->create_publisher<fire_control_interfaces::msg::GimbalCmd>(
    "fire_control/cmd_gimbal",rclcpp::SensorDataQoS());
  pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(4), std::bind(&FireControlNode::timerCallback, this));

  debug_mode_ = this->declare_parameter<bool>("debug_mode", false); 
    
}

void FireControlNode::targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg)  
{  
  latest_target_msg_ = *msg; 
}  

void FireControlNode::timerCallback()  
{  
  //  init
  fire_control_interfaces::msg::GimbalCmd control_msg;  
 
  if (!latest_target_msg_) 
  { 
    return;  
  }  

  if(latest_target_msg_.header.frame_id.empty())
  {
    control_msg.yaw_diff = 0;  
    control_msg.pitch_diff = 0;  
    control_msg.distance = -1;  
    control_msg.pitch = 0;  
    control_msg.yaw = 0;  
    control_msg.fire_advice = false;  
    gimbal_pub_->publish(control_msg); 
    return;
  }

  auto target_msg = *latest_target_msg_;  
 
  if (target_msg.tracking) {  
    try {  
      control_msg = solver_->solve(target_msg, this->now());  
    } catch (const std::runtime_error &e) {  
      RCLCPP_ERROR(this->get_logger(), "Runtime error in solver: %s", e.what());  
      control_msg.yaw_diff = 0;  
      control_msg.pitch_diff = 0;  
      control_msg.distance = -1;  
      control_msg.fire_advice = false;  
    } catch (const std::exception &e) {  
      RCLCPP_ERROR(this->get_logger(), "Exception in solver: %s", e.what());  
      control_msg.yaw_diff = 0;  
      control_msg.pitch_diff = 0;  
      control_msg.distance = -1;  
      control_msg.fire_advice = false;  
    } catch (...) {  
      RCLCPP_ERROR(this->get_logger(), "Unknown error in solver!");  
      control_msg.yaw_diff = 0;  
      control_msg.pitch_diff = 0;  
      control_msg.distance = -1;  
      control_msg.fire_advice = false;  
    }  
  } else {  
    control_msg.yaw_diff = 0;  
    control_msg.pitch_diff = 0;  
    control_msg.distance = -1;  
    control_msg.fire_advice = false;  
  }  

  gimbal_pub_->publish(control_msg);  
 
  if (debug_mode_) {  
    publishMarkers(target_msg, control_msg);  
  }  
}

}    //namespace rm_fire_control