
//  C++ system
#include <memory>
#include <string>
#include <vector>
#include <optional> 

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/convert.h>
#include "fire_control/fire_control_node.hpp"


namespace rm_fire_control
{
FireControlNode::FireControlNode(const rclcpp::NodeOptions & options) 
:Node("fire_control",options), solver_(std::make_unique<Solver>(shared_from_this()))
{
  RCLCPP_INFO(this->get_logger(), "Starting FireControlNode!");
  
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  
  //  target subscribe
  aim_sub_.subscribe(this, "/tracker/target", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  aim_sub_.registerCallback(std::bind(&FireControlNode::TargetCallback, this, std::placeholders::_1));

  gimbal_pub_ = this->create_publisher<fire_control_interfaces::msg::GimbalCmd>(
    "fire_control/cmd_gimbal",rclcpp::SensorDataQoS());
  pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(4), std::bind(&FireControlNode::TimerCallback, this));

  //debug_mode_ = this->declare_parameter<bool>("debug_mode", false); //之后用于发布markers
    
}

void FireControlNode::TargetCallback(const std::shared_ptr<const auto_aim_interfaces::msg::Target> msg)  
{  
  latest_target_msg_ = *msg; 
}  

void FireControlNode::TimerCallback()  
{  
  //  init
  fire_control_interfaces::msg::GimbalCmd control_msg;  
 
  if (!latest_target_msg_) 
  { 
    return;  
  }  

  auto target_msg = *latest_target_msg_; 

  if(target_msg.header.frame_id.empty())
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

 
 
  if (target_msg.tracking) {  
    try {  
      control_msg = solver_->Solve(target_msg, this->now(), tf2_buffer_);  
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
   
}

}    //namespace rm_fire_control