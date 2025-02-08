
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

  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  
  //  target subscribe and fliter
  target_sub_.subscribe(this, "/tracker/target", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<auto_aim_interfaces::msg::Target>>(target_sub_,
                                             *tf2_buffer_,
                                             target_frame_,
                                             10,
                                             this->get_node_logging_interface(),
                                             this->get_node_clock_interface(),
                                             std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when
  // transforms are available
  tf2_filter_->registerCallback(&FireControlNode::TargetCallback, this);

  gimbal_pub_ = this->create_publisher<fire_control_interfaces::msg::GimbalCmd>(
    "fire_control/cmd_gimbal",rclcpp::SensorDataQoS());
  pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(4), std::bind(&FireControlNode::TimerCallback, this));

  //debug_mode_ = this->declare_parameter<bool>("debug_mode", false); //之后用于发布markers
  RCLCPP_INFO(this->get_logger(), "FireControlNode initialization completed!"); 
}

void FireControlNode::TargetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg)  
{  
  RCLCPP_INFO(this->get_logger(),   
    "Target received - frame_id: %s, tracking: %d, stamp: %.3f",  
    msg->header.frame_id.c_str(),  
    msg->tracking,  
    rclcpp::Time(msg->header.stamp).seconds());  
  
  latest_target_msg_ = msg; 
}  

void FireControlNode::TimerCallback()  
{  
  //  init
  fire_control_interfaces::msg::GimbalCmd control_msg;  
 
  if (!latest_target_msg_) 
  { 
    return;  
  }  

  if (!tf2_buffer_->canTransform(  
        "odom", "camera_optical_frame",  
        tf2::TimePointZero)) {  
        RCLCPP_WARN(this->get_logger(), "Required transform is not available");  
        return;  
    }

  if(latest_target_msg_->header.frame_id.empty())
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

 
 
  if (latest_target_msg_->tracking) {  
    try {  
      control_msg = solver_->Solve(*latest_target_msg_, this->now(), tf2_buffer_);  
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