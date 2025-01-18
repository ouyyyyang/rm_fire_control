
//  C++ system
#include <memory>
#include <string>
#include <vector>

#include "fire_control/fire_control_node.hpp"



namespace rm_fire_control
{
FireControlNode::FireControlNode(const rclcpp::NodeOptions & options) 
:Node("fire_control",options), solver_(nullptr)
{
    RCLCPP_INFO(this->get_logger(), "Starting FireControlNode!");

    solver_ = std::make_unique<Solver>();

    aim_sub_.subscribe(this, "/tracker/target", rclcpp::SensorDataQoS().get_rmw_qos_profile());

    gimbal_pub_ = this->create_publisher<fire_control_interfaces::msg::GimbalCmd>("fire_control/cmd_gimbal",
                                                                      rclcpp::SensorDataQoS());
    pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(4),
                                         std::bind(&FireControlNode::timerCallback, this));
    
}

void FireControlNode::timerCallback() {


  // Init message
  fire_control_interfaces::msg::GimbalCmd control_msg;

  // If target never detected
  if (aim_sub_.header.frame_id.empty()) {
    control_msg.yaw_diff = 0;
    control_msg.pitch_diff = 0;
    control_msg.distance = -1;
    control_msg.pitch = 0;
    control_msg.yaw = 0;
    control_msg.fire_advice = false;
    gimbal_pub_->publish(control_msg);
    return;
  }

  if (aim_sub_.tracking) {
    try {
      control_msg = solver_->solve(aim_sub_, this->now());
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Something went wrong in solver!");
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
    publishMarkers(aim_sub_, control_msg);
  }
}


}  //namespace rm_fire_control