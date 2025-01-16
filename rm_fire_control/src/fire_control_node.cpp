



namespace rm_fire_control
{
FireControlNode::FireControlNode(const rclcpp::NodeOptions & options) 
:node("firt_control",options)
{
    RCLCPP_INFO(this->get_logger(), "Starting FireControlNode!");
    aim_sub_.subscribe(this, "/tracker/target", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  

    gimbal_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>("armor_solver/cmd_gimbal",
                                                                      rclcpp::SensorDataQoS());
    pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(4),
                                         std::bind(&ArmorSolverNode::timerCallback, this));
    
}

void ArmorSolverNode::timerCallback() {
  if (solver_ == nullptr) {
    return;
  }

  if (!enable_) {
    return;
  }

  // Init message
  rm_interfaces::msg::GimbalCmd control_msg;

  // If target never detected
  if (armor_target_.header.frame_id.empty()) {
    control_msg.yaw_diff = 0;
    control_msg.pitch_diff = 0;
    control_msg.distance = -1;
    control_msg.pitch = 0;
    control_msg.yaw = 0;
    control_msg.fire_advice = false;
    gimbal_pub_->publish(control_msg);
    return;
  }

  if (armor_target_.tracking) {
    try {
      control_msg = solver_->solve(armor_target_, this->now(), tf2_buffer_);
    } catch (...) {
      FYT_ERROR("armor_solver", "Something went wrong in solver!");
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
    publishMarkers(armor_target_, control_msg);
  }
}


}  //namespace rm_fire_control