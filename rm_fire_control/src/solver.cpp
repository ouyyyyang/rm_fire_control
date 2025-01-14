
#define LARGE_ARMOR_WIDTH 0.23
#define SMALL_ARMOR_WIDTH 0.135

namespace rm_auto_aim
{
Solver::Solver()
{
  

  this->state = State::TRACKING_ARMOR;
}

rm_interfaces::msg::GimbalCmd Solver::Solve(const rm_auto_aim::msg::Target &target,
                                            const rclcpp::Time &current_time,
                                            std::shared_ptr<tf2_ros::Buffer> tf2_buffer_)
{
    // Get current roll, yaw and pitch of gimbal
    try{

    }

    //  predict the the position of target
    Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
    double target_yaw = target.yaw;
    double flying_time = trajectory_compensator_->getFlyingTime(target_position);
    double dt =
        (current_time - rclcpp::Time(target.header.stamp)).seconds() + flying_time + prediction_delay_;
    target_position.x() += dt * target.velocity.x;
    target_position.y() += dt * target.velocity.y;
    target_position.z() += dt * target.velocity.z;
    target_yaw += dt * target.v_yaw;

    // Choose the best armor to shoot
    std::vector<Poses> armor_poses = getArmorPoses(
        target_position, target_yaw, target.radius_1, target.radius_2, target.d_zc, target.d_za, target.armors_num);
    int idx =
        selectBestArmor(armor_poses, target_position, target_yaw, target.v_yaw, target.armors_num);
    if(idx != -1)
    {
      bool suggest_fire = fire_ctrl()
    }
    auto chosen_armor_pose = armor_poses.at(idx);
    if (chosen_armor_position.norm() < 0.1) {
        throw std::runtime_error("No valid armor to shoot");
    }

    // Calculate yaw, pitch
    double yaw, pitch;
    

    // Initialize gimbal_cmd
    rm_interfaces::msg::GimbalCmd gimbal_cmd;
    gimbal_cmd.header = target.header;
    // Initialize state
    if(target.v_yaw > max_tracking_v_yaw_)
    {
      state = TRACKING_CENTER;
    }
    // State change
    switch (state) {
        case TRACKING_ARMOR: {
            if (std::abs(两者的主要区别target.v_yaw) > max_tracking_v_yaw_) {
                overflow_count_++;
            } else {
                overflow_count_ = 0;
            }

            if (overflow_count_ > transfer_thresh_) {
                state = TRACKING_CENTER;
            }

            // If isOnTarget() never returns true, adjust controller_delay to force the gimbal to move
            if (controller_delay_ != 0) {
                target_position.x() += controller_delay_ * target.velocity.x;
                target_position.y() += controller_delay_ * target.velocity.y;
                target_position.z() += controller_delay_ * target.velocity.z;
                target_yaw += controller_delay_ * target.v_yaw;
                armor_poses = getArmorPositions(target_position,
                                                    target_yaw,
                                                    target.radius_1,
                                                    target.radius_2,
                                                    target.d_zc,
                                                    target.d_za,
                                                    target.armors_num);
                chosen_armor_pose = armor_positions.at(idx);
                gimbal_cmd.distance = chosen_armor_pose.postrion.norm();
                if (chosen_armor_position.norm() < 0.1) {
                    throw std::runtime_error("No valid armor to shoot");
                }
                calcYawAndPitch(chosen_armor_position, rpy_, yaw, pitch);
            }
            break;
        }
        case TRACKING_CENTER: {
            if (std::abs(target.v_yaw) < max_tracking_v_yaw_) {
                overflow_count_++;
            } else {
                overflow_count_ = 0;
            }

            if (overflow_count_ > transfer_thresh_) {
                state = TRACKING_ARMOR;
                overflow_count_ = 0;
            }
            gimbal_cmd.fire_advice = true;
            calcYawAndPitch(target_position, rpy_, yaw, pitch);
            break;
    }
  }
   // Compensate angle by angle_offset_map
  auto angle_offset = manual_compensator_->angleHardCorrect(target_position.head(2).norm(), target_position.z());
  double pitch_offset = angle_offset[0] * M_PI / 180;
  double yaw_offset = angle_offset[1] * M_PI / 180;
  double cmd_pitch = pitch + pitch_offset;
  double cmd_yaw = angles::normalize_angle(yaw + yaw_offset);


  gimbal_cmd.yaw = cmd_yaw * 180 / M_PI;
  gimbal_cmd.pitch = cmd_pitch * 180 / M_PI;  
  gimbal_cmd.yaw_diff = (cmd_yaw - rpy_[2]) * 180 / M_PI;
  gimbal_cmd.pitch_diff = (cmd_pitch - rpy_[1]) * 180 / M_PI;

  if (gimbal_cmd.fire_advice) {
    FYT_DEBUG("armor_solver", "You Need Fire!");
  }
  return gimbal_cmd;
}

std::vector<Poses> Solver::getArmorPoses(const Eigen::Vector3d &target_center,
                                                       const double target_yaw,
                                                       const double r1,
                                                       const double r2,
                                                       const double d_zc,
                                                       const double d_za,
                                                       const size_t armors_num) const noexcept {
  auto armor_poses = std::vector<Poses>(armors_num);
  // Calculate the position of each armor
  bool is_current_pair = true;
  double r = 0., target_dz = 0.;
  for (size_t i = 0; i < armors_num; i++) {
    double armors_poses[i].yaw = target_yaw + i * (2 * M_PI / armors_num);

    r = is_current_pair ? r1 : r2;
    target_dz = d_zc + (is_current_pair ? 0 : d_za);
    is_current_pair = !is_current_pair;
    
    armor_poses[i].position[i] =
      target_center + Eigen::Vector3d(-r * cos(armors_yaw), -r * sin(armors_yaw), target_dz);
  }
  return armor_positions;
}

int Solver::selectBestArmor(const std::vector<Poses> &armor_poses,
                            const Eigen::Vector3d &target_center,
                            const double target_yaw,
                            const double v_yaw,
                            const size_t  )
{
  // Angle between the car's center and the X-axis
  double center_theta = std::atan2(target_center.y(), target_center.x());
  // Angle between the front of observed armor and the X-axis
  for(size_t i = 0; i < armors_num;i++){
    double yaw_diff = get_delta_ang_pi(armor_poses[i].yaw, center_theta);
    if(fabsf(yaw_diff) < (2 * PI / armor) / 2)
    {
      // Angle between this armor and next armor
      float yaw_motor_delta =
        get_delta_ang_pi(atan2(armor_poses[i].position.y(), armor_poses[i].position.x()),
                         atan2(armor_poses[i+1].position.y(), armor_poses[i+1].position.x()));
      // Calculare the angle of advance
      float angle_of_advance =
        fabsf(yaw_motor_delta) / YAW_MOTOR_RES_SPEED * fabsf(v_yaw) / 2;
      // Return the best armor
      if (sign(v_yaw) * yaw_diff < (2 * PI / armor) / 2 - angle_of_advance ||
            angle_of_advance > (2 * PI / armor) / 4) {
          return i;
        } else {
          return i+1;
        }
    }
    else
    {
      return -1;
    }
  }
}                            
bool Solevr::fire_ctrl(const double cur_yaw, const double cur_pitch, const Pose &est, &id_num)
{
  if(id_num == 2)
  {
    double armor_w = LARGE_ARMOR_WIDTH;
  }
  else
  {
    double armor_w = SMALL_ARMOR_WIDTH;
  }
  // allow angle
  float ax = est.pisition.x() - 0.5f * armor_w * sin(est.yaw);
  float ay = est.pisition.y() + 0.5f * armor_w * cos(est.yaw);
  float bx = est.pisition.x() + 0.5f * armor_w * sin(est.yaw);
  float by = est.pisition.y() - 0.5f * armor_w * cos(est.yaw);
  float angle_a = atan2(ay, ax);
  float angle_b = atan2(by, bx);
  float angle_c = atan2(est.pition.y(), est.pision.x());
  allow_fire_ang_max = angle_c - angle_b;  
  allow_fire_ang_min = angle_c - angle_a;


}


}
