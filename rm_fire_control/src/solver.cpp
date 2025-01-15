
#define LARGE_ARMOR_WIDTH 0.23
#define SMALL_ARMOR_WIDTH 0.135

//std
#include<cmath>

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
    Eigen::Vector3f target_position(target.position.x, target.position.y, target.position.z);
    float target_yaw = target.yaw;
    float flying_time = getFlyingTime(target_position);
    float dt =
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
    
    bool suggest_fire = false;
    if(idx != -1)
    {
      bool suggest_fire = fire_ctrl()
    }

    auto chosen_armor_pose = armor_poses.at(idx);
    if (chosen_armor_pose.position.norm() < 0.1) {
        throw std::runtime_error("No valid armor to shoot");
    }

    // Initialize yaw, pitch
    float yaw, pitch;
    // Initialize gimbal_cmd
    rm_interfaces::msg::GimbalCmd gimbal_cmd;
    gimbal_cmd.header = target.header;
    gimbal_cmd.fire_advice = fire_ctrl(cur_yaw, cur_pitch, chosen_armor_pose, id_num);
    // Initialize state
    if(target.v_yaw > max_tracking_v_yaw_)
    {
      state = TRACKING_CENTER;
    }
    // State change
    switch (state) {
        case TRACKING_ARMOR: {
            if (std::abs(target.v_yaw) > max_tracking_v_yaw_) {
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
                armor_poses = getArmorPoses(target_position,
                                                    target_yaw,
                                                    target.radius_1,
                                                    target.radius_2,
                                                    target.d_zc,
                                                    target.d_za,
                                                    target.armors_num);
                auto chosen_armor_pose = armor_poses.at(idx);
                gimbal_cmd.distance = chosen_armor_pose.position.norm();
                if (chosen_armor_pose.position.norm() < 0.1) {
                    throw std::runtime_error("No valid armor to shoot");
                }
                calcYawAndPitch(chosen_armor_pose.position, current_v, yaw, pitch);
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
            calcYawAndPitch(target_position, current_v, yaw, pitch);
            break;
    }
  }
//弧度制
  gimbal_cmd.yaw = yaw;
  gimbal_cmd.pitch = pitch;  

  if (gimbal_cmd.fire_advice) {
    FYT_DEBUG("armor_solver", "You Need Fire!");
  }
  return gimbal_cmd;
}

float SolvergetFlyingTime(const Eigen::Vector3f &p)
{
  float distance = p.head(2).norm() + s_bias;
  float angle = std::atan2(z, distance);
  float t = (float)((std::exp(k * s) - 1) / (k * v * std::cos(angle)))
  return t;
}

std::vector<Poses> Solver::getArmorPoses(const Eigen::Vector3f &target_center,
                                                       const float target_yaw,
                                                       const float r1,
                                                       const float r2,
                                                       const float d_zc,
                                                       const float d_za,
                                                       const size_t armors_num) const noexcept {
  auto armor_poses = std::vector<Poses>(armors_num);
  // Calculate the position of each armor
  bool is_current_pair = true;
  float r = 0., target_dz = 0.;
  for (size_t i = 0; i < armors_num; i++) {
    float armors_poses[i].yaw = target_yaw + i * (2 * M_PI / armors_num);

    r = is_current_pair ? r1 : r2;
    target_dz = d_zc + (is_current_pair ? 0 : d_za);
    is_current_pair = !is_current_pair;
    
    armor_poses[i].position[i] =
      target_center + Eigen::Vector3f(-r * cos(armors_yaw), -r * sin(armors_yaw), target_dz);
  }
  return armor_poses;
}

int Solver::selectBestArmor(const std::vector<Poses> &armor_poses,
                            const Eigen::Vector3f &target_center,
                            const float target_yaw,
                            const float v_yaw)
{
  // Angle between the car's center and the X-axis
  float center_theta = atan2(target_center.y(), target_center.x());
  // Angle between the front of observed armor and the X-axis
  for(size_t i = 0; i < armors_num;i++){
    float yaw_diff = get_delta_ang_pi(armor_poses[i].yaw, center_theta);
    if(std::abs(yaw_diff) < (2 * PI / armor) / 2)
    {
      // Angle between this armor and next armor
      float yaw_motor_delta =
        get_delta_ang_pi(atan2(armor_poses[i].position.y(), armor_poses[i].position.x()),
                         atan2(armor_poses[i+1].position.y(), armor_poses[i+1].position.x()));
      // Calculare the angle of advance
      float angle_of_advance =
        std::abs(yaw_motor_delta) / YAW_MOTOR_RES_SPEED * std::abs(v_yaw) / 2;
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

// fire_control
bool Solevr::fire_ctrl(const float cur_yaw, const float cur_pitch, const Pose &est, const int &id_num)
{
  if(id_num == 2)
  {
    float armor_w = large_armor_width;
    float armor_h = large_armor_hight;
  }
  else
  {
    float armor_w = small_armor_width;
    float armor_h = small_armor_width;
  }
  // allow angle of yaw
  float ax = est.position.x() - 0.5f * armor_w * sin(est.yaw);
  float ay = est.position.y() + 0.5f * armor_w * cos(est.yaw);
  float bx = est.position.x() + 0.5f * armor_w * sin(est.yaw);
  float by = est.position.y() - 0.5f * armor_w * cos(est.yaw);
  float angle_a = atan2(ay, ax);
  float angle_b = atan2(by, bx);
  float angle_c = atan2(est.pition.y(), est.pision.x());
  float allow_fire_yawang_max = angle_c - angle_b;  
  float allow_fire_yawang_min = angle_c - angle_a;

  float control_yaw_angle = get_delta_ang_pi(angle c, cur_yaw);

  //  allow angle of pitch
  float distance = est.position.head(2).norm();
  float az = est.position.z() - 0.5f * armor_h;
  float bz = est.position.z() + 0.5f * armor_h;
  float allow_fire_pitchang_min = atan2(az, distance);
  float allow_fire_pitchang_max = atan2(bz, distance);

  
  return (control_yaw_angle < allow_fire_yawang_max && control_yaw_angle > allow_fire_yawang_min)&&
         (cur_pitch < allow_fire_pitchang_max && cur_pitch > allow_fire_pitchang_min)
}

//calculate the suitable gimbal pose
void Solve::calcYawAndPitch(const Eigen::Vector3f &position, const float &current_v, float &yaw, float &pitch)
{
  yaw = atan2(position.y(), position.x());
  pitch = -pitchTrajectoryCompensation(position.head(2).norm() - s_bias, position.z() - z_bias, current_v);
}

float Solve::pitchTrajectoryCompensation(const float &s, const float &z, const float &v)
{
  float z_temp, z_actual, dz;
  float angle_pitch;
  int i = 0;
  z_temp = z;
  // iteration
  for (i = 0; i < 20; i++)
  {
    angle_pitch = atan2(z_temp, s); // rad
    z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
    dz = 0.3*(z - z_actual);
    z_temp = z_temp + dz;
    RCLCPP_INFO(node->get_logger(),   
    "iteration num %d: angle_pitch %f, temp target z:%f, err of z:%f, s:%f",   
    i + 1, angle_pitch * 180 / M_PI, z_temp, dz, s);  

    if (std::fabs(dz) < 0.00001)
    {
      break;
    }
  }
  return angle_pitch;
}

float Solve::monoDirectionalAirResistanceModel(float s, float v, float angle)
{
  float t, z;
  //t为给定v与angle时的飞行时间
  t = (float)((std::exp(k * s) - 1) / (k * v * std::cos(angle)));
  //z为给定v与angle时的高度
  z = (float)(v * std::sin(angle) * t - gravity * t * t / 2);
  printf("model %f %f\n", t, z);
  return z;
}

}



