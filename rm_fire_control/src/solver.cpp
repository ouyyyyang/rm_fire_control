 

//c++ system
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense> 
#include <cstdio>
//project
#include "fire_control/fire_control_node.hpp"
#include "fire_control/armor_solver.hpp"


namespace rm_fire_control
{
Solver::Solver()
{
  //input parameter
  LargeArmorWidth_ = this->declare_parameter<float>("LargeArmorWidth", 0.23);  
  SmallArmorWidth_ = this->declare_parameter<float>("SmallArmorWidth", 0.135);
  LargeArmorHeight_ = this->declare_parameter<float>("LargeArmorHeight",);
  SmallArmorHeight_ = this->declare_parameter<float>("SmallArmorHeight",);
  K_ = this->declare_parameter<float>("K", );  //弹丸参数（分小弹丸和大弹丸）
  Gravity_ = this->declare_parameter<float>("Gravity", );
  YawMotorResSpeed_ = this->declare_parameter<float>("YawMotorResSpeed", )  // 电机响应速度
  


  this->state_ = State::TRACKING_ARMOR;
  
}

rm_interfaces::msg::GimbalCmd Solver::Solve(const rm_auto_aim::msg::Target &target,
                                            const rclcpp::Time &current_time)
{
    // Get current roll, yaw and pitch of gimbal
    try
    {
      
    }

    //  predict the the position of target
    Eigen::Vector3f target_position(target.position.x, target.position.y, target.position.z);
    float target_yaw = target.yaw;
    float flying_time = GetFlyingTime(target_position);
    float dt =
        (current_time - rclcpp::Time(target.header.stamp)).seconds() + flying_time + prediction_delay_;
    target_position.x() += dt * target.velocity.x;
    target_position.y() += dt * target.velocity.y;
    target_position.z() += dt * target.velocity.z;
    target_yaw += dt * target.v_yaw;

    // Initialize gimbal_cmd
    rm_interfaces::msg::GimbalCmd gimbal_cmd;
    gimbal_cmd.header = target.header;

    // Choose the best armor to shoot
    std::vector<Poses> armor_poses = GetArmorPoses(
        target_position, target_yaw, target.radius_1, target.radius_2, target.dz, target.armors_num);
    
    int idx =
        SelectBestArmor(armor_poses, target_position, target_yaw, target.v_yaw, target.armors_num);
    if(idx != -1)
    {
      auto chosen_armor_pose = armor_poses.at(idx);
      if (chosen_armor_pose.position.norm() < 0.1) {
          throw std::runtime_error("No valid armor to shoot");
      }
      // Initialize yaw, pitch
      float yaw, pitch;
      gimbal_cmd.fire_advice = FireCtrl(cur_yaw, cur_pitch, chosen_armor_pose, id_num);
      // Initialize state
      if(target.v_yaw > max_tracking_v_yaw_)
      {
        state = TRACKING_CENTER;
      }
      // State change 转速慢就算上云台转动，转速快就直接打
      switch (state) 
      {
        case TRACKING_ARMOR: 
        {
          if (std::abs(target.v_yaw) > max_tracking_v_yaw_) 
          {
              overflow_count_++;
          } else 
          {
              overflow_count_ = 0;
          }

          if (overflow_count_ > transfer_thresh_) 
          {
              state = TRACKING_CENTER;
          }
          if (!gimbal_cmd.fire_advice) 
          {
            CalcYawAndPitch(chosen_armor_pose.position, cur_v, yaw, pitch);
            controller_delay = std::abs(yaw - cur_yaw) / YawMotorResSpeed;
            target_position.x() += controller_delay * target.velocity.x;
            target_position.y() += controller_delay * target.velocity.y;
            target_position.z() += controller_delay * target.velocity.z;
            target_yaw += controller_delay_ * target.v_yaw;
            armor_poses = GetArmorPoses(target_position,
                                        target_yaw,
                                        target.radius_1,
                                        target.radius_2,
                                        target.dz,
                                        target.armors_num);
            chosen_armor_pose = armor_poses.at(idx);
            gimbal_cmd.distance = chosen_armor_pose.position.norm();
            CalcYawAndPitch(chosen_armor_pose.position, cur_v, yaw, pitch);
            if (chosen_armor_pose.position.norm() < 0.1) {
                throw std::runtime_error("No valid armor to shoot");
            }
          }else
          {
            CalcYawAndPitch(chosen_armor_pose.position, cur_v, yaw, pitch);
            gimbal_cmd.distance = chosen_armor_pose.position.norm();
          }
          break;
        }
        case TRACKING_CENTER: 
        {
          if (std::abs(target.v_yaw) < max_tracking_v_yaw_) 
          {
            overflow_count_++;
          } else 
          {
            overflow_count_ = 0;
          }

          if (overflow_count_ > transfer_thresh_) 
          {
            state = TRACKING_ARMOR;
            overflow_count_ = 0;
          }
          gimbal_cmd.fire_advice = 1;
          gimbal_cmd.distance = target_position.norm();
          CalcYawAndPitch(target_position, cur_v, yaw, pitch);
          break;
        }
      }
    //弧度制
    gimbal_cmd.yaw = yaw;
    gimbal_cmd.pitch = pitch; 
    //  change of angle
    gimbal_cmd.diff_yaw = yaw - cur_yaw;
    gimbal_cmd.diff_pitch = pitch - cur_pitch;

    if (gimbal_cmd.fire_advice) {
      FYT_DEBUG("armor_solver", "You Need Fire!");
    }
  }
  else
  {
    throw std::runtime_error("No valid armor to shoot");
  }
  return gimbal_cmd;
}

float Solver::GetFlyingTime(const Eigen::Vector3f &p)const noexcept
{
  float distance = p.head(2).norm() + s_bias;/ YawMotorResSpeed * std::abs(v_yaw) / 2;
  return t;
}

std::vector<Poses> Solver::GetArmorPoses(const Eigen::Vector3f &target_center,
                                         const float target_yaw,
                                         const float r1,
                                         const float r2,
                                         const float dz,
                                         const size_t armors_num) const noexcept 
{
  auto armor_poses = std::vector<Poses>(armors_num);
  // Calculate the position of each armor
  bool is_current_pair = true;
  float r = 0., target_dz = 0.;
  for (size_t i = 0; i < armors_num; i++) 
  {
    float armors_poses[i].yaw = target_yaw + i * (2 * M_PI / armors_num);
    r = is_current_pair ? r1 : r2;
    target_dz =  is_current_pair ? 0 : dz;
    is_current_pair = !is_current_pair;
    armor_poses[i].position[i] =
      target_center + Eigen::Vector3f(-r * cos(armors_yaw), -r * sin(armors_yaw), target_dz);
  }
  return armor_poses;
}

int Solver::SelectBestArmor(const std::vector<Poses> &armor_poses,
                            const Eigen::Vector3f &target_center,
                            const float target_yaw,
                            const float v_yaw)const noexcept
{
  // Angle between the car's center and the X-axis
  float center_theta = atan2(target_center.y(), target_center.x());
  // Angle between the front of observed armor and the X-axis
  for(size_t i = 0; i < armors_num;i++){
    float yaw_diff = get_delta_ang_pi(armor_poses[i].yaw, center_theta);
    if(std::abs(yaw_diff) < (2 * M_PI / armor) / 2)
    {
      // Angle between this armor and next armor
      float yaw_motor_delta =
        get_delta_ang_pi(atan2(armor_poses[i].position.y(), armor_poses[i].position.x()),
                         atan2(armor_poses[i+1].position.y(), armor_poses[i+1].position.x()));

      float angle_of_advance =
        std::abs(yaw_motor_delta) / YawMotorResSpeed * std::abs(v_yaw) / 2;
      // Return the best armor
      if (sign(v_yaw) * yaw_diff < (2 * M_PI / armor) / 2 - angle_of_advance ||
            angle_of_advance > (2 * M_PI / armor) / 4) {
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
bool Solevr::FireCtrl(const float cur_yaw, const float cur_pitch, const Pose &est, const int &id_num)
{
  if(id_num == 2)
  {
    float armor_w = LargeArmorWidth_;
    float armor_h = LargeArmorHeight_;
  }
  else
  {
    float armor_w = SmallArmorWidth_;
    float armor_h = SmallArmorHeight_;
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

  float control_yaw_angle = angle_c - cur_yaw;

  //  allow angle of pitch
  float distance = est.position.head(2).norm();
  float az = est.position.z() - 0.5f * armor_h;
  float bz = est.position.z() + 0.5f * armor_h;
  float allow_fire_pitchang_min = atan2(az, distance);
  float allow_fire_pitchang_max = atan2(bz, distance);
  return (control_yaw_angle < allow_fire_yawang_max && control_yaw_angle > allow_fire_yawang_min)&&
         (cur_pitch < allow_fire_pitchang_max && cur_pitch > allow_fire_pitchang_min);
}

//calculate the suitable gimbal pose
void Solve::CalcYawAndPitch(const Eigen::Vector3f &position, const float &current_v, float &yaw, float &pitch)
{
  yaw = atan2(position.y(), position.x());
  pitch = -pitchTrajectoryCompensation(position.head(2).norm() - s_bias, position.z() - z_bias, current_v);
}

float Solve::PitchTrajectoryCompensation(const float &s, const float &z, const float &v)const noexcept
{
  float z_temp, z_actual, dz;
  float angle_pitch;
  int i = 0;
  z_temp = z;
  // iteration
  for (i = 0; i < 20; i++)
  {
    angle_pitch = atan2(z_temp, s); // rad
    z_actual = MonoDirectionalAirResistanceModel(s, v, angle_pitch);
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

float Solve::MonoDirectionalAirResistanceModel(const float &s, const float &v, const float &angle)const noexcept
{
  float t, z;
  //t为给定v与angle时的飞行时间
  t = (float)((std::exp(K_ * s) - 1) / (K_ * v * std::cos(angle)));
  //z为给定v与angle时的高度
  z = (float)(v * std::sin(angle) * t - Gravity * t * t / 2);
  printf("model %f %f\n", t, z);
  return z;
}

} //namespace rm_fire_control

