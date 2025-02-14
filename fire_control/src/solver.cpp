 
#include "fire_control/solver.hpp"

//c++ system
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <memory>
#include <string>
#include <vector>
#include <cstdio>
//project



namespace rm_fire_control
{
Solver::Solver(std::weak_ptr<rclcpp::Node> n)  
: node_(n)
{
  auto node = node_.lock();

  //input parameter
  LargeArmorWidth_ = node->declare_parameter<double>("large_armor_width", 0.23);  
  SmallArmorWidth_ = node->declare_parameter<double>("small_armor_width", 0.135);
  LargeArmorHeight_ = node->declare_parameter<double>("large_armor_height", 0.127);
  SmallArmorHeight_ = node->declare_parameter<double>("small_armor_height", 0.125);
  K_ = node->declare_parameter<double>("k",0.025);  //弹丸参数（分小弹丸和大弹丸）
  Gravity_ = node->declare_parameter<double>("gravity", 9.79);
  YawMotorResSpeed_ = node->declare_parameter<double>("yaw_motor_res_speed", 1.6);  // 电机响应速度
  Transfer_Thresh_ = node->declare_parameter<int>("tranfer_thresh", 5);
  SBias_ = node->declare_parameter<double>("s_bias", 0.0);  //记得转换为秒为单位
  ZBias_ = node->declare_parameter<double>("z_bias", 0.0);
  max_tracking_v_yaw_ = node->declare_parameter<double>("max_tracking_v_yaw", 6.0);  
  Cur_V_ = node->declare_parameter<double>("current_v", 22.0);  //  m/s

  overflow_count_ = 0;
  state_ = State::TRACKING_ARMOR;
  

  node.reset();
}

fire_control_interfaces::msg::GimbalCmd Solver::Solve(const auto_aim_interfaces::msg::Target &target,
                                                      const rclcpp::Time &current_time,
                                                      std::shared_ptr<tf2_ros::Buffer> tf2_buffer)
{   
  node_shared_ = node_.lock();
  
  // Get current yaw and pitch of gimbal
  try {
    auto gimbal_tf =
      tf2_buffer->lookupTransform(target.header.frame_id, "gimbal_link", tf2::TimePointZero);
    auto msg_q = gimbal_tf.transform.rotation;

    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    double roll;
    tf2::Matrix3x3(tf_q).getRPY(roll, cur_pitch_, cur_yaw_);
    cur_pitch_ = -cur_pitch_;
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(node_shared_->get_logger(), "armor_solver: %s", ex.what());
    throw ex;
  }


  //  predict the the position of target
  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
  double target_yaw = target.yaw;
  double flying_time = GetFlyingTime(target_position);
  double dt =
      (current_time - rclcpp::Time(target.header.stamp)).seconds() + flying_time;  
  target_position.x() += dt * target.velocity.x;
  target_position.y() += dt * target.velocity.y;
  target_position.z() += dt * target.velocity.z;
  target_yaw += dt * target.v_yaw;

  // Initialize gimbal_cmd
  fire_control_interfaces::msg::GimbalCmd gimbal_cmd;
  gimbal_cmd.header = target.header;

  // Choose the best armor to shoot
  std::vector<Pose> armor_poses = GetArmorPoses(
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
    double yaw, pitch;
    gimbal_cmd.fire_advice = FireCtrl(cur_yaw_, cur_pitch_, chosen_armor_pose, target.id[0]);

    if(target.v_yaw > max_tracking_v_yaw_)
    {
      state_ = TRACKING_CENTER;
    }
    // State change 转速慢就算上云台转动，转速快就直接打
    switch (state_) 
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

        if (overflow_count_ > Transfer_Thresh_) 
        {
          state_ = TRACKING_CENTER;
        }
        if (!gimbal_cmd.fire_advice) 
        {
          CalcYawAndPitch(chosen_armor_pose.position, yaw, pitch);
          double controller_delay = std::abs(yaw - cur_yaw_) / YawMotorResSpeed_;
          target_position.x() += controller_delay * target.velocity.x;
          target_position.y() += controller_delay * target.velocity.y;
          target_position.z() += controller_delay * target.velocity.z;
          target_yaw += controller_delay * target.v_yaw;
          armor_poses = GetArmorPoses(target_position,
                                      target_yaw,
                                      target.radius_1,
                                      target.radius_2,
                                      target.dz,
                                      target.armors_num);
          chosen_armor_pose = armor_poses.at(idx);
          gimbal_cmd.distance = chosen_armor_pose.position.norm();
          CalcYawAndPitch(chosen_armor_pose.position, yaw, pitch);
          if (chosen_armor_pose.position.norm() < 0.1) 
          {  
            RCLCPP_ERROR(node_shared_->get_logger(), "No valid armor to shoot");
            node_shared_.reset();   
            return fire_control_interfaces::msg::GimbalCmd();  
          } 
        }else
        {
          CalcYawAndPitch(chosen_armor_pose.position, yaw, pitch);
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

        if (overflow_count_ > Transfer_Thresh_) 
        {
          state_ = TRACKING_ARMOR;
          overflow_count_ = 0;
        }
        gimbal_cmd.fire_advice = 1;
        gimbal_cmd.distance = target_position.norm();
        CalcYawAndPitch(target_position, yaw, pitch);
        break;
      }
    }
    //弧度制
    gimbal_cmd.yaw = yaw;
    gimbal_cmd.pitch = pitch; 
    //  change of angle
    gimbal_cmd.yaw_diff = yaw - cur_yaw_;
    gimbal_cmd.pitch_diff = - (pitch - cur_pitch_);

    if (gimbal_cmd.fire_advice) {
      RCLCPP_INFO(node_shared_->get_logger(), "You Need Fire!");
    }
  }
  else
  {
    RCLCPP_ERROR(node_shared_->get_logger(), "No valid armor to shoot"); 
    node_shared_.reset(); 
    return fire_control_interfaces::msg::GimbalCmd();
  }
  node_shared_.reset();
  return gimbal_cmd;
}

double Solver::GetFlyingTime(const Eigen::Vector3d &p)const noexcept
{
  double distance = p.head(2).norm() + SBias_;
  double angle = std::atan2(p.z(), distance);
  double t = (double)((std::exp(K_ * distance) - 1) / (K_ * Cur_V_ * std::cos(angle)));
  return t;
}

std::vector<Pose> Solver::GetArmorPoses(const Eigen::Vector3d &target_center,
                                         const double target_yaw,
                                         const double r1,
                                         const double r2,
                                         const double dz,
                                         const size_t armors_num) const noexcept 
{
  auto armors_poses = std::vector<Pose>(armors_num);
  // Calculate the position of each armor
  bool is_current_pair = true;
  double r = 0., target_dz = 0.;
  for (size_t i = 0; i < armors_num; i++) 
  {
    if(armors_num == 4)
    {
      armors_poses[i].yaw = target_yaw + i * (2 * M_PI / armors_num);
      r = is_current_pair ? r1 : r2;
      target_dz =  is_current_pair ? 0 : dz;
      is_current_pair = !is_current_pair;
    }
    else
    {
      r = r1;
      target_dz = 0;
    }
    
    armors_poses[i].position =
      target_center + Eigen::Vector3d(-r * cos(armors_poses[i].yaw), -r * sin(armors_poses[i].yaw), target_dz);
  }
  return armors_poses;
}

int Solver::SelectBestArmor(const std::vector<Pose> &armor_poses,
                            const Eigen::Vector3d &target_center,
                            const double target_yaw,
                            const double v_yaw,
                            const int armors_num)const noexcept
{
  // Angle between the car's center and the X-axis
  double center_theta = atan2(target_center.y(), target_center.x());
  // Angle between the front of observed armor and the X-axis
  for(int i = 0; i < armors_num - 1;i++)
  {
    double yaw_diff = armor_poses[i].yaw - center_theta;
    int best_armor_index = -1; 
    if(std::abs(yaw_diff) < (2 * M_PI / armors_num) / 2)
    {
      // Angle between this armor and next armor
      double yaw_motor_delta = atan2(armor_poses[i].position.y(), armor_poses[i].position.x()) - 
                              atan2(armor_poses[i+1].position.y(), armor_poses[i+1].position.x());

      double angle_of_advance =
        std::abs(yaw_motor_delta) / YawMotorResSpeed_ * std::abs(v_yaw) / 2;

      int sign = v_yaw > 0 ? 1 : -1;
      // Return the best armor
      if (sign * yaw_diff < (2 * M_PI / armors_num) / 2 - angle_of_advance ||
            angle_of_advance > (2 * M_PI / armors_num) / 4) 
      {
        best_armor_index = i;
      } else 
      {
        best_armor_index = i+1;
      }
      return best_armor_index;
    }
    else
    {
      return best_armor_index;
    }
  }
  return -1;
}      

// fire_control
bool Solver::FireCtrl(const double cur_yaw, const double cur_pitch, const Pose &est, const char &id_num)
{

  double armor_w, armor_h;
  if(id_num == '2')
  {
    armor_w = LargeArmorWidth_;
    armor_h = LargeArmorHeight_;
  }
  else
  {
    armor_w = SmallArmorWidth_;
    armor_h = SmallArmorHeight_;
  }
  // allow angle of yaw
  double ax = est.position.x() - 0.5f * armor_w * sin(est.yaw);
  double ay = est.position.y() + 0.5f * armor_w * cos(est.yaw);
  double bx = est.position.x() + 0.5f * armor_w * sin(est.yaw);
  double by = est.position.y() - 0.5f * armor_w * cos(est.yaw);
  double angle_a = atan2(ay, ax);
  double angle_b = atan2(by, bx);
  double angle_c = atan2(est.position.y(), est.position.x());
  double allow_fire_yawang_max = angle_c - angle_b;  
  double allow_fire_yawang_min = angle_c - angle_a;

  double control_yaw_angle = angle_c - cur_yaw;

  //  allow angle of pitch
  double distance = est.position.head(2).norm();
  double az = est.position.z() - 0.5f * armor_h;
  double bz = est.position.z() + 0.5f * armor_h;
  double allow_fire_pitchang_min = atan2(az, distance);
  double allow_fire_pitchang_max = atan2(bz, distance);
  return (control_yaw_angle < allow_fire_yawang_max && control_yaw_angle > allow_fire_yawang_min)&&
         (cur_pitch < allow_fire_pitchang_max && cur_pitch > allow_fire_pitchang_min);
}

//calculate the suitable gimbal pose
void Solver::CalcYawAndPitch(const Eigen::Vector3d &position, double &yaw, double &pitch)
{
  yaw = atan2(position.y(), position.x());
  pitch = PitchTrajectoryCompensation(position.head(2).norm() - SBias_, position.z() - ZBias_, Cur_V_);
}

double Solver::PitchTrajectoryCompensation(const double &s, const double &z, const double &v)const noexcept
{
  double z_temp, z_actual, dz;
  double angle_pitch;
  int i = 0;
  z_temp = z;
  // iteration
  for (i = 0; i < 20; i++)
  {
    angle_pitch = atan2(z_temp, s); // rad
    z_actual = MonoDirectionalAirResistanceModel(s, v, angle_pitch);
    dz = 0.3*(z - z_actual);
    z_temp = z_temp + dz;
    // RCLCPP_INFO(node_shared_->get_logger(),   
    //             "iteration num %d: angle_pitch %f, temp target z:%f, err of z:%f, s:%f",   
    //             i + 1, angle_pitch * 180 / M_PI, z_temp, dz, s);  

    if (std::fabs(dz) < 0.00001)
    {
      break;
    }
  }
  return angle_pitch;
}

double Solver::MonoDirectionalAirResistanceModel(const double &s, const double &v, const double &angle)const noexcept
{
  double t, z;
  //t为给定v与angle时的飞行时间
  t = (double)((std::exp(K_ * s) - 1) / (K_ * v * std::cos(angle)));
  //z为给定v与angle时的高度
  z = (double)(v * std::sin(angle) * t - Gravity_ * t * t / 2);
  // RCLCPP_INFO(node_shared_->get_logger(), "model %f %f\n", t, z);
  return z;
}

} //namespace rm_fire_control

// int main(int argc, char **argv)  
// {  
//     // 初始化 ROS 节点  
//     rclcpp::init(argc, argv);  
//     auto node = std::make_shared<rclcpp::Node>("test_node");  

//     // 创建 Solver 对象  
//     rm_fire_control::Solver solver(node);  

//     // 测试案例 1: 计算飞行时间  
//     Eigen::Vector3d target_position(10.0, 0.0, 0.0);  
//     double flying_time = solver.GetFlyingTime(target_position);  
//     std::cout << "Flying time: " << flying_time << " seconds" << std::endl;  

//     // 测试案例 2: 计算装甲板位置  
//     Eigen::Vector3d target_center(0.5, 0.0, 0.0);  
//     double target_yaw = 0.0;  
//     double r1 = 0.5, r2 = 0.3, dz = 0.1;  
//     size_t armors_num = 4;  
//     auto armor_poses = solver.GetArmorPoses(target_center, target_yaw, r1, r2, dz, armors_num);  
//     std::cout << "Armor poses:" << std::endl;  
//     for (const auto &pose : armor_poses)  
//     {  
//         std::cout << "Position: (" << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z() << "), "  
//                   << "Yaw: " << pose.yaw << std::endl;  
//     }  

//     // 测试案例 3: 选择最佳装甲板  
//     int best_armor_index = solver.SelectBestArmor(armor_poses, target_center, target_yaw, 0.0, armors_num);  
//     std::cout << "Best armor index: " << best_armor_index << std::endl;  

//     // 关闭 ROS 节点  
//     rclcpp::shutdown();  
//     return 0;  
// }node