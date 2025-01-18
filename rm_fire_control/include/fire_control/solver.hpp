

#ifndef FIRE_CONTROL__SOLVER_HPP_
#define FIRE_CONTROL__SOLVER_HPP_

// Eigen
#include <Eigen/Eigen>

//ros2
#include <rclcpp/time.hpp>
#include <rclcpp/rclcpp.hpp>

// project
#include "fire_control_interfaces/msg/gimbal_cmd.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "fire_control_interfaces/msg/target.hpp"

//c++ system
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <memory>
#include <string>
#include <vector>
#include <cstdio>

namespace rm_auto_aim
{
class Solver
{
public:
  Solver();
  rm_interfaces::msg::GimbalCmd solve(const rm_interfaces::msg::Target &target_msg,
                                      const rclcpp::Time &current_time,
                                      std::shared_ptr<tf2_ros::Buffer> tf2_buffer_);

  enum State { TRACKING_ARMOR = 0, TRACKING_CENTER = 1 } state;
private:
  float GetFlyingTime(const Eigen::Vector3f &p)const noexcept;

  std::vector<Poses> Solver::GetArmorPoses(const Eigen::Vector3f &target_center,
                               YawMotorResSpeed_             const float target_yaw,
                                            const float r1,
                                            const float r2,
                                            const float dz,
                                            const size_t armors_num) const noexcept;

  int Solver::SelectBestArmor(const std::vector<Poses> &armor_poses,
                              const Eigen::Vector3f &target_center,
                              const float target_yaw,
                              const float v_yaw) const noexcept;

  bool Solevr::FireCtrl(const float cur_yaw, const float cur_pitch, const Pose &est, const int &id_num);

  float Solve::PitchTrajectoryCompensation(const float &s, const float &z, const float &v)const noexcept;

  float Solve::MonoDirectionalAirResistanceModel(const float &s, const float &v, const float &angle)const noexcept;

  float LargeArmorWidth_;
  float SmallArmorWidth_;
  float LargeArmorHeight_;
  float SmallArmorHeight_;
  float K_;
  float Gravity_; 
  float YawMotorResSpeed_;
  int Transfer_Thresh_
  int overflow_count_
};

struct Poses
{
  Eigen::Vector3d position;
  double yaw;
};
  
}
#endif