

#ifndef FIRE_CONTROL__SOLVER_HPP_
#define FIRE_CONTROL__SOLVER_HPP_

// Eigen

#include <eigen3/Eigen/Dense>
//ros2
#include <rclcpp/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

// project
#include "fire_control_interfaces/msg/gimbal_cmd.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

//c++ system
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <memory>
#include <string>
#include <vector>
#include <cstdio>

namespace rm_fire_control
{
  
struct Pose
{
  Eigen::Vector3d position;
  double yaw;
};

class Solver
{
public:
  Solver(rclcpp::Node::SharedPtr node); 

  fire_control_interfaces::msg::GimbalCmd Solve(const auto_aim_interfaces::msg::Target &target_msg,
                                      const rclcpp::Time &current_time,
                                      std::shared_ptr<tf2_ros::Buffer> tf2_buffer);

  enum State { TRACKING_ARMOR = 0, TRACKING_CENTER = 1 } state_;
private:
  double GetFlyingTime(const Eigen::Vector3d &p)const noexcept;

  std::vector<Pose> GetArmorPoses(const Eigen::Vector3d &target_center,
                                  const double target_yaw,
                                  const double r1,
                                  const double r2,
                                  const double dz,
                                  const size_t armors_num) const noexcept;

  int SelectBestArmor(const std::vector<Pose> &armor_poses,
                      const Eigen::Vector3d &target_center,
                      const double target_yaw,
                      const double v_yaw,
                      const int armors_num) const noexcept;

  bool FireCtrl(const double cur_yaw, const double cur_pitch, const Pose &est, const char &id_num);

  void CalcYawAndPitch(const Eigen::Vector3d &position, double &yaw, double &pitch);

  double PitchTrajectoryCompensation(const double &s, const double &z, const double &v)const noexcept;

  double MonoDirectionalAirResistanceModel(const double &s, const double &v, const double &angle)const noexcept;

  double LargeArmorWidth_;
  double SmallArmorWidth_;
  double LargeArmorHeight_;
  double SmallArmorHeight_;
  double K_;
  double Gravity_; 
  double YawMotorResSpeed_;
  double SBias_;
  double ZBias_;
  int Transfer_Thresh_;
  int overflow_count_;
  
  double max_tracking_v_yaw_;
  double cur_yaw_;
  double cur_pitch_;
  double Cur_V_;

  rclcpp::Node::SharedPtr node_; 
};


  
}
#endif