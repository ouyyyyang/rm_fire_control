

#ifndef FIRE_CONTROL_SOLVER_HPP_
#define FIRE_CONTROL_SOLVER_HPP_


namespace rm_auto_aim
{
  class Solver
  {
  public:

  private:
    std::vector<Poses> getArmorPoses(const Eigen::Vector3d &target_center,
                                     const double yaw,
                                     const double r1,
                                     const double r2,
                                     const double d_zc,
                                     const double d_za,
                                     const size_t armors_num)
  };


  struct Poses
  {
    Eigen::Vector3d position;
    double yaw;
  };
  
}
#endif