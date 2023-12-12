#ifndef FORMATION_MPC__INTERPOLATION_HPP
#define FORMATION_MPC__INTERPOLATION_HPP

#include "data_types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Core>

#include <moveit_msgs/msg/cartesian_trajectory.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace formation_mpc{
namespace utils {

class Interpolator {
public:
  Interpolator(Trajectory* plan);
  Interpolator();

  void set_plan(Trajectory* plan);
  static Interpolator from_msg(const moveit_msgs::msg::CartesianTrajectory& trj, Trajectory* plan);

  void interpolate(const rclcpp::Time& t_t,
                   const rclcpp::Time& t_start,
                   Eigen::Vector6d& interp_vel,
                   Eigen::Affine3d& out);
private:
  Trajectory* m_plan;
};

} // utils
} // formation_mpc

#endif // FORMATION_MPC__INTERPOLATION_HPP
