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
  Interpolator(const Trajectory& plan);
  Interpolator();

  void set_plan(const Trajectory& plan);
  Trajectory get_plan()
  {
    return m_plan;
  }
  static Interpolator from_msg(const moveit_msgs::msg::CartesianTrajectory& trj);

  bool ready() {return is_ready;}
  void start_plan(const rclcpp::Time& t_time);
  void end_plan() {m_plan.available = false;};
  bool is_plan_started(){return m_plan.started;}
  bool is_plan_available(){return m_plan.available;}

  Interpolator clone_with_transform(const geometry_msgs::msg::TransformStamped& t_tf);

  void interpolate(const rclcpp::Time& t_t,
                   Eigen::Vector6d& interp_vel,
                   Eigen::Affine3d& out);
private:
  Trajectory m_plan;
  bool is_ready;
};

} // utils
} // formation_mpc

#endif // FORMATION_MPC__INTERPOLATION_HPP
