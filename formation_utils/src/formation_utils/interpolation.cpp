#include "formation_utils/interpolation.hpp"

#define DEBUG
namespace formation_mpc {
namespace utils {

Interpolator::Interpolator(const Trajectory& t_plan)
{
  is_ready = false;
  this->set_plan(t_plan);
}

Interpolator::Interpolator() {}

void Interpolator::set_plan(const Trajectory& t_plan)
{
  m_plan = t_plan;
  m_plan.available = true;
  is_ready = true;
}

Interpolator Interpolator::from_msg(const moveit_msgs::msg::CartesianTrajectory& t_trj){
  Trajectory plan;
  size_t size = t_trj.points.size();
  plan.clear();
  plan.resize(size);
  for(size_t idx = 0; idx < size; ++idx)
  {
    tf2::fromMsg(t_trj.points.at(idx).point.pose, plan.pose.at(idx));
    tf2::fromMsg(t_trj.points.at(idx).point.velocity, plan.twist.at(idx));
    //    tf2::fromMsg(trj.points.at(idx).point.acceleration, m_plan.acc.at(idx));
    plan.time.at(idx) = rclcpp::Time(t_trj.points.at(idx).time_from_start.sec,
                                       t_trj.points.at(idx).time_from_start.nanosec);
  }
  std::cerr << "New interpolator created from msg";
  plan.started = false;
  plan.available = true;

  Interpolator inter(plan);
  return inter;
}

void Interpolator::start_plan(const rclcpp::Time& t_time)
{
  m_plan.start = t_time;
  m_plan.started = true;
}

Interpolator Interpolator::clone_with_transform(const geometry_msgs::msg::TransformStamped &t_tf)
{
  Trajectory new_plan;
  new_plan.resize(m_plan.size());
  for(size_t idx = 0; idx < new_plan.size(); ++idx)
  {
//    tf2::doTransform(m_plan.pose.at(idx), new_plan.pose.at(idx),t_tf);
    Eigen::Isometry3d tf__eig = tf2::transformToEigen(t_tf);
    new_plan.pose.at(idx) = m_plan.pose.at(idx) * tf__eig;
    // map -> object * object -> grasp
    new_plan.twist.at(idx) = rdyn::spatialTranslation(m_plan.twist.at(idx), (new_plan.pose.at(idx).translation() - m_plan.pose.at(idx).translation()));
    new_plan.time.at(idx) = m_plan.time.at(idx);
  }
  return Interpolator(new_plan);
}

void Interpolator::interpolate(const rclcpp::Time& t_t,
                               Eigen::Vector6d& interp_vel,
                               Eigen::Affine3d& out)
{
  rclcpp::Duration T = t_t - m_plan.start;
  rclcpp::Time t_now = rclcpp::Time(T.nanoseconds());

  if(t_now > m_plan.time.back())
  {
    out = m_plan.pose.back();
    interp_vel.setZero();
    return;
  }

  int idx = std::distance(m_plan.time.begin(),
                      std::find_if(m_plan.time.begin(), m_plan.time.end(), [t_now](const auto& t_time){
                            return t_now < t_time;
                      }));
  if(idx >= m_plan.time.size())
  {
    throw std::runtime_error("Non dovresti essere qui!");
  }

  const double delta_time = (m_plan.time.at(idx) - m_plan.time.at(idx-1)).seconds();
  const double t = (t_now - m_plan.time.at(idx-1)).seconds();
  const double ratio = t / delta_time;

  Eigen::Isometry3d pose_interp;
  const Eigen::Vector3d p0 = m_plan.pose.at(idx-1).translation();
  const Eigen::Vector3d p1 = m_plan.pose.at(idx).translation();
  const Eigen::Vector3d v0 = m_plan.twist.at(idx-1).head<3>();
  const Eigen::Vector3d v1 = m_plan.twist.at(idx).head<3>();

  interp_vel.head<3>() = (  3 * (2*p0 + delta_time*v0 - 2*p1 + delta_time*v1)    * std::pow(ratio,2)/delta_time
                          + 2 * (-3*p0 + 3*p1 - 2*delta_time*v0 - delta_time*v1) * ratio/delta_time
                          + v0);
  interp_vel.tail<3>() = Eigen::Vector3d::Zero(3,1);
  out.translation() =   (  2*p0 + delta_time*v0 - 2*p1 + delta_time*v1) * std::pow(ratio,3)
                      + (- 3*p0 + 3*p1 - 2*delta_time*v0 - delta_time*v1) * std::pow(ratio,2)
                      +                              delta_time*v0  * ratio
                      +                             p0;
  // out.linear() = Eigen::Quaterniond(m_plan.pose.at(idx-1).linear()).slerp(ratio, Eigen::Quaterniond(m_plan.pose.at(idx).linear())).toRotationMatrix();
  out.linear() = m_plan.pose.at(0).linear();
}

}
} // formation_mpc
