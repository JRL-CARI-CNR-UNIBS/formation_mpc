#include "formation_utils/interpolation.hpp"

#define DEBUG
namespace formation_mpc {
namespace utils {

Interpolator::Interpolator(Trajectory* plan)
{this->set_plan(plan);}

Interpolator::Interpolator() {}

void Interpolator::set_plan(Trajectory* t_plan)
{
  m_plan = t_plan;
}

Interpolator Interpolator::from_msg(const moveit_msgs::msg::CartesianTrajectory& trj, Trajectory* plan){
  Interpolator inter;
  size_t size = trj.points.size();
  plan->clear();
  plan->resize(size);
  for(size_t idx = 0; idx < size; ++idx)
  {
    tf2::fromMsg(trj.points.at(idx).point.pose, plan->pose.at(idx));
    tf2::fromMsg(trj.points.at(idx).point.velocity, plan->twist.at(idx));
    //    tf2::fromMsg(trj.points.at(idx).point.acceleration, m_plan->acc.at(idx));
    plan->time.at(idx) = rclcpp::Time(trj.points.at(idx).time_from_start.sec,
                                       trj.points.at(idx).time_from_start.nanosec);
    #ifdef DEBUG
        std::cerr << "pos  -> " << plan->pose.at(idx).translation().transpose() << "\n" <<
                     "vel  -> " << plan->twist.at(idx).transpose() << "\n" <<
        "time -> " << plan->time.at(idx).seconds() << std::endl;
    #endif
  }
  std::cerr << "New interpolator created from msg";
  plan->started = false;
  plan->available = true;

  inter.m_plan = plan;
  return inter;
}

void Interpolator::interpolate(const rclcpp::Time& t_t,
                               const rclcpp::Time& t_start,
                               Eigen::Vector6d& interp_vel,
                               Eigen::Affine3d& out)
{
  rclcpp::Duration T = t_t - t_start;
  rclcpp::Time t_now = rclcpp::Time(T.nanoseconds());

  if(t_now > m_plan->time.back())
  {
    out = m_plan->pose.back();
    interp_vel.setZero();
    return;
  }

  int idx = std::distance(m_plan->time.begin(),
                      std::find_if(m_plan->time.begin(), m_plan->time.end(), [t_now](const auto& t_time){
                            return t_now < t_time;
                      }));
  if(idx >= m_plan->time.size())
  {
    printf("ERRORE!!!");
  }
  printf("t_now -> %f\nidx -> %d\nm_plan->time.at(idx) -> %f\nm_plan->time.at(idx-1) -> %f\n",
         t_now.seconds(), idx, m_plan->time.at(idx).seconds(),m_plan->time.at(idx-1).seconds());
  const double delta_time = (m_plan->time.at(idx) - m_plan->time.at(idx-1)).seconds();
  const double t = (t_now - m_plan->time.at(idx-1)).seconds();
  const double ratio = t / delta_time;
  printf(" [ ratio ]: %f\n", ratio);

  Eigen::Isometry3d pose_interp;
  const Eigen::Vector3d p0 = m_plan->pose.at(idx-1).translation();
  const Eigen::Vector3d p1 = m_plan->pose.at(idx).translation();
  const Eigen::Vector3d v0 = m_plan->twist.at(idx-1).head<3>();
  const Eigen::Vector3d v1 = m_plan->twist.at(idx).head<3>();

  interp_vel.head<3>() = (  3 * (2*p0 + delta_time*v0 - 2*p1 + delta_time*v1)    * std::pow(ratio,2)/delta_time
                          + 2 * (-3*p0 + 3*p1 - 2*delta_time*v0 - delta_time*v1) * ratio/delta_time
                          + v0);
  // interp_vel.head<3>() =   (6*std::pow(ratio,2)/delta_time - 6*ratio/delta_time + 1) * p0
                         // + (3*std::pow(ratio,2) - 4*ratio + 1) * v0
                         // + (-6*std::pow(ratio,2)/delta_time + 6*ratio/delta_time) * p1
                         // + (3*std::pow(ratio,2) - 2*ratio) * v1;
  // interp_vel.tail<3>() = ratio * m_plan->twist.at(idx).tail<3>() + (1-ratio) * m_plan->twist.at(idx-1).tail<3>();
  interp_vel.tail<3>() = Eigen::Vector3d::Zero(3,1);
  out.translation() =  (  2*p0 + v0 - 2*p1 + v1) * std::pow(ratio,3)
                      + (- 3*p0 + 3*p1 - 2*v0 - v1) * std::pow(ratio,2)
                      +                             v0  * ratio
                       +                             p0;
  // out.linear() = Eigen::Quaterniond(m_plan->pose.at(idx-1).linear()).slerp(ratio, Eigen::Quaterniond(m_plan->pose.at(idx).linear())).toRotationMatrix();
  out.linear() = m_plan->pose.at(0).linear();
}

}
} // formation_mpc
