#ifndef LEADER_MPC_HPP
#define LEADER_MPC_HPP

#include <map>
#include <vector>
#include <chrono>
#include <algorithm>
#include <iterator>
#include <mutex>



#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rcl/time.h>

//#include <rclcpp_lifecycle/state.hpp>
//#include <rclcpp_lifecycle/lifecycle_node.hpp>
//#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"

#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <task_math/task_math.h>
#include <task_math/common_limits.h>
#include <task_math/common_tasks.h>

//#include "formation_tasks.hpp"

#include <urdf_model/model.h>

#include <rdyn_core/primitives.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

// msg, srv, action
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "formation_msgs/msg/trj_optim_results.hpp"

#include "subscription_notifier/subscription_notifier.h"

#include "leader_mpc_parameters.hpp"

#ifndef FAKE_MOBILE_BASE_PATH
#define FAKE_MOBILE_BASE_PATH "../share/config/fake_base.urdf"
#endif

namespace formation_mpc
{
  using CmdType = geometry_msgs::msg::Twist;
  constexpr double k_task_level_multiplier = 1e-3;
class LeaderMPC : public controller_interface::ControllerInterface
{
public:
  LeaderMPC();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update( const rclcpp::Time & time,
                                            const rclcpp::Duration & period) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure (
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate  (
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup   (
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_error     (
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_shutdown  (
    const rclcpp_lifecycle::State & previous_state) override;
protected:


  const rclcpp::Duration k_max_delay = rclcpp::Duration::from_nanoseconds(1e8); // 100ms
  double k_e_f {0.1};

  std::mutex m_mtx;

  unsigned int m_nax;
  unsigned int m_nax_arm;
  unsigned int m_nax_base;

  Eigen::VectorXd m_q_now;

  Eigen::MatrixXd m_q;
  Eigen::MatrixXd m_dq;
  Eigen::MatrixXd m_ddq;

  Eigen::VectorXd m_target_dx;
  Eigen::Affine3d m_target_x;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> m_rt_command_ptr;
  rclcpp::Subscription<CmdType>::SharedPtr m_joints_command_subscriber;

  template <typename T>
  using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

  InterfaceReferences<hardware_interface::LoanedCommandInterface> m_joint_command_interface;
  InterfaceReferences<hardware_interface::LoanedStateInterface> m_joint_state_interface;

  std::vector<std::string> m_joint_names;

  std::vector<std::string> m_command_interface_types;

  std::string m_base_footprint  {"base_footprint"};
  std::string m_base_link       {"base_link"};
  std::string m_tool_frame      {"tool0"};
  std::string m_map_frame       {"map"};
  std::string m_namespace       {"leader"};
  std::string this_frame(const std::string& s) { return m_namespace + "/" + s;}

  const std::vector<std::string> m_allowed_interface_types = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION};

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> m_cmd_vel_pub_wrapped;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>> m_cmd_vel_pub_rt; // cmd_to_base

  rclcpp::Publisher<formation_msgs::msg::TrjOptimResults>::SharedPtr m_leader_trj_pub;
  std::vector<rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr> m_flw_trj_sub;
  std::vector<rclcpp::Publisher<formation_msgs::msg::TrjOptimResults>::SharedPtr> m_flw_trj_pub;

  std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_ptr;

  std::vector<std::string> m_followers;
  std::unordered_map<std::string, Eigen::MatrixXd> m_flw_q;
  std::unordered_map<std::string, Eigen::MatrixXd> m_flw_dq;
  std::unordered_map<std::string, Eigen::MatrixXd> m_flw_ddq;
  std::unordered_map<std::string, rclcpp::Time> m_followers_last_message_stamp;
  std::unordered_map<std::string, Eigen::Isometry3d> m_follower_tool_in_leader_tool;

  bool m_is_omni {false};

  // MPC parameters
  unsigned int m_control_horizon;
  unsigned int m_prediction_horizon;
  double m_clik_gain;
  double m_dt;

  // rdyn parameters
  Eigen::Vector3d m_gravity {0, 0, -9.80665};
  rdyn::ChainPtr m_rdyn_full_chain;
//  std::unordered_map<std::string, rdyn::ChainPtr> m_flw_rdyn_chain_map;

  // HQP solution
  Eigen::VectorXd       m_solutions;

  // Stack of tasks
  taskQP::math::TaskStack                           m_sot;

  // Array of Limits
  taskQP::math::LimitsArray                  m_ineq_array;

  // Model
  taskQP::math::virtualModel                      m_model;

  //** Tasks **//
  taskQP::math::MinimizeAcceleration          m_minimize_acc;
  taskQP::math::MinimizeVelocity              m_minimize_vel;
  taskQP::math::CartesianTask                 m_cartesian_leader_task;

  //** Limits **//
  taskQP::math::UpperAccelerationLimits             m_ub_acc;
  taskQP::math::LowerAccelerationLimits             m_lb_acc;
  taskQP::math::UpperVelocityLimits                 m_ub_vel;
  taskQP::math::LowerVelocityLimits                 m_lb_vel;
  taskQP::math::UpperPositionLimits                 m_ub_pos;
  taskQP::math::UpperInvarianceConstraint           m_ub_inv;
  taskQP::math::LowerPositionLimits                 m_lb_pos;
  taskQP::math::LowerInvarianceConstraint           m_lb_inv;
  taskQP::math::ScalingLimits                  m_max_scaling;

  // Parameters generation
  std::shared_ptr<ParamListener> m_param_listener;
  Params m_params;
  void declare_parameters();
  controller_interface::CallbackReturn read_parameters();
private:
  void updateFollowerTrjCallback(const trajectory_msgs::msg::JointTrajectory& msg, const std::string& name);
  void acquireFormation();
//  bool loadFollowerMobileChain(const std::string& t_name,
//                                    const std::string& t_base_frame,
//                                    const std::string& t_tool_frame,
//                                    const std::string& t_robot_description,
//                                    rdyn::ChainPtr& t_rdyn_chain);
};
} // formation_mpc
//PLUGINLIB_EXPORT_CLASS;
#endif // LEADER_MPC_HPP
