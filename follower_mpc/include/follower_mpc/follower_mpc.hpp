#ifndef FOLLOWER_MPC_HPP
#define FOLLOWER_MPC_HPP

#include "formation_utils/utils.hpp"

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
#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

//#include <rclcpp_lifecycle/state.hpp>
//#include <rclcpp_lifecycle/lifecycle_node.hpp>
//#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <controller_interface/controller_interface.hpp>
#include <controller_interface/helpers.hpp>
#include <control_toolbox/pid.hpp>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_server_goal_handle.h>

//#include <hardware_interface/types/hardware_interface_type_values.hpp>

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
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/msg/cartesian_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "formation_msgs/msg/trj_optim_results.hpp"

#include "follower_mpc_parameters.hpp"

#ifndef FAKE_MOBILE_BASE_PATH
#define FAKE_MOBILE_BASE_PATH "../share/config/fake_base.urdf"
#endif

namespace formation_mpc
{
  using CallbackReturn = controller_interface::CallbackReturn;
  using namespace std::placeholders;
  constexpr double k_task_level_multiplier = 1e-3;

class FollowerMPC : public controller_interface::ControllerInterface
{
public:
  FollowerMPC();

  void compute_velocity_command();

  void set_plan(const moveit_msgs::msg::CartesianTrajectory& trj);

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure (const rclcpp_lifecycle::State& /*state*/) override;
  CallbackReturn on_activate  (const rclcpp_lifecycle::State& /*state*/) override;
//  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*state*/) override;
  // CallbackReturn on_cleanup   (const rclcpp_lifecycle::State& /*state*/) override;
  // CallbackReturn on_error     (const rclcpp_lifecycle::State& /*state*/) override;
  // CallbackReturn on_shutdown  (const rclcpp_lifecycle::State& /*state*/) override;
  void reset     ();

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;


protected:

  // --- ros2_control ---
  std::vector<std::string> m_joint_names;
  std::vector<std::string> m_joint_base_names;
  std::vector<std::string> m_joint_arm_names;
  std::vector<std::string> m_command_interface_types;
  std::vector<std::string> m_state_interface_types;

  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> m_joint_position_command_interface;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> m_joint_velocity_command_interface;

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>   m_joint_position_state_interface;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>   m_joint_velocity_state_interface;
  // ------


  rclcpp::TimerBase::SharedPtr m_update_timer;

  rclcpp::Time m_t {0};

  utils::Trajectory m_plan;

  std::vector<geometry_msgs::msg::PoseStamped> m_plan_timed;

  const rclcpp::Duration k_max_delay = rclcpp::Duration::from_nanoseconds(1e8); // 100ms
  double k_e_f {0.1};

  Eigen::ArrayXd m_kp;

  std::mutex m_mtx;

  unsigned int m_nax;
  unsigned int m_nax_arm;
  unsigned int m_nax_base;

  Eigen::VectorXd m_q__state;
  Eigen::VectorXd m_q__start;

  // Eigen::VectorXd m_q;
  // Eigen::VectorXd m_dq;
  // Eigen::VectorXd m_ddq;

  Eigen::VectorXd m_q__cmd;
  Eigen::VectorXd m_dq__cmd;
  Eigen::VectorXd m_ddq__cmd;

  std::vector<control_toolbox::Pid> m_pid__vector;

  Eigen::Vector6d m_twist_tool_in_map;
  Eigen::Vector6d m_old_twist;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr          m_robot_description__sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr   m_joint_state__sub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr      m_leader_twist__sub;
  void get_payload_twist__cb(const geometry_msgs::msg::Twist& msg);

  rclcpp::TimerBase::SharedPtr m_timer;

  std::string              m_robot_description;

  std::string m_base_footprint;
  std::string m_base_link;
  std::string m_tool_frame;
  std::string m_map_frame;
  std::string m_payload_frame;

  Eigen::Affine3d m_T_from_payload_to_tool;

  std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_ptr;

  bool m_is_omni {false};

  // MPC parameters
  double m_control_horizon_in_s;
  // unsigned int m_prediction_horizon;
  unsigned int m_number_of_points;
  double m_clik_gain;
  double m_dt;

//  utils::Interpolator m_interpolator;

  // rdyn parameters
  Eigen::Vector3d m_gravity {0, 0, -9.80665};
  rdyn::ChainPtr  m_rdyn_full_chain;

  // HQP solution
  Eigen::VectorXd       m_solutions;

  // Scaling
  double m_scaling {1.0};

  // Stack of tasks
  taskQP::math::TaskStack                           m_sot;

  // Array of Limits
  taskQP::math::LimitsArray                  m_ineq_array;

  // Model
  taskQP::math::virtualModel                      m_model;

  //** Tasks **//
  taskQP::math::JointPositionTask             m_task__joint_position;
  taskQP::math::CartesianTask                 m_task__cartesian;

  //** Limits **//
  taskQP::math::UpperAccelerationLimits             m_ub_acc;
  taskQP::math::LowerAccelerationLimits             m_lb_acc;
  taskQP::math::UpperVelocityLimits                 m_ub_vel;
  taskQP::math::LowerVelocityLimits                 m_lb_vel;
  taskQP::math::UpperInvarianceConstraint           m_ub_inv;
  taskQP::math::LowerInvarianceConstraint           m_lb_inv;
  taskQP::math::UpperPositionLimits                 m_ub_pos;
  taskQP::math::LowerPositionLimits                 m_lb_pos;
  taskQP::math::ScalingLimits                  m_max_scaling;

  // Parameters generation
  std::shared_ptr<ParamListener> m_param_listener;
  Params m_params;
  void declare_parameters ();
  bool read_parameters    ();
};
} // formation_mpc

#endif // FOLLOWER_MPC_HPP
