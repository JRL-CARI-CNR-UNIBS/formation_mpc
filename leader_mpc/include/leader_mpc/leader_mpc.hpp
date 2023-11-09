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

//#include "controller_interface/controller_interface.hpp"
//#include "controller_interface/helpers.hpp"

//#include "realtime_tools/realtime_buffer.h"
//#include "realtime_tools/realtime_publisher.h"

#include "nav2_core/controller.hpp"

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
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "formation_msgs/msg/trj_optim_results.hpp"

#include "subscription_notifier/subscription_notifier.h"

#include "leader_mpc_parameters.hpp"

#ifndef FAKE_MOBILE_BASE_PATH
#define FAKE_MOBILE_BASE_PATH "../share/config/fake_base.urdf"
#endif

namespace formation_mpc
{
constexpr double k_task_level_multiplier = 1e-3;
class LeaderMPC : public nav2_core::Controller // FIXME: ros2_controller or nav2_controller or moveit_local_planner
{
public:
  LeaderMPC();
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & robot_pose,
      const geometry_msgs::msg::Twist & robot_speed,
      nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  bool init(const rclcpp_lifecycle::LifecycleNode::WeakPtr&);
  void configure (const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void activate  () override;
  void deactivate() override;
  void cleanup   () override;
  void error     ();
  void shutdown  ();
  void reset     ();

protected:

  rclcpp::Time m_t {0};
  bool m_is_new_plan {false};
  nav_msgs::msg::Path m_plan_path;
  trajectory_msgs::msg::MultiDOFJointTrajectory m_plan;
  std::vector<geometry_msgs::msg::PoseStamped> m_plan_timed;

  const rclcpp::Duration k_max_delay = rclcpp::Duration::from_nanoseconds(1e8); // 100ms
  double k_e_f {0.1};

  rclcpp_lifecycle::LifecycleNode::WeakPtr m_node_weak;

  std::mutex m_mtx;

//  std::string m_plan_topic {"/plan"};
//  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_get_plan__sub_ptr;
//  nav_msgs::msg::Path m_original_plan;

  unsigned int m_nax;
  unsigned int m_nax_arm;
  unsigned int m_nax_base;

  Eigen::VectorXd m_q_now;

  Eigen::MatrixXd m_q;
  Eigen::MatrixXd m_dq;
  Eigen::MatrixXd m_ddq;

  Eigen::Vector6d m_target_dx;
  Eigen::Affine3d m_target_x;

  // Publishers
  rclcpp::Publisher<formation_msgs::msg::TrjOptimResults>::SharedPtr m_leader_trj__pub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_cmd_vel__pub;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_joint_trajectory__pub;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_joint_state__sub;
  //  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_joints_command__sub; // ??

  std::vector<std::string> m_joint_names;

  std::vector<std::string> m_command_interface_types;

  std::string m_base_footprint  {"base_footprint"};
  std::string m_base_link       {"base_link"};
  std::string m_tool_frame      {"tool0"};
  std::string m_map_frame       {"map"};
  std::string m_namespace       {"leader"};
  std::string this_frame(const std::string& s) { return m_namespace + "/" + s;}

//  const std::vector<std::string> m_allowed_interface_types = {
//    hardware_interface::HW_IF_POSITION,
//    hardware_interface::HW_IF_VELOCITY,
//    hardware_interface::HW_IF_ACCELERATION};

//  std::vector<rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr> m_flw_trj_sub;
//  std::vector<rclcpp::Publisher<formation_msgs::msg::TrjOptimResults>::SharedPtr> m_flw_trj_pub;

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

  Eigen::Vector3d       m_cartesian_velocity_limit {0.0, 0.0, 0.0};
  Eigen::Vector3d       m_cartesian_rotation_velocity_limit {0.0,0.0,0.0};

  // HQP solution
  Eigen::VectorXd       m_solutions;

  // Stack of tasks
  taskQP::math::TaskStack                           m_sot;

  // Array of Limits
  taskQP::math::LimitsArray                  m_ineq_array;

  // Model
  taskQP::math::virtualModel                      m_model;

  //** Tasks **//
//  taskQP::math::MinimizeAcceleration          m_minimize_acc;
//  taskQP::math::MinimizeVelocity              m_minimize_vel;
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
  void declare_parameters (const rclcpp_lifecycle::LifecycleNode::WeakPtr&);
  bool read_parameters    (const rclcpp_lifecycle::LifecycleNode::SharedPtr&);
private:
  void updateFollowerTrjCallback(const trajectory_msgs::msg::JointTrajectory& msg, const std::string& name);
  void acquireFormation();
  void interpolate(geometry_msgs::msg::Pose& out);
//  bool loadFollowerMobileChain(const std::string& t_name,
//                                    const std::string& t_base_frame,
//                                    const std::string& t_tool_frame,
//                                    const std::string& t_robot_description,
//                                    rdyn::ChainPtr& t_rdyn_chain);
};
} // formation_mpc
//PLUGINLIB_EXPORT_CLASS;
#endif // LEADER_MPC_HPP
