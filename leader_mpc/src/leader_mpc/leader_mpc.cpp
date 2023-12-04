#include "leader_mpc/leader_mpc.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace formation_mpc {
LeaderMPC::LeaderMPC()
  : controller_interface::ControllerInterface(),
    m_action_monitor_period(0,0)
{
  m_joint_names = auto_declare<std::vector<std::string>>("joints", m_joint_names);
  m_command_interface_types =
    auto_declare<std::vector<std::string>>("command_interfaces", m_command_interface_types);
  m_state_interface_types =
    auto_declare<std::vector<std::string>>("state_interfaces", m_state_interface_types);

  #ifdef DEBUG_ON
  #include "rcutils/error_handling.h"
  if(rcutils_logging_set_logger_level(this->get_node()->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG))
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Cannot set logger severity to DEBUG, using default");
  }
  else
  {
    RCLCPP_DEBUG(this->get_node()->get_logger(), "Enable DEBUG logging");
  }
  #endif
}

controller_interface::InterfaceConfiguration
LeaderMPC::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf
      = {controller_interface::interface_configuration_type::INDIVIDUAL, {}};

  conf.names.reserve(m_joint_names.size() * m_command_interface_types.size());
  for (const auto & joint_name : m_joint_names)
  {
    for (const auto & interface_type : m_command_interface_types)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::InterfaceConfiguration
LeaderMPC::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf
      = {controller_interface::interface_configuration_type::INDIVIDUAL, {}};

  conf.names.reserve(m_joint_names.size() * m_state_interface_types.size());
  for (const auto & joint_name : m_joint_names)
  {
    for (const auto & interface_type : m_state_interface_types)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

void
LeaderMPC::declare_parameters()
{
  m_param_listener = std::make_shared<ParamListener>(this->get_node()->shared_from_this());
  RCLCPP_DEBUG(this->get_node()->get_logger(), "Parameter declared");
}

bool
LeaderMPC::read_parameters()
{
  if(!m_param_listener)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Error encountered during init");
    return false;
  }
  m_params = m_param_listener->get_params();
  if(m_params.__stamp == rclcpp::Time(0.0))
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Parameters not loaded");
    return false;
  }
  return true;

}

CallbackReturn
LeaderMPC::on_init()
{
  try
  {
    declare_parameters();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn
LeaderMPC::on_configure(const rclcpp_lifecycle::State& /*state*/)
{
//  init();
  m_tf_buffer_ptr = std::make_unique<tf2_ros::Buffer>(this->get_node()->get_clock());
  m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer_ptr);
  if (!this->read_parameters()) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Error while reading parameters");
    return CallbackReturn::ERROR;
  }; //TODO: boh...

  m_robot_description = m_params.robot_description;

  if(m_params.initial_positions.size() != m_params.joints.size())
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Parameters: initial position size =/= joints size");
    return CallbackReturn::ERROR;
  }

  // rdyn
  urdf::Model urdf_model;
  if(!urdf_model.initString(m_robot_description))
  {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(), "robot description cannot be parsed");
    return CallbackReturn::ERROR;
  }
  if(urdf_model.getLink(m_params.mobile_base_link) == nullptr)
  {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(), m_params.mobile_base_link << " is missing from robot description");
    return CallbackReturn::ERROR;
  }
  if(urdf_model.getLink(m_params.manipulator_base_link) == nullptr)
  {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(), m_params.manipulator_base_link << " is missing from robot description");
    return CallbackReturn::ERROR;
  }
  if(urdf_model.getLink(m_params.tool_frame) == nullptr)
  {
    RCLCPP_ERROR_STREAM(this->get_node()->get_logger(), m_params.tool_frame << " is missing from robot description");
    return CallbackReturn::ERROR;
  }

  rdyn::ChainPtr rdyn_fixed_chain = std::make_shared<rdyn::Chain>(urdf_model,
                                                     m_params.mobile_base_link,
                                                     m_params.tool_frame,
                                                     m_gravity);

  m_tool_frame = m_params.tool_frame;
  m_map_frame = m_params.map_frame;

  urdf::Model fake_mobile_urdf;
  std::string fake_base_pkg_dir;
  try
  {
    fake_base_pkg_dir = ament_index_cpp::get_package_share_directory(m_params.fake_base.package);
  }
  catch(...)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Cannot find package %s. Exiting...", m_params.fake_base.package.c_str());
    return CallbackReturn::ERROR;
  }
  if(!fake_mobile_urdf.initFile(fake_base_pkg_dir + "/" + m_params.fake_base.path))
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Cannot read fake base urdf in %s/%s", fake_base_pkg_dir.c_str(), m_params.fake_base.path.c_str());
    return CallbackReturn::ERROR;
  }
  m_nax_arm = rdyn_fixed_chain->getActiveJointsNumber();

  rdyn::ChainPtr rdyn_fake_mobile_chain = rdyn::createChain(fake_mobile_urdf,
                                                         m_params.map_frame,
                                                         m_params.mount_frame,
                                                         m_gravity);
  m_nax_base = rdyn_fake_mobile_chain->getActiveJointsNumber();
  m_rdyn_full_chain = rdyn::joinChains(rdyn_fake_mobile_chain, rdyn_fixed_chain);

  std::string e;
  // FIXME: ricontrolla a cosa serve rdyn::chain->setInputJointsName
  m_rdyn_full_chain->setInputJointsName(m_params.joints,e);
  m_joint_names = m_rdyn_full_chain->getActiveJointsName();
  RCLCPP_DEBUG(this->get_node()->get_logger(), "Rdyn: OK");

  RCLCPP_DEBUG(this->get_node()->get_logger(), "MPC config: start");
  int nax = m_rdyn_full_chain->getActiveJointsNumber();
  m_nax = nax;
  m_model.setPredictiveHorizon(m_params.prediction_horizon);
  m_prediction_horizon = m_params.prediction_horizon;
  m_model.setSamplingPeriod(m_params.dt);
  m_dt = m_params.dt;

  m_model.init(nax,m_params.control_horizon, m_dt);
  m_control_horizon = m_params.control_horizon;
  // Stack of tasks
  m_sot.set_n_axis(nax);
  m_sot.set_np(m_control_horizon);
  // Inequality array
  m_ineq_array.set_n_axis(nax);
  m_ineq_array.set_np(m_control_horizon);
  // Create task pointers
//  m_minimize_acc.init(&m_model);
//  m_minimize_vel.init(&m_model);
  m_cartesian_leader_task.init(&m_model, m_rdyn_full_chain.get(), {1,1,1,1,1,1}); // activate all cartesian axis
  m_cartesian_leader_task.activateScaling(m_params.scaling.active);

  m_sot.clear();
//  m_sot.taskPushBack(&m_minimize_acc, std::pow(k_task_level_multiplier, m_params.hierarchy.at(0)-1));
//  m_sot.taskPushBack(&m_minimize_vel, std::pow(k_task_level_multiplier, m_params.hierarchy.at(1)-1));
  m_sot.taskPushBack(&m_cartesian_leader_task, std::pow(k_task_level_multiplier, m_params.hierarchy.at(0)-1));

  m_ub_acc.init(&m_model);
  m_lb_acc.init(&m_model);
  m_ub_vel.init(&m_model);
  m_lb_vel.init(&m_model);
  m_ub_pos.init(&m_model);
  m_ub_inv.init(&m_model);
  m_lb_pos.init(&m_model);
  m_lb_inv.init(&m_model);
  m_max_scaling.init(&m_model);
  m_ub_acc.setLimits( m_rdyn_full_chain->getDDQMax());
  m_lb_acc.setLimits(-m_rdyn_full_chain->getDDQMax());
  RCLCPP_DEBUG_STREAM(this->get_node()->get_logger(), "acceleration limits: " << m_rdyn_full_chain->getDDQMax());
  m_ub_vel.setLimits( m_rdyn_full_chain->getDQMax());
  m_lb_vel.setLimits(-m_rdyn_full_chain->getDQMax());
  RCLCPP_DEBUG_STREAM(this->get_node()->get_logger(), "velocity limits: " << m_rdyn_full_chain->getDQMax());
  m_ub_pos.setLimits( m_rdyn_full_chain->getQMax());
  m_ub_inv.setLimits( m_rdyn_full_chain->getQMax(), m_rdyn_full_chain->getDQMax(), -m_rdyn_full_chain->getDDQMax());
  m_lb_pos.setLimits( m_rdyn_full_chain->getQMin());
  m_lb_inv.setLimits( m_rdyn_full_chain->getQMin(), -m_rdyn_full_chain->getDQMax(), m_rdyn_full_chain->getDDQMax());
  m_max_scaling.setLimits(m_params.scaling.limit);
  m_ineq_array.addConstraint(&m_lb_inv);
  m_ineq_array.addConstraint(&m_lb_pos);
  m_ineq_array.addConstraint(&m_ub_inv);
  m_ineq_array.addConstraint(&m_ub_pos);
  m_ineq_array.addConstraint(&m_lb_vel);
  m_ineq_array.addConstraint(&m_ub_vel);
  m_ineq_array.addConstraint(&m_lb_acc);
  m_ineq_array.addConstraint(&m_ub_acc);
  m_ineq_array.addConstraint(&m_max_scaling);
  RCLCPP_DEBUG(this->get_node()->get_logger(), "MPC config: OK");


  m_q__state.resize(nax);
  m_q__cmd.resize(nax * m_control_horizon);
  for(size_t idx = 0; idx < nax; ++idx)
  {
    m_q__cmd(idx) = m_params.initial_positions.at(0);
  }
  m_dq__cmd.resize(nax * m_control_horizon);
  m_ddq__cmd.resize(nax * m_control_horizon);

  m_target_dx.resize(m_control_horizon * 6);
  m_target_dx.setZero();

  m_leader_trj__pub = this->get_node()->create_publisher<formation_msgs::msg::TrjOptimResults>(m_params.trj_leader_topic, 10);
  m_joint_trajectory__pub = this->get_node()->create_publisher<trajectory_msgs::msg::JointTrajectory>(m_params.cmd_trj_topic, 10);
  m_cmd_vel__pub = this->get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(m_params.cmd_vel_topic, 10);

  // TODO: Leggi da state interface
  m_joint_state__sub = this->get_node()->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, [this](const sensor_msgs::msg::JointState& msg){
    std::lock_guard<std::mutex> l(m_mtx);
    for(size_t idx = 0; idx < msg.name.size(); ++idx)
    {
      m_q__state(idx) = msg.position.at(idx);
    }
  });
  //-------

  m_base_pos__pub  = this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("/base_position_controller/commands", 10);
  m_manip_pos__pub = this->get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("/ur_position_controller/commands", 10);

  m_trj_server__action = rclcpp_action::create_server<FollowFormationTrajectory>(
                           this->get_node(),
                           std::string(this->get_node()->get_name()) + "follow_leader_trajectory",
                           std::bind(&LeaderMPC::handle_goal__cb, this, _1, _2),
                           std::bind(&LeaderMPC::handle_cancel__cb, this, _1),
                           std::bind(&LeaderMPC::handle_accepted__cb, this, _1));

  m_action_monitor_period = rclcpp::Duration::from_seconds(1.0 / m_params.action_monitor_rate);

  if(m_params.pid.P.size() != m_nax)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Pid P-gains array of dimension %ld. Should be %d", m_params.pid.P.size(), m_nax);
    return CallbackReturn::ERROR;
  }

  m_pid__vector.reserve(m_nax);
  for(size_t idx = 0; idx < m_nax; ++idx)
  {
    m_pid__vector.push_back(control_toolbox::Pid());
    m_pid__vector.back().initPid(m_params.pid.P.at(idx), 0.0,0.0,0.0,0.0,false);
  }

  RCLCPP_INFO(this->get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
LeaderMPC::on_activate(const rclcpp_lifecycle::State& /*state*/)
{
  using namespace std::chrono_literals;
  RCLCPP_INFO(this->get_node()->get_logger(), "Activate successfully");
  m_solutions.resize((m_nax+1)*m_control_horizon);
  return CallbackReturn::SUCCESS;
  m_dq__cmd  = Eigen::VectorXd::Zero(m_nax * m_control_horizon);
  m_ddq__cmd = Eigen::VectorXd::Zero(m_nax * m_control_horizon);

  Eigen::VectorXd initial_state( 2 * m_nax);
  initial_state.head(m_nax) = m_q__cmd;
  initial_state.tail(m_nax) = m_dq__cmd;
  m_model.setInitialState( initial_state );

  geometry_msgs::msg::TransformStamped msg = m_tf_buffer_ptr->lookupTransform(m_tool_frame, m_map_frame, t_start);
  m_target_x = tf2::transformToEigen(msg);
  m_target_dx.resize(m_control_horizon * 6);
  m_target_dx.setZero();
}

CallbackReturn
LeaderMPC::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_WARN(this->get_node()->get_logger(), "on_deactive() not implemented properly. Skipping...");
  return CallbackReturn::SUCCESS;
}

void
LeaderMPC::set_plan(const moveit_msgs::msg::CartesianTrajectory& trj)
{
  size_t size = trj.points.size();
  m_plan.clear();
  m_plan.resize(size);
  for(size_t idx = 0; idx < size; ++idx)
  {
    tf2::fromMsg(trj.points.at(idx).point.pose, m_plan.pose.at(idx));
    tf2::fromMsg(trj.points.at(idx).point.velocity, m_plan.twist.at(idx));
//    tf2::fromMsg(trj.points.at(idx).point.acceleration, m_plan.acc.at(idx));
    m_plan.time.at(idx) = rclcpp::Time(trj.points.at(idx).time_from_start.sec,
                                       trj.points.at(idx).time_from_start.nanosec);
  }
  RCLCPP_INFO(this->get_node()->get_logger(), "New plan received");
  m_plan.started = false;
}

controller_interface::return_type
LeaderMPC::update(const rclcpp::Time & t_time, const rclcpp::Duration & /*t_period*/)
{
//  auto start_ros = m_t = this->get_node()->get_clock()->now();
//  if(!m_plan.started)
//  {
//    m_plan.start = start_ros;
//    m_plan.started = true;
//  }
  for(size_t idx = 0; idx < m_nax; ++idx)
  {
    m_q__state(idx) = m_joint_position_state_interface.at(idx).get().get_value();
  }

//  std::unique_lock<std::mutex> trj_lock(m_mtx);
  Eigen::Affine3d target_x;
  Eigen::Vector6d target_dx;
  for(size_t idx = 0; idx < m_control_horizon; ++idx)
  {
    interpolate(m_t, m_plan.start, target_dx, target_x);
    m_target_dx.segment<6>(idx) = target_dx;
    if(idx == 0) m_target_x = target_x;
  }

//  m_cartesian_leader_task.setTargetScaling();
  m_cartesian_leader_task.setTargetTrajectory(m_target_dx, m_target_x);

  for(size_t idx = 0; idx < m_sot.stackSize(); ++idx)
  {
    m_sot.getTask(idx)->update(m_model.getPositionPrediction(), m_model.getVelocityPrediction());
  }
  for(size_t idx = 0; idx < m_ineq_array.arraySize(); ++idx)
  {
    m_ineq_array.getConstraint(idx)->update(m_model.getPositionPrediction(), m_model.getVelocityPrediction());
  }
  Eigen::MatrixXd CE;
  Eigen::VectorXd ce0;
  taskQP::math::computeHQPSolution(m_sot, CE, ce0, m_ineq_array.matrix(), m_ineq_array.vector(), m_solutions);
  RCLCPP_DEBUG_STREAM(this->get_node()->get_logger(), "HQP solution" << m_solutions);
//  double scaling = m_solutions(m_nax * m_control_horizon);

  m_model.updatePredictions(m_solutions.head(m_nax*m_control_horizon));
  m_model.updateState(m_solutions.head(m_nax));

  m_ddq__cmd = m_solutions.head(m_nax);
  m_dq__cmd  = m_model.getState().tail(m_nax);
  m_q__cmd   = m_model.getState().head(m_nax);

  Eigen::VectorXd joint_error = m_q__cmd - m_q__state;
  for(size_t idx = 0; idx < m_nax; ++idx)
  {
    m_joint_velocity_command_interface
        .at(idx)
        .get()
        .set_value(
          m_pid__vector.at(idx).computeCommand(joint_error(idx), m_dt) + m_dq__cmd(idx)
          );
  }
  if((this->get_node()->get_clock()->now() - t_time).seconds() > m_dt)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Control iteration duration did not respect deadline");
  }

}

void
LeaderMPC::interpolate(const rclcpp::Time& t_t,
                       const rclcpp::Time& t_start,
                       Eigen::Vector6d& interp_vel,
                       Eigen::Affine3d& out)
{
  rclcpp::Duration T = t_t - t_start;

  for(size_t idx = 1; idx < m_plan.time.size(); idx++)
  {
    rclcpp::Time t_now = rclcpp::Time(T.seconds());
    if(!(    ((t_now - m_plan.time.at(idx)).seconds() < 0)
          && ((t_now - m_plan.time.at(idx-1)).seconds() >= 0)
         )
       )
    {
      continue;
    }
    double delta_time = (m_plan.time.at(idx) - m_plan.time.at(idx-1)).seconds();
    double t = (t_now - m_plan.time.at(idx-1)).seconds();
    double ratio = t / delta_time;
    Eigen::Isometry3d pose_interp;

    const Eigen::Vector3d p0 = m_plan.pose.at(idx-1).translation();
    const Eigen::Vector3d p1 = m_plan.pose.at(idx).translation();
    const Eigen::Vector3d v0 = m_plan.twist.at(idx-1).head<3>();
    const Eigen::Vector3d v1 = m_plan.twist.at(idx).head<3>();

    interp_vel.head<3>() =   3 * (2*p0 + v0 - 2*p1 + v1) * std::pow(t,2)
                           + 2 * (-3*p0 + 3*p1 - 2*v0 - v1) * t
                           + v0;
//    interp_vel.tail<3>() = ratio * m_plan.twist.at(idx).tail<3>() + (1-ratio) * m_plan.twist.at(idx-1).tail<3>();
    interp_vel.tail<3>() = Eigen::Vector3d::Zero(3,1);

    double h00, h01, h10, h11;
    h00 = 2*std::pow(ratio,3) - 3*std::pow(ratio,2) + 1;
    h10 = std::pow(ratio,3) - 2*std::pow(ratio,2) + ratio;
    h01 = -2*std::pow(ratio,3) + 3*std::pow(ratio,2);
    h11 = std::pow(ratio,3) - std::pow(ratio,2);
    out.translation() =   h00 * m_plan.pose.at(idx-1).translation()
                                + h10 * m_plan.twist.at(idx-1).head<3>()
                                + h01 * m_plan.pose.at(idx).translation()
                                + h11 * m_plan.twist.at(idx).head<3>();
//    out.linear() = Eigen::Quaterniond(m_plan.pose.at(idx-1).linear()).slerp(ratio, Eigen::Quaterniond(m_plan.pose.at(idx).linear())).toRotationMatrix();
    out.linear() = m_plan.pose.at(0).linear();
    break;
  }
}

rclcpp_action::GoalResponse
LeaderMPC::handle_goal__cb(const rclcpp_action::GoalUUID & uuid,
                       std::shared_ptr<const FollowFormationTrajectory::Goal> goal)
{
  RCLCPP_INFO(this->get_node()->get_logger(), "Received goal request");
  if (this->get_node()->get_current_state().label() != "active")
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Cannot accept request. Lifecycle state: %s", this->get_node()->get_current_state().label().c_str());
//    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    rclcpp_action::GoalResponse::REJECT;
  }
  else if (m_is_there_goal_active)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Goal active. Cannot be overwritten");
    rclcpp_action::GoalResponse::REJECT;
  }
  else
  {
    RCLCPP_INFO(this->get_node()->get_logger(), "Goal request accepted");
  }
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
LeaderMPC::handle_cancel__cb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowFormationTrajectory>> /*non_rt_goal_handle*/)
{
  RCLCPP_WARN(this->get_node()->get_logger(), "Received request to cancel goal");
  RCLCPP_WARN(this->get_node()->get_logger(), "Proper cancel not implemented yet");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void
LeaderMPC::handle_accepted__cb(std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowFormationTrajectory>> non_rt_goal_handle)
{
  RTGoalHandlePtr rt_goal = std::make_shared<RTGoalHandle>(non_rt_goal_handle);
  set_plan(non_rt_goal_handle->get_goal()->payload_cartesian_trajectory);
  rt_goal->preallocated_feedback_->joint_names = m_joint_names;
  rt_goal->execute();
  m_rt_active_goal.writeFromNonRT(rt_goal);

  // Crea timer per controllare lo stato dell'azione
  m_timer.reset();
  m_timer = this->get_node()->create_wall_timer(
              m_action_monitor_period.to_chrono<std::chrono::nanoseconds>(),
              std::bind(&RTGoalHandle::runNonRealtime, rt_goal));
  RCLCPP_INFO(this->get_node()->get_logger(), "Starting trajectory");
}


} // formation_mpc

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(formation_mpc::LeaderMPC, controller_interface::ControllerInterface)
