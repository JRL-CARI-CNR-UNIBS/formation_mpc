#include "leader_mpc/leader_mpc.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace formation_mpc {
LeaderMPC::LeaderMPC()
  : controller_interface::ControllerInterface(),
    m_action_monitor_period(0,0)
{}

controller_interface::InterfaceConfiguration
LeaderMPC::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf
      = {controller_interface::interface_configuration_type::INDIVIDUAL, {}};

  conf.names.reserve(m_joint_names.size() * m_command_interface_types.size());
  RCLCPP_DEBUG(this->get_node()->get_logger(), "Command interfaces:");
  for (const auto & joint_name : m_joint_names)
  {
    for (const auto & interface_type : m_command_interface_types)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
      RCLCPP_DEBUG(this->get_node()->get_logger(), "%s", conf.names.back().c_str());
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
  RCLCPP_DEBUG(this->get_node()->get_logger(), "State interfaces:");
  for (const auto & joint_name : m_joint_names)
  {
    for (const auto & interface_type : m_state_interface_types)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
      RCLCPP_DEBUG(this->get_node()->get_logger(), "%s", conf.names.back().c_str());
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
  if (m_params.command_interface.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'command_interface' parameter was empty");
    return false;
  }
  else if (std::find(m_params.command_interface.begin(), m_params.command_interface.end(), hardware_interface::HW_IF_VELOCITY) == m_params.command_interface.end() &&
           std::find(m_params.command_interface.begin(), m_params.command_interface.end(), hardware_interface::HW_IF_POSITION) == m_params.command_interface.end())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'command_interface' has no 'velocity' nor 'position' type");
    return false;
  }
  if (m_params.state_interface.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'state_interface' parameter was empty");
    return false;
  }
  else if (   std::find(m_params.state_interface.begin(), m_params.state_interface.end(), hardware_interface::HW_IF_VELOCITY) == m_params.state_interface.end()
           || std::find(m_params.state_interface.begin(), m_params.state_interface.end(), hardware_interface::HW_IF_POSITION) == m_params.state_interface.end())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'state_interface' has no 'velocity' or 'position' type");
    return false;
  }
  return true;

}

CallbackReturn
LeaderMPC::on_init()
{
//#ifdef DEBUG_ON
//#include "rcutils/error_handling.h"
//if(rcutils_logging_set_logger_level(this->get_node()->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG))
//{
//  RCLCPP_ERROR(this->get_node()->get_logger(), "Cannot set logger severity to DEBUG, using default");
//}
//else
//{
//  RCLCPP_DEBUG(this->get_node()->get_logger(), "Enable DEBUG logging");
//}
//#endif
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

  m_joint_base_names = m_params.joints.base;
  m_joint_arm_names  = m_params.joints.arm;
  m_joint_names = m_joint_base_names;
  m_joint_names.insert(m_joint_names.end(), m_joint_arm_names.begin(), m_joint_arm_names.end());
  for(auto& cmd_iface : m_params.command_interface)
  {
    m_command_interface_types.push_back(cmd_iface);
  }
  for(auto& st_iface : m_params.state_interface)
  {
    m_state_interface_types.push_back(st_iface);
  }

  m_robot_description = m_params.robot_description;

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
//  if(!fake_mobile_urdf.initFile(fake_base_pkg_dir + "/" + m_params.fake_base.path))
//  {
//    RCLCPP_ERROR(this->get_node()->get_logger(), "Cannot read fake base urdf in %s/%s", fake_base_pkg_dir.c_str(), m_params.fake_base.path.c_str());
//    return CallbackReturn::ERROR;
//  }
  if(!fake_mobile_urdf.initString(m_params.robot_description))
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
  m_rdyn_full_chain->setInputJointsName(m_joint_names, e);
  m_joint_names = m_rdyn_full_chain->getActiveJointsName();
  RCLCPP_DEBUG(this->get_node()->get_logger(), "Rdyn: OK");

  RCLCPP_DEBUG(this->get_node()->get_logger(), "MPC config: start");
  m_control_horizon_in_s = m_params.control_horizon_in_s;
  m_number_of_points = m_params.number_of_points;
  int nax = m_rdyn_full_chain->getActiveJointsNumber();
  m_nax = nax;
  m_model.setPredictiveHorizon(m_control_horizon_in_s);
  m_model.setSamplingPeriod(m_params.dt);
  m_dt = m_params.dt;

  m_model.init(nax, m_params.number_of_points, m_dt);

  // Inequality array
  m_ineq_array.clear();
  m_ineq_array.set_n_axis(nax);
  m_ineq_array.set_np(m_number_of_points);

  // == Tasks ==
  // ==== Cartesian Task
  std::vector<int> int_axis(m_params.cartesian_axis.size());
  std::transform(m_params.cartesian_axis.begin(),m_params.cartesian_axis.end(),int_axis.begin(),
                 [](const auto& b){return (int)b;});
  m_task__cartesian.init(&m_model, m_rdyn_full_chain.get(), int_axis);
  m_task__cartesian.activateScaling(m_params.cartesian_task_weight.active);
  m_task__cartesian.setWeightScaling(m_params.cartesian_task_weight.weight);
  m_task__cartesian.enableClik(m_params.clik.active);
  m_task__cartesian.setWeightClik(m_params.clik.gain);

  // ==== Joint Position
  m_task__joint_position.init(&m_model);
  Eigen::VectorXd weigth_vector_full(m_nax*m_number_of_points);
  for(size_t idx = 0; idx < m_number_of_points; ++idx)
  {
    weigth_vector_full.segment(idx*m_nax, m_nax) << 0,0,1,1,1,1,1,1,1;
  }
  Eigen::MatrixXd diag(m_nax * m_number_of_points, m_nax * m_number_of_points);
  diag.diagonal() = weigth_vector_full;
  m_task__joint_position.setWeightingMatrix(diag);


  // == Stack of tasks ==
  m_sot.clear();
  m_sot.set_n_axis(nax);
  m_sot.set_np(m_number_of_points);
  m_sot.taskPushBack(&m_task__cartesian,      1);
  m_sot.taskPushBack(&m_task__joint_position, 1e-3);

  // == Constraints ==
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
  m_max_scaling.setLimits(m_params.scaling);
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
  m_q__start.resize(nax);
  m_q__cmd   = Eigen::VectorXd::Zero(m_nax * m_number_of_points);
  m_dq__cmd  = Eigen::VectorXd::Zero(m_nax * m_number_of_points);
  m_ddq__cmd = Eigen::VectorXd::Zero(m_nax * m_number_of_points);

  m_target_dx.resize(m_number_of_points * 6);
  m_target_dx.setZero();

  m_trj_server__action = rclcpp_action::create_server<FollowFormationTrajectory>(
                           this->get_node(),
                           std::string(this->get_node()->get_name()) + "/follow_leader_trajectory",
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

  m_joint_position_command_interface.clear();
  m_joint_position_state_interface.clear();
  m_joint_velocity_state_interface.clear();

  m_send_to_follower__pub = this->get_node()->create_publisher<geometry_msgs::msg::Twist>("leader/cmd_vel", 1);

  // DEBUG
  m_pose_target__pub =  this->get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose",10);
  m_twist_target__pub = this->get_node()->create_publisher<geometry_msgs::msg::TwistStamped>("target_twist",10);

  RCLCPP_INFO(this->get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
LeaderMPC::on_activate(const rclcpp_lifecycle::State& /*state*/)
{
  using namespace std::chrono_literals;
  m_solutions.resize((m_nax+1)*m_number_of_points);
  try
  {
    // TODO: Sostituisci con FK?
    geometry_msgs::msg::TransformStamped msg = m_tf_buffer_ptr->lookupTransform(m_map_frame, m_tool_frame, tf2::TimePointZero, 1s);
    m_target_x = tf2::transformToEigen(msg);
  }
  catch(tf2::TransformException& ex)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(),"Cannot transform from [%s] to [%s]: %s",
                                                m_map_frame.c_str(), m_tool_frame.c_str(), ex.what());
    return CallbackReturn::FAILURE;
  }

  m_target_dx.resize(m_number_of_points * 6);
  m_target_dx.setZero();

  for (size_t idx = 0; idx < state_interfaces_.size(); ++idx)
  {
    if(state_interfaces_.at(idx).get_interface_name() == hardware_interface::HW_IF_POSITION)
    {
      m_joint_position_state_interface.push_back(state_interfaces_.at(idx));
    }
    else if(state_interfaces_.at(idx).get_interface_name() == hardware_interface::HW_IF_VELOCITY)
    {
      m_joint_velocity_state_interface.push_back(state_interfaces_.at(idx));
    }
  }
  for (size_t idx = 0; idx < command_interfaces_.size(); ++idx)
  {
    if(command_interfaces_.at(idx).get_interface_name() == hardware_interface::HW_IF_POSITION)
    {
      m_joint_position_command_interface.push_back(command_interfaces_.at(idx));
    }
    else if(command_interfaces_.at(idx).get_interface_name() == hardware_interface::HW_IF_VELOCITY)
    {
      m_joint_velocity_command_interface.push_back(command_interfaces_.at(idx));
    }
  }

  for(size_t idx = 0; idx < m_nax; ++idx)
  {
    m_q__state(idx) = m_joint_position_state_interface.at(idx).get().get_value();
  }

  Eigen::VectorXd initial_state( 2 * m_nax);
  initial_state.head(m_nax) = m_q__state;
  initial_state.tail(m_nax) = Eigen::VectorXd::Zero(m_nax);
  m_model.setInitialState( initial_state );

  RCLCPP_INFO(this->get_node()->get_logger(), "Activate successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
LeaderMPC::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_WARN(this->get_node()->get_logger(), "on_deactive() not implemented properly. Skipping...");
  return CallbackReturn::SUCCESS;
}

void
LeaderMPC::set_plan(const moveit_msgs::msg::CartesianTrajectory& t_trj)
{
  utils::Interpolator interpolator = utils::Interpolator::from_msg(t_trj);
  geometry_msgs::msg::TransformStamped T_object_to_tool = m_tf_buffer_ptr->lookupTransform(m_params.object_frame, m_params.tool_frame, tf2::TimePointZero);
  m_interpolator = interpolator.clone_with_transform(T_object_to_tool);
  m_T_from_object_to_tool = tf2::transformToEigen(T_object_to_tool);
}

controller_interface::return_type
LeaderMPC::update(const rclcpp::Time & t_time, const rclcpp::Duration & /*t_period*/)
{
//  m_t = t_time;
  for(size_t idx = 0; idx < m_nax; ++idx)
  {
    m_q__state(idx) = m_joint_position_state_interface.at(idx).get().get_value();
  }
  if(!m_interpolator.ready() || !m_interpolator.is_plan_available())
  {
    geometry_msgs::msg::Twist empty_msg;
    Eigen::Vector6d zero = Eigen::Vector6d::Zero();
    empty_msg = tf2::toMsg(zero);
    m_send_to_follower__pub->publish(empty_msg);
    return controller_interface::return_type::OK;
  }
  if(!m_interpolator.is_plan_started())
  {
    m_interpolator.start_plan(t_time);
    RCLCPP_DEBUG(this->get_node()->get_logger(), "Starting plan");
    // ******* Debug ***********
    {
      auto pub__shifted = this->get_node()->create_publisher<moveit_msgs::msg::CartesianTrajectory>("/cartesian_trajectory", 10);
      auto ct = moveit_msgs::msg::CartesianTrajectory();
      ct.tracked_frame="t2";
      ct.points.resize(m_interpolator.get_plan().size());
      for(size_t idx = 0; idx < ct.points.size(); idx++)
      {
        ct.points.at(idx).point.pose = tf2::toMsg(m_interpolator.get_plan().pose.at(idx));
      }
      pub__shifted->on_activate();
      pub__shifted->publish(ct);
    }
    // *************************
    for(size_t idx = 0; idx < m_nax; ++idx)
    {
      m_q__start(idx) = m_q__state(idx);
      m_q__cmd(idx)   = m_q__state(idx);
    }
  }

  Eigen::Affine3d target_x;
  Eigen::Vector6d target_dx;
  for(size_t idx = 0; idx < m_number_of_points; ++idx)
  {
    target_x.setIdentity();
    target_dx.setZero();
    m_interpolator.interpolate(t_time + rclcpp::Duration::from_seconds(m_model.getPredictionTimes()(idx)), target_dx, target_x);
    m_target_dx.segment<6>(idx*6) = target_dx;
    if(idx == 0)
    {
      // DEBUG
      m_target_x = target_x;
      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = t_time;
      msg.pose = tf2::toMsg(target_x);
      m_pose_target__pub->publish(msg);

      // DEBUG
      geometry_msgs::msg::TwistStamped msg2;
      msg2.header.stamp = t_time;
      msg2.twist = tf2::toMsg(target_dx);
      m_twist_target__pub->publish(msg2);
    }
  }
  if((t_time - m_interpolator.get_plan().start).seconds() >= m_interpolator.get_plan().time.back().seconds())
  {
      m_interpolator.end_plan();
  }


  // Update Task references
  // == Cartesian task ==
  taskQP::math::CartesianTask* cartesian_task_ptr = dynamic_cast<taskQP::math::CartesianTask*>(m_sot.getTask(0));
  cartesian_task_ptr->setTargetTrajectory(m_target_dx, m_target_x);
  cartesian_task_ptr->setTargetScaling(1.0);
  // == Joint position ==
  Eigen::VectorXd np_q = Eigen::VectorXd(m_nax*m_number_of_points);
  for(unsigned int idx = 0; idx < m_number_of_points; ++idx){np_q.segment(idx*m_nax, m_nax) = m_q__start;}
  m_task__joint_position.setTargetPosition(np_q);


  // Update task
  for(size_t idx = 0; idx < m_sot.stackSize(); ++idx)
  {
    m_sot.getTask(idx)->update(m_model.getPositionPrediction(), m_model.getVelocityPrediction());
  }
  // Update constraints
  for(size_t idx = 0; idx < m_ineq_array.arraySize(); ++idx)
  {
    m_ineq_array.getConstraint(idx)->update(m_model.getPositionPrediction(), m_model.getVelocityPrediction());
  }
  Eigen::MatrixXd CE;
  Eigen::VectorXd ce0;
  taskQP::math::computeHQPSolution(m_sot, CE, ce0, m_ineq_array.matrix(), m_ineq_array.vector(), m_solutions);

  m_model.updatePredictions(m_solutions.head(m_nax*m_number_of_points));
  m_model.updateState(m_solutions.head(m_nax));

  m_ddq__cmd = m_solutions.head(m_nax);
  m_dq__cmd  = m_model.getState().tail(m_nax);
  m_q__cmd   = m_model.getState().head(m_nax);

  m_scaling = m_solutions(m_nax * m_number_of_points);

  Eigen::Vector6d twist__to_follower = rdyn::spatialTranslation(m_rdyn_full_chain->getJacobian(m_q__cmd) * m_dq__cmd, m_T_from_object_to_tool.translation());
  geometry_msgs::msg::Twist msg__to_follower;
  msg__to_follower.linear.x =  twist__to_follower(0);
  msg__to_follower.linear.y =  twist__to_follower(1);
  msg__to_follower.linear.z =  twist__to_follower(2);
  msg__to_follower.angular.x = twist__to_follower(3);
  msg__to_follower.angular.y = twist__to_follower(4);
  msg__to_follower.angular.z = twist__to_follower(5);
  m_send_to_follower__pub->publish(msg__to_follower);

  // Eigen::VectorXd cartesian_error = cartesian_task_ptr->computeTaskError(m_target_x, m_q__cmd);
  // Eigen::VectorXd joint_error = m_q__cmd - m_q__state;
  for(size_t idx = 0; idx < m_nax; ++idx)
  {
//    m_joint_velocity_command_interface.at(idx).get().set_value(m_dq__cmd(idx));
    m_joint_position_command_interface.at(idx).get().set_value(m_q__cmd(idx));
//    m_joint_velocity_command_interface
//        .at(idx)
//        .get()m_joint_position_state_interface.at(idx).get().get_value()
//        .set_value(
//          m_pid__vector.at(idx).computeCommand(joint_error(idx), m_dt) + m_dq__cmd(idx)
//          );
  }
  if((this->get_node()->get_clock()->now() - t_time).seconds() > m_dt)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Control iteration duration did not respect the deadline");
  }
  return controller_interface::return_type::OK;
}

rclcpp_action::GoalResponse
LeaderMPC::handle_goal__cb(const rclcpp_action::GoalUUID & uuid,
                       std::shared_ptr<const FollowFormationTrajectory::Goal> /*goal*/)
{
  RCLCPP_INFO(this->get_node()->get_logger(), "Received goal request");
  if (this->get_node()->get_current_state().label() != "active")
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Cannot accept request. Lifecycle state: %s", this->get_node()->get_current_state().label().c_str());
//    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    return rclcpp_action::GoalResponse::REJECT;
  }
  else if (m_is_there_goal_active)
  {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Goal active. Cannot be overwritten");
    return rclcpp_action::GoalResponse::REJECT;
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
  RCLCPP_INFO(this->get_node()->get_logger(), "Trajectory accepted");
}


} // formation_mpc

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(formation_mpc::LeaderMPC, controller_interface::ControllerInterface)
