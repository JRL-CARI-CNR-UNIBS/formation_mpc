#include "leader_mpc/leader_mpc.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace formation_mpc {
LeaderMPC::LeaderMPC(const std::string name, const rclcpp::NodeOptions options)
  : rclcpp_lifecycle::LifecycleNode(name, options)
{
  #ifdef DEBUG_ON
  #include "rcutils/error_handling.h"
  if(rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG))
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot set logger severity to DEBUG, using default");
  }
  else
  {
    RCLCPP_DEBUG(this->get_logger(), "Enable DEBUG logging");
  }
  #endif
}

void LeaderMPC::declare_parameters()
{
  m_param_listener = std::make_shared<ParamListener>(this->shared_from_this());
  RCLCPP_DEBUG(this->get_logger(), "Parameter declared");
}

bool LeaderMPC::read_parameters()
{
  if(!m_param_listener)
  {
    RCLCPP_ERROR(this->get_logger(), "Error encountered during init");
    return false;
  }
  m_params = m_param_listener->get_params();
  if(m_params.__stamp == rclcpp::Time(0.0))
  {
    RCLCPP_ERROR(this->get_logger(), "Parameters not loaded");
    return false;
  }
  return true;

}

bool LeaderMPC::init()
{
  try
  {
    declare_parameters();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return false;
  }

  return true;
}

 CallbackReturn LeaderMPC::on_configure(const rclcpp_lifecycle::State& /*state*/)
{
  init();
  m_tf_buffer_ptr = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer_ptr);
  if (!this->read_parameters()) {
    RCLCPP_ERROR(this->get_logger(), "Error while reading parameters");
    return CallbackReturn::ERROR;
  }; //TODO: boh...

  m_robot_description = m_params.robot_description;

  // rdyn
  urdf::Model urdf_model;
  if(!urdf_model.initString(m_robot_description))
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "robot description cannot be parsed");
    return CallbackReturn::ERROR;
  }
  if(urdf_model.getLink(m_params.mobile_base_link) == nullptr)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), m_params.mobile_base_link << " is missing from robot description");
    return CallbackReturn::ERROR;
  }
  if(urdf_model.getLink(m_params.manipulator_base_link) == nullptr)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), m_params.manipulator_base_link << " is missing from robot description");
    return CallbackReturn::ERROR;
  }
  if(urdf_model.getLink(m_params.tool_frame) == nullptr)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), m_params.tool_frame << " is missing from robot description");
    return CallbackReturn::ERROR;
  }

  rdyn::ChainPtr rdyn_fixed_chain = std::make_shared<rdyn::Chain>(urdf_model,
                                                     m_params.mobile_base_link,
                                                     m_params.tool_frame,
                                                     m_gravity);

  urdf::Model fake_mobile_urdf;
  std::string fake_base_pkg_dir;
  try
  {
    fake_base_pkg_dir = ament_index_cpp::get_package_share_directory(m_params.fake_base.package);
  }
  catch(...)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot find package %s. Exiting...", m_params.fake_base.package.c_str());
    return CallbackReturn::ERROR;
  }
  if(!fake_mobile_urdf.initFile(fake_base_pkg_dir + "/" + m_params.fake_base.path))
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot read fake base urdf in %s/%s", fake_base_pkg_dir.c_str(), m_params.fake_base.path.c_str());
    return CallbackReturn::ERROR;
  }
  m_nax_arm = rdyn_fixed_chain->getActiveJointsNumber();
  rdyn::ChainPtr rdyn_fake_mobile_chain = std::make_shared<rdyn::Chain>(fake_mobile_urdf,
                                                         m_params.map_frame,
                                                         m_params.mount_frame, // TODO: parameter? constant?
                                                         m_gravity);
  m_nax_base = rdyn_fake_mobile_chain->getActiveJointsNumber();
  m_rdyn_full_chain = rdyn::joinChains(rdyn_fake_mobile_chain, rdyn_fixed_chain);

  std::string e;
  // FIXME: ricontrolla a cosa serve rdyn::chain->setInputJointsName
  m_rdyn_full_chain->setInputJointsName(m_params.joints,e);
  m_joint_names = m_rdyn_full_chain->getActiveJointsName();
  RCLCPP_DEBUG(this->get_logger(), "Rdyn: OK");

  RCLCPP_DEBUG(this->get_logger(), "MPC config: start");
  int nax = m_rdyn_full_chain->getActiveJointsNumber();
  m_nax = nax;
  m_model.setPredictiveHorizon(m_params.prediction_horizon);
  m_prediction_horizon = m_params.prediction_horizon;
  m_model.setSamplingPeriod(m_params.dt);
  m_dt = m_params.dt;

  m_model.init(nax,m_params.control_horizon, m_dt);
  m_prediction_horizon = m_params.prediction_horizon;
  m_control_horizon = m_params.control_horizon;
  // Stack of tasks
  m_sot.set_n_axis(nax);
  m_sot.set_np(m_prediction_horizon);
  // Inequality array
  m_ineq_array.set_n_axis(nax);
  m_ineq_array.set_np(m_prediction_horizon);
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
  m_ub_vel.setLimits( m_rdyn_full_chain->getDQMax());
  m_lb_vel.setLimits(-m_rdyn_full_chain->getDQMax());
  m_ub_pos.setLimits( m_rdyn_full_chain->getQMax());
  m_ub_inv.setLimits( m_rdyn_full_chain->getQMax(), m_rdyn_full_chain->getDQMax(), -m_rdyn_full_chain->getDDQMax());
  m_lb_pos.setLimits( m_rdyn_full_chain->getQMin());
  m_lb_inv.setLimits( m_rdyn_full_chain->getQMax(), -m_rdyn_full_chain->getDQMax(), m_rdyn_full_chain->getDDQMax());
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
  RCLCPP_DEBUG(this->get_logger(), "MPC config: OK");


  m_q_now.resize(nax);
  m_q.resize(nax, m_prediction_horizon);
  m_dq.resize(nax, m_prediction_horizon);
  m_ddq.resize(nax, m_prediction_horizon);

  m_target_dx.resize(m_prediction_horizon * 6);
  m_target_dx.setZero();

  m_leader_trj__pub = this->create_publisher<formation_msgs::msg::TrjOptimResults>(m_params.trj_leader_topic, 10);
  m_joint_trajectory__pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(m_params.cmd_trj_topic, 10);
  m_cmd_vel__pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(m_params.cmd_vel_topic, 10);
  m_joint_state__sub = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, [this](const sensor_msgs::msg::JointState& msg){
    std::lock_guard<std::mutex> l(m_mtx);
    for(size_t idx = 0; idx < msg.name.size(); ++idx)
    {
      m_q_now.col(0)(idx) = msg.position.at(idx);
    }
  }); // TODO: metti fra parametri il topic

  m_trj_server__action = rclcpp_action::create_server<FollowFormationTrajectory>(
                           this,
                           "follow_leader_trajectory",
                           std::bind(&LeaderMPC::handle_goal, this, _1, _2),
                           std::bind(&LeaderMPC::handle_cancel, this, _1),
                           std::bind(&LeaderMPC::handle_accepted, this, _1));
//  std::chrono::milliseconds(m_dt);
  RCLCPP_INFO(this->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LeaderMPC::on_activate(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Activate successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LeaderMPC::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_WARN(this->get_logger(), "on_deactive() not implemented properly. Skipping...");
  return CallbackReturn::SUCCESS;
}

void LeaderMPC::set_plan(const trajectory_msgs::msg::MultiDOFJointTrajectory& trj)
{
  size_t size = trj.points.size();
  m_plan.clear();
  m_plan.resize(size);
  for(size_t idx = 0; idx < size; ++idx)
  {
    m_plan.pose.at(idx) = tf2::transformToEigen(trj.points.at(idx).transforms.at(0));
    tf2::fromMsg(trj.points.at(idx).velocities.at(0), m_plan.twist.at(idx));
    tf2::fromMsg(trj.points.at(idx).accelerations.at(0), m_plan.acc.at(idx));
    m_plan.time.at(idx) = rclcpp::Time(trj.points.at(idx).time_from_start.sec,
                                       trj.points.at(idx).time_from_start.nanosec);
  }
  RCLCPP_INFO(this->get_logger(), "New plan received");
  m_plan.started = false;
}

geometry_msgs::msg::TwistStamped LeaderMPC::compute_velocity_command()
{
  auto start_ros = m_t = this->get_clock()->now();
  if(!m_plan.started)
  {
    m_plan.start = start_ros;
    m_plan.started = true;
  }
  // update joint state
  // NOTA: mi aspetto che il joint state sia aggiornato dal subscriber?

//  std::unique_lock<std::mutex> trj_lock(m_mtx);
  Eigen::Affine3d target_x;
  Eigen::VectorXd target_dx;
  for(size_t idx = 0; idx < m_prediction_horizon; ++idx)
  {
    interpolate(m_t, m_plan.start, target_dx, target_x);
    m_target_dx.segment<6>(idx, 1) = target_dx;
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
  double scaling = m_solutions(m_nax * m_prediction_horizon);

  m_model.updatePredictions(m_solutions.head(m_nax*m_prediction_horizon));
  m_model.updateState(m_solutions.head(m_nax));

  for(size_t idx = 0; idx < m_prediction_horizon; ++idx)
  {
    m_dq.col(idx) = m_solutions.segment((m_nax+1)*idx, m_nax);
    m_q.col(idx) = ((idx==0) ? m_q_now : m_q.col(idx-1)) + m_dq.col(idx)*m_dt; // FIXME: m_dt? oppure un dt legato all'invio dei messaggi?
  }

  rclcpp::Time now = this->get_clock()->now();
  //cmd_vel for base
  geometry_msgs::msg::TwistStamped vel_msg;
  vel_msg.header.stamp = now;
  vel_msg.twist.linear.x  = m_solutions(0);
  vel_msg.twist.linear.y  = m_solutions(1);
  vel_msg.twist.linear.z  = 0;
  vel_msg.twist.angular.x = 0;
  vel_msg.twist.angular.y = 0;
  vel_msg.twist.angular.z = m_solutions(2);

  // joint values for manipulator
  trajectory_msgs::msg::JointTrajectory trj_msg;
  trj_msg.header.stamp = now;
  trj_msg.joint_names = m_joint_names;
  trj_msg.points.at(0).positions = std::vector<double>(m_q.col(0).data(), m_q.col(0).data() + m_q.col(0).rows());
  trj_msg.points.at(0).velocities = std::vector<double>(m_dq.col(0).data(), m_dq.col(0).data() + m_dq.col(0).rows());

  m_joint_trajectory__pub->publish(trj_msg);

  // Parte di comunicazione con il follower
  formation_msgs::msg::TrjOptimResults sol;
  sol.trj.header.frame_id = m_map_frame;
  sol.trj.header.stamp = start_ros;
  sol.trj.joint_names = m_joint_names;
  for(size_t idx = 0; idx < m_prediction_horizon; ++idx)
  {
    sol.trj.points.at(idx).positions = std::vector<double>(m_q.data(), m_q.data() + m_q.rows() * m_q.cols());
    sol.trj.points.at(idx).velocities = std::vector<double>(m_dq.data(), m_dq.data() + m_dq.rows() * m_dq.cols());
  }
  sol.scaling = scaling;
  m_leader_trj__pub->publish(sol);

  return vel_msg;
}

void loop()
{

}

void LeaderMPC::interpolate(const rclcpp::Time& t_t, const rclcpp::Time& t_start, Eigen::VectorXd& interp_vel, Eigen::Affine3d& out)
{
  rclcpp::Duration T = t_t - t_start;
  for(size_t idx = 1; idx < m_plan.time.size(); idx++)
  {
    rclcpp::Time t_start = rclcpp::Time(T.seconds());
    if(!(    ((t_start - m_plan.time.at(idx)).seconds() < 0)
          && ((t_start - m_plan.time.at(idx-1)).seconds() >= 0)
         )
       )
    {
      continue;
    }
    double delta_time = (m_plan.time.at(idx) - m_plan.time.at(idx-1)).seconds();
    double t = (t_start - m_plan.time.at(idx-1)).seconds();
    double ratio = t / delta_time;
    Eigen::Isometry3d pose_interp;

    const Eigen::Vector3d& p0 = m_plan.pose.at(idx-1).translation();
    const Eigen::Vector3d& p1 = m_plan.pose.at(idx).translation();
    const Eigen::Vector3d& v0 = m_plan.twist.at(idx-1).head<3>();
    const Eigen::Vector3d& v1 = m_plan.twist.at(idx).head<3>();

    interp_vel.head<3>() =   3 * (2*p0 + v0 - 2*p1 + v1) * std::pow(t,2)
                           + 2 * (-3*p0 + 3*p1 - 2*v0 - v1) * t
                           + v0;
    interp_vel.tail<3>() = ratio * m_plan.twist.at(idx).tail<3>() + (1-ratio) * m_plan.twist.at(idx-1).tail<3>();

    double h00, h01, h10, h11;
    h00 = 2*std::pow(ratio,3) - 3*std::pow(ratio,2) + 1;
    h10 = std::pow(ratio,3) - 2*std::pow(ratio,2) + ratio;
    h01 = -2*std::pow(ratio,3) + 3*std::pow(ratio,2);
    h11 = std::pow(ratio,3) - std::pow(ratio,2);
    out.translation() =   h00 * m_plan.pose.at(idx-1).translation()
                                + h10 * m_plan.twist.at(idx-1).head<3>()
                                + h01 * m_plan.pose.at(idx).translation()
                                + h11 * m_plan.twist.at(idx).head<3>();
    out.linear() = Eigen::Quaterniond(m_plan.pose.at(idx-1).linear()).slerp(ratio, Eigen::Quaterniond(m_plan.pose.at(idx).linear())).toRotationMatrix();
    break;
  }
}

rclcpp_action::GoalResponse LeaderMPC::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowFormationTrajectory::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  if(this->get_current_state().label() != "Active")
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot accept request. Lifecycle state: %s", this->get_current_state().label().c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Goal request accepted");
  }
  (void)uuid;
  set_plan(goal->payload_cartesian_trajectory);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LeaderMPC::handle_cancel(const std::shared_ptr<GoalHandleFFT> goal_handle)
{
  RCLCPP_WARN(this->get_logger(), "Received request to cancel goal");
  RCLCPP_WARN(this->get_logger(), "Proper cancel not implemented yet");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LeaderMPC::handle_accepted(const std::shared_ptr<GoalHandleFFT> goal_handle)
{
  std::thread{std::bind(&LeaderMPC::update, this, _1), goal_handle}.detach();
}

void LeaderMPC::update(const std::shared_ptr<GoalHandleFFT> goal_handle)
{
  using namespace std::chrono_literals;
  rclcpp::Time t_start = get_clock()->now();
  rclcpp::Rate rate(m_dt);
  Eigen::Affine3d actual_pose;
  while(!m_tf_buffer_ptr->canTransform(m_tool_frame, m_map_frame, t_start, 10s)) this->get_clock()->sleep_for(10ms);
  geometry_msgs::msg::TransformStamped msg = m_tf_buffer_ptr->lookupTransform(m_tool_frame, m_map_frame, t_start);
  actual_pose = tf2::transformToEigen(msg);
  std::shared_ptr<FollowFormationTrajectory::Result> result = std::make_shared<FollowFormationTrajectory::Result>();
  std::shared_ptr<FollowFormationTrajectory::Feedback> feedback = std::make_shared<FollowFormationTrajectory::Feedback>();
  t_start = this->get_clock()->now();
  while(   (actual_pose.rotation() - m_plan.pose.back().rotation()).norm() > 1e-3
        || (actual_pose.translation() - m_plan.pose.back().translation()).norm() > 1e-3)
  {
    compute_velocity_command();
    feedback->header.stamp = this->get_clock()->now();
    for(size_t idx = 0; idx < m_q.rows(); ++idx)
    {
      feedback->desired.positions.at(idx) = m_q.col(0)(idx);
      feedback->desired.velocities.at(idx) = m_dq.col(0)(idx);
    }
    feedback->joint_names = m_joint_names;
    goal_handle->publish_feedback(feedback);
    if(!rate.sleep())
    {
      RCLCPP_WARN(get_logger(), "Cycle time too low! Cannot wait");
    }
  }
  rclcpp::Time t_end = get_clock()->now();
  if (rclcpp::ok())
  {
    result->error = 0;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Plan executed correctly");
  }
}

} // formation_mpc
