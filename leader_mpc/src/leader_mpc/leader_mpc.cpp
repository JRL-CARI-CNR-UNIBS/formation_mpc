#include "leader_mpc/leader_mpc.hpp"

//std::array<Eigen::MatrixXd, 3> jointTrjToEigen(const trajectory_msgs::msg::JointTrajectory& msg)
//{
//  std::array<Eigen::MatrixXd, 3> trj;
//  Eigen::MatrixXd trj_mat(msg.joint_names.size(), msg.points.size());
//  for(size_t idx {0}; idx < msg.joint_names.size(); ++idx)
//  {
//    trj_mat.col(idx) = Eigen::VectorXd::Map(&(msg.points.at(idx).positions[0]), msg.points.at(idx).positions.size());
//  }
//  trj.at(0) = trj_mat;
//  trj_mat = Eigen::MatrixXd::Zero(msg.joint_names.size(), msg.points.size());
//  for(size_t idx {0}; idx < msg.joint_names.size(); ++idx)
//  {
//    trj_mat.col(idx) = Eigen::VectorXd::Map(&(msg.points.at(idx).velocities[0]), msg.points.at(idx).velocities.size());
//  }
//  trj.at(1) = trj_mat;
//  trj_mat = Eigen::MatrixXd::Zero(msg.joint_names.size(), msg.points.size());
//  for(size_t idx {0}; idx < msg.joint_names.size(); ++idx)
//  {
//    trj_mat.col(idx) = Eigen::VectorXd::Map(&(msg.points.at(idx).accelerations[0]), msg.points.at(idx).accelerations.size());
//  }
//  trj.at(2) = trj_mat;
//  return trj;
//}

namespace formation_mpc {
LeaderMPC::LeaderMPC(const std::string& name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode(name, options)
{
  init();
}

void LeaderMPC::declare_parameters()
{
  m_param_listener = std::make_shared<ParamListener>(this->shared_from_this());
}

bool LeaderMPC::read_parameters()
{
  if(!m_param_listener)
  {
    RCLCPP_ERROR(this->get_logger(), "Error econuntered during init");
    return false;
  }
  m_params = m_param_listener->get_params();
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
  m_tf_buffer_ptr = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer_ptr);
  if (this->read_parameters()) {
    RCLCPP_ERROR(this->get_logger(), "Error while reading parameters");
    return CallbackReturn::ERROR;
  }; //TODO: boh...

  auto robot_description__sub = this->create_subscription<std_msgs::msg::String>(
    m_params.robot_description_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [this](std_msgs::msg::String::SharedPtr msg) { m_robot_description = msg->data; });

  while(m_robot_description == "")
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "robot_description_msg " << m_robot_description);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // rdyn
  urdf::Model urdf_model;
  if(!urdf_model.initString(m_robot_description))
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "robot description cannot be parsed");
    return CallbackReturn::ERROR;
  }
  if(urdf_model.getLink(m_params.base_footprint) == nullptr)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), m_params.base_footprint << " is missing from robot description");
    return CallbackReturn::ERROR;
  }
  if(urdf_model.getLink(m_params.base_link) == nullptr)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), m_params.base_link << " is missing from robot description");
    return CallbackReturn::ERROR;
  }
  if(urdf_model.getLink(m_params.tool_frame) == nullptr)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), m_params.tool_frame << " is missing from robot description");
    return CallbackReturn::ERROR;
  }

  rdyn::ChainPtr rdyn_fixed_chain = std::make_shared<rdyn::Chain>(urdf_model,
                                                     m_params.base_footprint,
                                                     m_params.tool_frame,
                                                     m_gravity);

  urdf::Model fake_mobile_urdf;
  if(!fake_mobile_urdf.initFile(FAKE_MOBILE_BASE_PATH))
  {
    RCLCPP_ERROR(this->get_logger(), "fake base urdf file not found");
    return CallbackReturn::ERROR;
  }
  m_nax_arm = rdyn_fixed_chain->getActiveJointsNumber();
  rdyn::ChainPtr rdyn_fake_mobile_chain = std::make_shared<rdyn::Chain>(fake_mobile_urdf,
                                                         m_params.map_frame,
                                                         "mount_link", // TODO: parameter? constant?
                                                         m_gravity);
  m_nax_base = rdyn_fake_mobile_chain->getActiveJointsNumber();
  m_rdyn_full_chain = rdyn::mergeChains(rdyn_fake_mobile_chain, rdyn_fixed_chain);

  std::string e;
  // FIXME: ricontrolla a cosa serve rdyn::chain->setInputJointsName
  m_rdyn_full_chain->setInputJointsName(m_params.joints,e);

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
  std::chrono::milliseconds(m_dt);
  RCLCPP_INFO(this->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LeaderMPC::on_activate(const rclcpp_lifecycle::State& /*state*/)
{

  // Aggiornamento m_q_now fatto da subscriber
//  Eigen::Isometry3d pose_base_in_map;
//  if(!m_tf_buffer_ptr->canTransform(m_base_footprint,
//                               m_map_frame,
//                               this->now(),
//                               rclcpp::Duration::from_seconds(10))
//     )
//  {
//    RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot find transform from " << m_map_frame << " to " << m_base_footprint << ". Cannot activate controller.");
//  }
//  else
//  {
//    geometry_msgs::msg::TransformStamped tf_map_to_base = m_tf_buffer_ptr->lookupTransform(m_base_footprint,
//                                                                                      m_map_frame,
//                                                                                      this->now());
//    pose_base_in_map = tf2::transformToEigen(tf_map_to_base);
//  }

//  acquireFormation();

  RCLCPP_INFO(this->get_logger(), "Activate successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LeaderMPC::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
{
  return CallbackReturn::SUCCESS;
}

void LeaderMPC::set_plan(const trajectory_msgs::msg::MultiDOFJointTrajectory& trj)
{
  size_t size = trj.points.size();
  m_plan.clear();
  m_plan.resize(size);
  for(size_t idx = 0; idx < size; ++idx)
  {
    tf2::fromMsg(trj.points.at(idx).transforms.at(0), m_plan.pose.at(idx));
    tf2::fromMsg(trj.points.at(idx).velocities.at(0), m_plan.twist.at(idx));
    tf2::fromMsg(trj.points.at(idx).accelerations.at(0), m_plan.acc.at(idx));
    m_plan.time.at(idx) = rclcpp::Time(trj.points.at(idx).time_from_start.sec,
                                       trj.points.at(idx).time_from_start.nanosec);
  }
  m_plan.started = false;
}

geometry_msgs::msg::TwistStamped LeaderMPC::compute_velocity_command()
{
  auto start = std::chrono::high_resolution_clock::now();
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

//  Eigen::VectorXd tmp_q(m_q), tmp_dq(m_dq);
//  tmp_dq = m_solutions.head(m_nax);
//  tmp_q += tmp_dq*m_dt;

//  if(std::find_if(m_followers.begin(), m_followers.end(),[&tmp_q, &flw_q, this](const std::string& s){
//                  Eigen::BDCSVD<Eigen::MatrixXd> svd(m_flw_rdyn_chain_map.at(s)->getJacobian(flw_q.at(s).col(0)), Eigen::ComputeThinU | Eigen::ComputeThinV);
//                  Eigen::VectorXd dq_n = svd.solve(m_follower_tool_in_leader_tool.at(s)*m_rdyn_full_chain->getJacobian(tmp_q)*m_solutions.head(m_nax));
//                  return (dq_n - m_flw_dq.at(s).col(0)).norm() < k_e_f;
//  }) != m_followers.end());

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
  for(size_t idx = 1; idx < m_plan.time.size(); idx++)
  {
    rclcpp::Time t_start = rclcpp::Time(t_t);
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



} // formation_mpc
