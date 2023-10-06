#include "leader_mpc/leader_mpc.hpp"

std::array<Eigen::MatrixXd, 3> jointTrjToEigen(const trajectory_msgs::msg::JointTrajectory& msg)
{
  std::array<Eigen::MatrixXd, 3> trj;
  Eigen::MatrixXd trj_mat(msg.joint_names.size(), msg.points.size());
  for(size_t idx {0}; idx < msg.joint_names.size(); ++idx)
  {
    trj_mat.col(idx) = Eigen::VectorXd::Map(&(msg.points.at(idx).positions[0]), msg.points.at(idx).positions.size());
  }
  trj.at(0) = trj_mat;
  trj_mat = Eigen::MatrixXd::Zero(msg.joint_names.size(), msg.points.size());
  for(size_t idx {0}; idx < msg.joint_names.size(); ++idx)
  {
    trj_mat.col(idx) = Eigen::VectorXd::Map(&(msg.points.at(idx).velocities[0]), msg.points.at(idx).velocities.size());
  }
  trj.at(1) = trj_mat;
  trj_mat = Eigen::MatrixXd::Zero(msg.joint_names.size(), msg.points.size());
  for(size_t idx {0}; idx < msg.joint_names.size(); ++idx)
  {
    trj_mat.col(idx) = Eigen::VectorXd::Map(&(msg.points.at(idx).accelerations[0]), msg.points.at(idx).accelerations.size());
  }
  trj.at(2) = trj_mat;
  return trj;
}

namespace formation_mpc {
LeaderMPC::LeaderMPC()
  : controller_interface::ControllerInterface(),
    m_rt_command_ptr(nullptr),
    m_joints_command_subscriber(nullptr)
{
  m_tf_buffer_ptr = std::make_unique<tf2_ros::Buffer>(this->get_node()->get_clock());
  m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer_ptr);
}

void LeaderMPC::declare_parameters()
{
  m_param_listener = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn LeaderMPC::read_parameters()
{
  if(!m_param_listener)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error econuntered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  m_params = m_param_listener->get_params();

  if(m_params.leader.joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No joint specified");
    return controller_interface::CallbackReturn::ERROR;
  }
  if(m_params.leader.command_joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No command joint specified");
    return controller_interface::CallbackReturn::ERROR;
  }
  if(m_params.leader.control_interface.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No control interface specified");
    return controller_interface::CallbackReturn::ERROR;
  }
  if(m_params.leader.state_interface.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No state interface specified");
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeaderMPC::on_init()
{
  try
  {
    declare_parameters();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeaderMPC::on_configure(const rclcpp_lifecycle::State &previous_state)
{
  if (auto ret = this->read_parameters(); ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }
  m_joint_state_interface.resize(m_allowed_interface_types.size());
  m_joint_command_interface.resize(m_allowed_interface_types.size());

  m_joints_command_subscriber = get_node()->create_subscription<CmdType>(
                                  "~/commands", // TODO: da mettere come parametro
                                  rclcpp::SystemDefaultsQoS(),
                                  [this](const CmdType::SharedPtr msg) { this->m_rt_command_ptr.writeFromNonRT(msg); });

  std::string robot_description_msg = "";

  auto robot_sub = get_node()->create_subscription<std_msgs::msg::String>(
    m_params.leader.robot_description_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [&robot_description_msg](std_msgs::msg::String::SharedPtr msg) { robot_description_msg = msg->data; });
  while(robot_description_msg == "")
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "robot_description_msg " << robot_description_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // rdyn
  urdf::Model urdf_model;
  if(!urdf_model.initString(robot_description_msg))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "robot description cannot be parsed");
    return controller_interface::CallbackReturn::ERROR;
  }
  if(urdf_model.getLink(m_params.leader.base_footprint) == nullptr)
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), m_params.leader.base_footprint << " is missing from robot description");
    return controller_interface::CallbackReturn::ERROR;
  }
  if(urdf_model.getLink(m_params.leader.base_link) == nullptr)
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), m_params.leader.base_link << " is missing from robot description");
    return controller_interface::CallbackReturn::ERROR;
  }
  if(urdf_model.getLink(m_params.leader.tool_frame) == nullptr)
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), m_params.leader.tool_frame << " is missing from robot description");
    return controller_interface::CallbackReturn::ERROR;
  }

  rdyn::ChainPtr rdyn_fixed_chain = std::make_shared<rdyn::Chain>(urdf_model,
                                                     m_params.leader.base_footprint,
                                                     m_params.leader.tool_frame,
                                                     m_gravity);

  urdf::Model fake_mobile_urdf;
  if(!fake_mobile_urdf.initFile(FAKE_MOBILE_BASE_PATH))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "fake base urdf file not found");
    return controller_interface::CallbackReturn::ERROR;
  }
  m_nax_arm = rdyn_fixed_chain->getActiveJointsNumber();
  rdyn::ChainPtr rdyn_fake_mobile_chain = std::make_shared<rdyn::Chain>(fake_mobile_urdf,
                                                         m_params.leader.map_frame,
                                                         "mount_link", // TODO: parameter? constant?
                                                         m_gravity);
  m_nax_base = rdyn_fake_mobile_chain->getActiveJointsNumber();
  m_rdyn_full_chain = rdyn::mergeChains(rdyn_fake_mobile_chain, rdyn_fixed_chain);

  std::string e;
  // FIXME: ricontrolla a cosa serve rdyn::chain->setInputJointsName
  m_rdyn_full_chain->setInputJointsName(m_params.leader.joints,e);

  int nax = m_rdyn_full_chain->getActiveJointsNumber();
  m_nax = nax;
  m_model.setPredictiveHorizon(m_params.leader.prediction_horizon);
  m_prediction_horizon = m_params.leader.prediction_horizon;
  m_model.setSamplingPeriod(m_params.leader.dt);
  m_dt = m_params.leader.dt;

  m_model.init(nax,m_params.leader.control_horizon, m_dt);
  m_prediction_horizon = m_params.leader.prediction_horizon;
  m_control_horizon = m_params.leader.control_horizon;
  // Stack of tasks
  m_sot.set_n_axis(nax);
  m_sot.set_np(m_prediction_horizon);
  // Inequality array
  m_ineq_array.set_n_axis(nax);
  m_ineq_array.set_np(m_prediction_horizon);
  // Create task pointers
  m_minimize_acc.init(&m_model);
  m_minimize_vel.init(&m_model);
  m_cartesian_leader_task.init(&m_model, m_rdyn_full_chain.get(), {1,1,1,1,1,1}); // activate all cartesian axis
  m_cartesian_leader_task.activateScaling(m_params.leader.scaling.active);

  m_sot.clear();
  m_sot.taskPushBack(&m_minimize_acc, std::pow(k_task_level_multiplier, m_params.leader.hierarchy.at(0)-1));
  m_sot.taskPushBack(&m_minimize_vel, std::pow(k_task_level_multiplier, m_params.leader.hierarchy.at(1)-1));
  m_sot.taskPushBack(&m_minimize_vel, std::pow(k_task_level_multiplier, m_params.leader.hierarchy.at(2)-1));

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
  m_max_scaling.setLimits(m_params.leader.scaling.limit);
  m_ineq_array.addConstraint(&m_lb_inv);
  m_ineq_array.addConstraint(&m_lb_pos);
  m_ineq_array.addConstraint(&m_ub_inv);
  m_ineq_array.addConstraint(&m_ub_pos);
  m_ineq_array.addConstraint(&m_lb_vel);
  m_ineq_array.addConstraint(&m_ub_vel);
  m_ineq_array.addConstraint(&m_lb_acc);
  m_ineq_array.addConstraint(&m_ub_acc);
  m_ineq_array.addConstraint(&m_max_scaling);

  m_cmd_vel_pub_wrapped = get_node()->create_publisher<geometry_msgs::msg::Twist>(m_params.leader.cmd_vel_topic, rclcpp::SystemDefaultsQoS());
  m_cmd_vel_pub_rt = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(m_cmd_vel_pub_wrapped);
  m_cmd_vel_pub_rt->lock();
  m_cmd_vel_pub_rt->msg_ = geometry_msgs::msg::Twist();
  m_cmd_vel_pub_rt->unlock();

  m_q_now.resize(nax);
  m_q.resize(nax, m_prediction_horizon);
  m_dq.resize(nax, m_prediction_horizon);
  m_ddq.resize(nax, m_prediction_horizon);

  m_target_dx.resize(m_prediction_horizon * 6);
  m_target_dx.setZero();

  k_e_f = m_params.leader.followers.trj_tollerance;

  if(m_params.leader.followers.names.size() == 0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No follower specified");
  }
  else if (m_params.leader.followers.names.size() != m_params.leader.followers.nax.size())
  {
    RCLCPP_FATAL(get_node()->get_logger(), "follower names and nax arrays have different dimensions");
    return controller_interface::CallbackReturn::FAILURE;
  }
  m_followers = m_params.leader.followers.names;

  std::transform(m_params.leader.followers.names.begin(),
                 m_params.leader.followers.names.end(),
                 m_params.leader.followers.nax.begin(),
                 std::inserter(m_flw_q, m_flw_q.end()),
                 [](const std::string& s, const int i){
    return std::make_pair<std::string const&, Eigen::VectorXd const&>(s,Eigen::VectorXd::Zero(i)); //NOTE: potrebbe essere un errore passarli per riferimento
  });
  std::transform(m_params.leader.followers.names.begin(),
                 m_params.leader.followers.names.end(),
                 m_params.leader.followers.nax.begin(),
                 std::inserter(m_flw_dq, m_flw_dq.end()),
                 [](const std::string& s, const int i){
    return std::make_pair<std::string const&, Eigen::VectorXd const&>(s,Eigen::VectorXd::Zero(i)); //NOTE: potrebbe essere un errore passarli per riferimento
  });
  std::transform(m_params.leader.followers.names.begin(),
                 m_params.leader.followers.names.end(),
                 m_params.leader.followers.nax.begin(),
                 std::inserter(m_flw_ddq, m_flw_ddq.end()),
                 [](const std::string& s, const int i){
    return std::make_pair<std::string const&, Eigen::VectorXd const&>(s,Eigen::VectorXd::Zero(i)); //NOTE: potrebbe essere un errore passarli per riferimento
  });

  // NOTE: Potrebbe essere sbagliato usare la stessa function per le callback
  m_flw_trj_sub.resize(m_params.leader.followers.names.size());
  std::transform(m_params.leader.followers.names.begin(),
                 m_params.leader.followers.names.end(),
                 m_flw_trj_sub.begin(),
                 [this](const std::string& s) -> rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr {
    std::string topic {s+"/"+m_params.leader.followers.trj_topic};
    return this->get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(s+"/"+m_params.leader.followers.trj_topic, 10, [this, &s](const trajectory_msgs::msg::JointTrajectory& msg)->void{this->updateFollowerTrjCallback(msg, s);});
  });
//  m_flw_trj_pub.resize(m_params.leader.followers.names.size());
//  std::transform(m_params.leader.followers.names.begin(),
//                 m_params.leader.followers.names.end(),
//                 m_flw_trj_pub.begin(),
//                 [this](const std::string& s) -> rclcpp::Publisher<formation_msgs::msg::TrjOptimResults>::SharedPtr {
//    std::string
//  });

//  m_get_plan__sub_ptr = this->get_node()->create_subscription<nav_msgs::msg::Path>(m_params.leader.plan_topic, 10, [this](const nav_msgs::msg::Path& msg){
//    m_original_plan = msg;
//  });

  m_leader_trj_pub = this->get_node()->create_publisher<formation_msgs::msg::TrjOptimResults>(m_params.leader.trj_leader_topic, 10);

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration LeaderMPC::command_interface_configuration() const
{
  std::vector<std::string> command_interfaces_config_names;
  for (const auto & interface : m_params.leader.control_interface)
  {
    for (const auto & joint : m_params.leader.command_joints)
    {
      auto full_name = joint + "/" + interface;
      command_interfaces_config_names.push_back(full_name);
    }
  }

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    command_interfaces_config_names
  };
}

controller_interface::InterfaceConfiguration LeaderMPC::state_interface_configuration() const
{
  std::vector<std::string> state_interfaces_config_names;
  for (size_t i = 0; i < m_params.leader.state_interface.size(); ++i)
  {
    const auto & interface = m_params.leader.state_interface[i];
    for (const auto & joint : m_params.leader.joints)
    {
      auto full_name = joint + "/" + interface;
      state_interfaces_config_names.push_back(full_name);
    }
  }

  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    state_interfaces_config_names
  };
}

controller_interface::CallbackReturn LeaderMPC::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  // ### [START] Setup interfaces ###
  for (const auto & interface : m_params.leader.state_interface)
  {
    auto it =
      std::find(m_allowed_interface_types.begin(), m_allowed_interface_types.end(), interface);
    auto index = std::distance(m_allowed_interface_types.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, m_params.leader.joints, interface,
          m_joint_state_interface[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.", m_params.leader.joints.size(),
        interface.c_str(), m_joint_state_interface[index].size());
      return CallbackReturn::ERROR;
    }
  }
  for (const auto & interface : m_params.leader.control_interface)
  {
    auto it =
      std::find(m_allowed_interface_types.begin(), m_allowed_interface_types.end(), interface);
    auto index = std::distance(m_allowed_interface_types.begin(), it);
    if (!controller_interface::get_ordered_interfaces(
          command_interfaces_, m_params.leader.command_joints, interface, m_joint_command_interface[index]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu '%s' command interfaces, got %zu.", m_params.leader.joints.size(),
        interface.c_str(), m_joint_command_interface[index].size());
      return CallbackReturn::ERROR;
    }
  }
  // ### [END] Setup interfaces ###

  m_rt_command_ptr = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  for(size_t idx = 0; idx < m_params.leader.joints.size(); ++idx)
  {
    m_q.col(0)(idx) = state_interfaces_.at(idx).get_value();
  }

  Eigen::Isometry3d pose_base_in_map;
  if(!m_tf_buffer_ptr->canTransform(m_base_footprint,
                               m_map_frame,
                               get_node()->now(),
                               rclcpp::Duration::from_seconds(10))
     )
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Cannot find transform from " << m_map_frame << " to " << m_base_footprint << ". Cannot activate controller.");
    return controller_interface::CallbackReturn::FAILURE;
  }
  else
  {
    geometry_msgs::msg::TransformStamped tf_map_to_base = m_tf_buffer_ptr->lookupTransform(m_base_footprint,
                                                                                      m_map_frame,
                                                                                      get_node()->now());
    pose_base_in_map = tf2::transformToEigen(tf_map_to_base);
  }

  acquireFormation();
//  std::unordered_map<std::string, std::string> flw_robot_description(m_followers.size());
//  for(const std::string& name : m_followers)
//  {

//    auto robot_sub = get_node()->create_subscription<std_msgs::msg::String>(
//      m_params.leader.robot_description_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
//      [&flw_robot_description, &name](std_msgs::msg::String::SharedPtr msg) { flw_robot_description.at(name) = msg->data; });
//    while(flw_robot_description.at(name) == "")
//    {
//      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "robot_description_msg for " << name << ": " << flw_robot_description.at(name));
//      std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    }
//    loadFollowerMobileChain(name, m_base_footprint, m_tool_frame, flw_robot_description.at(name), m_flw_rdyn_chain_map.at(name));
//  }

  RCLCPP_INFO(get_node()->get_logger(), "Activate successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeaderMPC::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  m_rt_command_ptr = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LeaderMPC::update( const rclcpp::Time & time,
                                          const rclcpp::Duration & period)
{
  auto start = std::chrono::high_resolution_clock::now();
  auto start_ros = get_node()->get_clock()->now();

  // update joint state
  std::transform(state_interfaces_.begin(), state_interfaces_.end(), m_q_now.begin(),
                 [this](const auto& hw){
    return hw.get_value();
  });

  std::unique_lock<std::mutex> trj_lock (m_mtx);

  trj_lock.lock(); // for m_followers_q|dq|ddq access
  if(std::find_if(m_followers_last_message_stamp.begin(),
                  m_followers_last_message_stamp.end(),
                  [this](const std::pair<std::string, rclcpp::Time>& pair){
    return (this->get_node()->get_clock()->now() - pair.second) > k_max_delay;
  }) != m_followers_last_message_stamp.end())
  {
    RCLCPP_FATAL(get_node()->get_logger(), "Follower position not up to date -> Failure");
    // TODO: manda riferimento 0 a tutti i follower
    return controller_interface::return_type::ERROR;
  }
  std::unordered_map<std::string, Eigen::MatrixXd> flw_q(m_flw_q);
  std::unordered_map<std::string, Eigen::MatrixXd> flw_dq(m_flw_dq);
  std::unordered_map<std::string, Eigen::MatrixXd> flw_ddq(m_flw_ddq);
  trj_lock.unlock();

  // get_target_x|dx
  // TODO: implementa interpolatore: nav_msgs::msg::Path -> comandi velocità
  geometry_msgs::msg::Twist::SharedPtr* target_dx_msg = m_rt_command_ptr.readFromRT();
//  Eigen::Vector6d target_dx_eig;
  tf2::fromMsg(target_dx_msg->get(), m_target_dx);
  m_target_x

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

  if (m_cmd_vel_pub_rt->trylock())
  {
    auto & msg = m_cmd_vel_pub_rt->msg_;
    m_cmd_vel_pub_rt->msg_ = geometry_msgs::msg::Twist();
    msg.linear.x  = m_solutions(0);
    msg.linear.y  = m_solutions(1);
    msg.angular.z = m_solutions(2);
    m_cmd_vel_pub_rt->unlockAndPublish();
  }

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
  m_leader_trj_pub->publish(sol);

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "duration \n" << duration.count() << "\n");
  return controller_interface::return_type::OK;
}

// ### Private ###
void LeaderMPC::updateFollowerTrjCallback(const trajectory_msgs::msg::JointTrajectory& msg, const std::string& name)
{
  std::lock_guard lock {m_mtx}; // released once exit scope
  auto trj_eigen = jointTrjToEigen(msg);
  m_flw_q.at(name) = trj_eigen.at(0);
  m_flw_dq.at(name) = trj_eigen.at(1);
  m_flw_ddq.at(name) = trj_eigen.at(2);
  m_followers_last_message_stamp.at(name) = rclcpp::Time(msg.header.stamp);
} // lock_guard out of scope -> unlock

void LeaderMPC::acquireFormation()
{
  using namespace std::chrono_literals;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ptr = std::make_unique<tf2_ros::Buffer>(this->get_node()->get_clock());;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer_ptr);

  for(size_t idx = 0; idx < m_followers.size(); ++idx)
  {
    rclcpp::Time t = get_node()->get_clock()->now();
    while(!tf_buffer_ptr->canTransform(m_followers.at(idx)+"/"+m_tool_frame,
                                       this_frame(m_tool_frame),
                                       t)) std::this_thread::sleep_for(100ms);
    geometry_msgs::msg::TransformStamped pose = tf_buffer_ptr->lookupTransform(m_followers.at(idx)+"/"+m_tool_frame, this_frame(m_tool_frame), t);
    m_follower_tool_in_leader_tool.at(m_followers.at(idx)) = tf2::transformToEigen(pose);
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "Formation acquired");
}


/*bool LeaderMPC::loadFollowerMobileChain(const std::string& t_name,
/                             const std::string& t_base_frame,
/                             const std::string& t_tool_frame,
/                             const std::string& t_robot_description,
/                             rdyn::ChainPtr& t_rdyn_chain)
/{
/  urdf::Model urdf_model;
/  if(!urdf_model.initString(t_robot_description))
/  {
/    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "robot description cannot be parsed");
/    return false;
/  }
/  if(urdf_model.getLink(t_base_frame) == nullptr)
/  {
/    RCLCPP_ERROR_STREAM(get_node()->get_logger(), t_base_frame << " is missing from robot description");
/    return false;
/  }
/  if(urdf_model.getLink(t_tool_frame) == nullptr)
/  {
/    RCLCPP_ERROR_STREAM(get_node()->get_logger(), t_tool_frame << " is missing from robot description");
/    return false;
/  }

/  rdyn::ChainPtr rdyn_fixed_chain = std::make_shared<rdyn::Chain>(urdf_model,
/                                                     t_base_frame,
/                                                     t_tool_frame,
/                                                     m_gravity);

/  urdf::Model fake_mobile_urdf;
/  if(!fake_mobile_urdf.initFile(FAKE_MOBILE_BASE_PATH))
/  {
/    RCLCPP_ERROR(get_node()->get_logger(), "fake base urdf file not found");
/    return false;
/  }
/  rdyn::ChainPtr rdyn_fake_mobile_chain = std::make_shared<rdyn::Chain>(fake_mobile_urdf,
/                                                         m_params.leader.map_frame,
/                                                         "mount_link", // TODO: parameter? constant?
/                                                         m_gravity);
/  t_rdyn_chain = rdyn::mergeChains(rdyn_fake_mobile_chain, rdyn_fixed_chain);
/  return true;
}*/

} // formation_mpc
