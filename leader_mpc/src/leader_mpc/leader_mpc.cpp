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

void raise_error(const std::string& what = "")
{
  throw std::runtime_error(what);
}

namespace formation_mpc {
LeaderMPC::LeaderMPC()
{
}

void LeaderMPC::declare_parameters(const rclcpp_lifecycle::LifecycleNode::WeakPtr& node_weak)
{
  auto node = node_weak.lock();
  m_param_listener = std::make_shared<ParamListener>(node);
}

bool LeaderMPC::read_parameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
{
//  auto node = node_weak.lock();
  if(!m_param_listener)
  {
    RCLCPP_ERROR(node->get_logger(), "Error econuntered during init");
    return false;
  }
  m_params = m_param_listener->get_params();

  if(m_params.joints.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "No joint specified");
    return false;
  }
  if(m_params.command_joints.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "No command joint specified");
    return false;
  }
  if(m_params.control_interface.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "No control interface specified");
    return false;
  }
  if(m_params.state_interface.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "No state interface specified");
    return false;
  }
  return true;
}

bool LeaderMPC::init(const rclcpp_lifecycle::LifecycleNode::WeakPtr& node_weak)
{
  try
  {
    declare_parameters(node_weak);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return false;
  }

  return true;
}

void LeaderMPC::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                          std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
                          const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  m_node_weak = parent;
  auto node = parent.lock();
  m_tf_buffer_ptr = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer_ptr);
  if (this->read_parameters(node)) {raise_error("Error while reading parameters");}; //TODO: boh...}

//  m_joints_command_subscriber = node->create_subscription<CmdType>(
//                                  "~/commands", // TODO: da mettere come parametro
//                                  rclcpp::SystemDefaultsQoS(),
//                                  [this](const CmdType::SharedPtr msg) { this->m_rt_command_ptr.writeFromNonRT(msg); });

  std::string robot_description_msg = "";

  auto robot_sub = node->create_subscription<std_msgs::msg::String>(
    m_params.robot_description_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [&robot_description_msg](std_msgs::msg::String::SharedPtr msg) { robot_description_msg = msg->data; });
  while(robot_description_msg == "")
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "robot_description_msg " << robot_description_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // rdyn
  urdf::Model urdf_model;
  if(!urdf_model.initString(robot_description_msg))
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "robot description cannot be parsed");
//    return controller_interface::CallbackReturn::ERROR;
    raise_error();
    return;
  }
  if(urdf_model.getLink(m_params.base_footprint) == nullptr)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), m_params.base_footprint << " is missing from robot description");
    raise_error();
    return;
  }
  if(urdf_model.getLink(m_params.base_link) == nullptr)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), m_params.base_link << " is missing from robot description");
    raise_error();
    return;
  }
  if(urdf_model.getLink(m_params.tool_frame) == nullptr)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), m_params.tool_frame << " is missing from robot description");
    raise_error();
    return;
  }

  rdyn::ChainPtr rdyn_fixed_chain = std::make_shared<rdyn::Chain>(urdf_model,
                                                     m_params.base_footprint,
                                                     m_params.tool_frame,
                                                     m_gravity);

  urdf::Model fake_mobile_urdf;
  if(!fake_mobile_urdf.initFile(FAKE_MOBILE_BASE_PATH))
  {
    RCLCPP_ERROR(node->get_logger(), "fake base urdf file not found");
    raise_error();
    return;
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

//  k_e_f = m_params.followers.trj_tollerance;

//  if(m_params.followers.names.size() == 0)
//  {
//    RCLCPP_ERROR(node->get_logger(), "No follower specified");
//  }
//  else if (m_params.followers.names.size() != m_params.followers.nax.size())
//  {
//    RCLCPP_FATAL(node->get_logger(), "follower names and nax arrays have different dimensions");
//    raise_error();
//    return;
//  }
//  m_followers = m_params.followers.names;

//  std::transform(m_params.followers.names.begin(),
//                 m_params.followers.names.end(),
//                 m_params.followers.nax.begin(),
//                 std::inserter(m_flw_q, m_flw_q.end()),
//                 [](const std::string& s, const int i){
//    return std::make_pair<std::string const&, Eigen::VectorXd const&>(s,Eigen::VectorXd::Zero(i)); //NOTE: potrebbe essere un errore passarli per riferimento
//  });
//  std::transform(m_params.followers.names.begin(),
//                 m_params.followers.names.end(),
//                 m_params.followers.nax.begin(),
//                 std::inserter(m_flw_dq, m_flw_dq.end()),
//                 [](const std::string& s, const int i){
//    return std::make_pair<std::string const&, Eigen::VectorXd const&>(s,Eigen::VectorXd::Zero(i)); //NOTE: potrebbe essere un errore passarli per riferimento
//  });
//  std::transform(m_params.followers.names.begin(),
//                 m_params.followers.names.end(),
//                 m_params.followers.nax.begin(),
//                 std::inserter(m_flw_ddq, m_flw_ddq.end()),
//                 [](const std::string& s, const int i){
//    return std::make_pair<std::string const&, Eigen::VectorXd const&>(s,Eigen::VectorXd::Zero(i)); //NOTE: potrebbe essere un errore passarli per riferimento
//  });

  m_cartesian_velocity_limit = Eigen::VectorXd::Map(&m_params.base_max_vel[0], 3);

  // NOTE: Potrebbe essere sbagliato usare la stessa function per le callback
//  m_flw_trj_sub.resize(m_params.followers.names.size());
//  std::transform(m_params.followers.names.begin(),
//                 m_params.followers.names.end(),
//                 m_flw_trj_sub.begin(),
//                 [this, &node](const std::string& s) -> rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr {
//    std::string topic {s+"/"+m_params.followers.trj_topic};
//    return node->create_subscription<trajectory_msgs::msg::JointTrajectory>(s+"/"+m_params.followers.trj_topic, 10, [this, &s](const trajectory_msgs::msg::JointTrajectory& msg)->void{this->updateFollowerTrjCallback(msg, s);});
//  });

  m_leader_trj__pub = node->create_publisher<formation_msgs::msg::TrjOptimResults>(m_params.trj_leader_topic, 10);
  m_joint_trajectory__pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(m_params.cmd_trj_topic, 10);
  m_cmd_vel__pub = node->create_publisher<geometry_msgs::msg::TwistStamped>(m_params.cmd_vel_topic, 10);
  m_joint_state__sub = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, [this](const sensor_msgs::msg::JointState& msg){
    for(size_t idx = 0; idx < msg.name.size(); ++idx)
    {
      m_q_now.col(0)(idx) = msg.position.at(idx);
    }
  }); // TODO: metti fra parametri il topic
  RCLCPP_INFO(node->get_logger(), "configure successful");
}

void LeaderMPC::activate()
{

  // Aggiornamento m_q_now fatto da subscriber
  auto node = m_node_weak.lock();
//  Eigen::Isometry3d pose_base_in_map;
//  if(!m_tf_buffer_ptr->canTransform(m_base_footprint,
//                               m_map_frame,
//                               node->now(),
//                               rclcpp::Duration::from_seconds(10))
//     )
//  {
//    RCLCPP_ERROR_STREAM(node->get_logger(), "Cannot find transform from " << m_map_frame << " to " << m_base_footprint << ". Cannot activate controller.");
//  }
//  else
//  {
//    geometry_msgs::msg::TransformStamped tf_map_to_base = m_tf_buffer_ptr->lookupTransform(m_base_footprint,
//                                                                                      m_map_frame,
//                                                                                      node->now());
//    pose_base_in_map = tf2::transformToEigen(tf_map_to_base);
//  }

  acquireFormation();

  RCLCPP_INFO(node->get_logger(), "Activate successfully");
}

void LeaderMPC::deactivate()
{
}

void LeaderMPC::setPlan(const nav_msgs::msg::Path & path)
{
  // Set plan
  m_plan_path = path;
  for(auto pose = path.poses.begin(); pose != path.poses.end(); ++pose)

  m_is_new_plan = true;
  m_t = rclcpp::Time(0.0);
}

geometry_msgs::msg::TwistStamped LeaderMPC::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & /*robot_pose*/,
                                                                    const geometry_msgs::msg::Twist & /*robot_speed*/,
                                                                    nav2_core::GoalChecker * /*goal_checker*/)
{
  auto node = m_node_weak.lock();
  auto start = std::chrono::high_resolution_clock::now();
  auto start_ros = m_t = node->get_clock()->now();

  // update joint state
  // NOTA: mi aspetto che il joint state sia aggiornato dal subscriber?



  std::unique_lock<std::mutex> trj_lock(m_mtx);

//  trj_lock.lock(); // for m_followers_q|dq|ddq access
//  if(std::find_if(m_followers_last_message_stamp.begin(),
//                  m_followers_last_message_stamp.end(),
//                  [this](const std::pair<std::string, rclcpp::Time>& pair){
//    return (this->node->get_clock()->now() - pair.second) > k_max_delay;
//  }) != m_followers_last_message_stamp.end())
//  {
//    RCLCPP_FATAL(node->get_logger(), "Follower position not up to date -> Failure");
//    // TODO: manda riferimento 0 a tutti i follower
//    return controller_interface::return_type::ERROR;
//  }
//  std::unordered_map<std::string, Eigen::MatrixXd> flw_q(m_flw_q);
//  std::unordered_map<std::string, Eigen::MatrixXd> flw_dq(m_flw_dq);
//  std::unordered_map<std::string, Eigen::MatrixXd> flw_ddq(m_flw_ddq);
//  trj_lock.unlock();

//  Eigen::Vector6d target_dx_eig;
  // tf2::fromMsg(target_dx_msg->get(), m_target_dx);
  // m_target_x
  geometry_msgs::msg::Pose target_x__msg;
  interpolate(target_x__msg);
  tf2::fromMsg(target_x__msg, m_target_x);
  m_target_dx.head(3) = m_cartesian_velocity_limit;
  m_target_dx.tail(3) = m_cartesian_rotation_velocity_limit; // TODO: velocità all'istante precedente?


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

  rclcpp::Time now = node->get_clock()->now();
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

//  m_cmd_vel__pub->publish(vel_msg);
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

// ### Private ###
void LeaderMPC::updateFollowerTrjCallback(const trajectory_msgs::msg::JointTrajectory& msg, const std::string& name)
{
  std::lock_guard lock {m_mtx}; // released once exit scope
  auto trj_eigen = jointTrjToEigen(msg);
  m_flw_q.at(name) = trj_eigen.at(0);
  m_flw_dq.at(name) = trj_eigen.at(1);
  m_flw_ddq.at(name) = trj_eigen.at(2);
  m_followers_last_message_stamp.at(name) = rclcpp::Time(msg.header.stamp);
}

void LeaderMPC::acquireFormation()
{
  auto node = m_node_weak.lock();
  using namespace std::chrono_literals;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ptr = std::make_unique<tf2_ros::Buffer>(node->get_clock());;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer_ptr);

  for(size_t idx = 0; idx < m_followers.size(); ++idx)
  {
    rclcpp::Time t = node->get_clock()->now();
    while(!tf_buffer_ptr->canTransform(m_followers.at(idx)+"/"+m_tool_frame,
                                       this_frame(m_tool_frame),
                                       t)) std::this_thread::sleep_for(100ms);
    geometry_msgs::msg::TransformStamped pose = tf_buffer_ptr->lookupTransform(m_followers.at(idx)+"/"+m_tool_frame, this_frame(m_tool_frame), t);
    m_follower_tool_in_leader_tool.at(m_followers.at(idx)) = tf2::transformToEigen(pose);
  }
  RCLCPP_DEBUG(node->get_logger(), "Formation acquired");
}

void LeaderMPC::interpolate(geometry_msgs::msg::Pose& out)
{
  for(size_t idx = 1; idx < m_plan.points.size(); idx++)
  {
    rclcpp::Time t_start = rclcpp::Time((m_t - m_plan.points.at(idx).time_from_start).seconds());
    if(!(    ((t_start - m_plan.points.at(idx).time_from_start).seconds() < 0)
          && ((t_start - m_plan.points.at(idx-1).time_from_start).seconds() >= 0)
         )
       )
    {
      continue;
    }
    double delta_time = (m_plan.points.at(idx).time_from_start.sec - m_plan.points.at(idx-1).time_from_start.sec)
                + 1e-9 * (m_plan.points.at(idx).time_from_start.nanosec - m_plan.points.at(idx-1).time_from_start.nanosec);
    double t = (t_start - m_plan.points.at(idx).time_from_start).seconds();
    double ratio = t / delta_time;
    trajectory_msgs::msg::MultiDOFJointTrajectory pnt;
    Eigen::Isometry3d pose_next, pose_prev, pose_interp;
    tf2::fromMsg(m_plan.points.at(idx).transforms.at(0), pose_next);
    tf2::fromMsg(m_plan.points.at(idx-1).transforms.at(0), pose_prev);
    pose_interp.linear() = Eigen::Quaterniond(pose_prev.linear()).slerp(ratio, Eigen::Quaterniond(pose_next.linear())).toRotationMatrix();
    pose_interp.translation() = ratio * pose_next.translation() + (1-ratio)*pose_prev.translation();

    out = tf2::toMsg(pose_interp);
  }
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
/    RCLCPP_ERROR_STREAM(node->get_logger(), "robot description cannot be parsed");
/    return false;
/  }
/  if(urdf_model.getLink(t_base_frame) == nullptr)
/  {
/    RCLCPP_ERROR_STREAM(node->get_logger(), t_base_frame << " is missing from robot description");
/    return false;
/  }
/  if(urdf_model.getLink(t_tool_frame) == nullptr)
/  {
/    RCLCPP_ERROR_STREAM(node->get_logger(), t_tool_frame << " is missing from robot description");
/    return false;
/  }

/  rdyn::ChainPtr rdyn_fixed_chain = std::make_shared<rdyn::Chain>(urdf_model,
/                                                     t_base_frame,
/                                                     t_tool_frame,
/                                                     m_gravity);

/  urdf::Model fake_mobile_urdf;
/  if(!fake_mobile_urdf.initFile(FAKE_MOBILE_BASE_PATH))
/  {
/    RCLCPP_ERROR(node->get_logger(), "fake base urdf file not found");
/    return false;
/  }
/  rdyn::ChainPtr rdyn_fake_mobile_chain = std::make_shared<rdyn::Chain>(fake_mobile_urdf,
/                                                         m_params.map_frame,
/                                                         "mount_link", // TODO: parameter? constant?
/                                                         m_gravity);
/  t_rdyn_chain = rdyn::mergeChains(rdyn_fake_mobile_chain, rdyn_fixed_chain);
/  return true;
}*/

} // formation_mpc

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(formation_mpc::LeaderMPC, nav2_core::Controller);
