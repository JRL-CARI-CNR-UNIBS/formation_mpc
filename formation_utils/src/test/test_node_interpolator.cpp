#include "formation_utils/utils.hpp"

#include <moveit_msgs/msg/cartesian_trajectory.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

formation_mpc::utils::Interpolator inter;

using namespace std::chrono_literals;
using namespace std::placeholders;

formation_mpc::utils::TrajectoryRawPtr trj__ptr;
bool start {false};
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub2;

void callback(const moveit_msgs::msg::CartesianTrajectory& msg){
    inter = formation_mpc::utils::Interpolator::from_msg(msg, trj__ptr);
    start = true;
    std::cout << "Start" << std::endl;
    Eigen::Vector6d vv;
    Eigen::Affine3d pp;
    for(double tt = 0; tt < trj__ptr->time.back().seconds(); tt+=0.1)
    {
      std::cout << tt << std::endl;
      inter.interpolate(rclcpp::Time(static_cast<uint64_t>(tt * 1e9)), rclcpp::Time(0), vv, pp);
      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(tt * 1e9));
      msg.pose = tf2::toMsg(pp);
      geometry_msgs::msg::TwistStamped msg2;
      msg2.header.stamp = rclcpp::Time(static_cast<uint64_t>(tt * 1e9));
      msg2.twist = tf2::toMsg(vv);
      pub->publish(msg);
      pub2->publish(msg2);
    }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_node");
  formation_mpc::utils::Trajectory trj;
  trj__ptr = &trj;
  pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose_2",10);
  pub2 = node->create_publisher<geometry_msgs::msg::TwistStamped>("/target_twist_2",10);
  rclcpp::Subscription<moveit_msgs::msg::CartesianTrajectory>::SharedPtr sub = node->create_subscription<moveit_msgs::msg::CartesianTrajectory>("/cartesian_trajectory", 10, callback);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
