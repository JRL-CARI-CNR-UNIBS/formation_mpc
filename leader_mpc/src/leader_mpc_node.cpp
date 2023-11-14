#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "leader_mpc/leader_mpc.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<formation_mpc::LeaderMPC> leader_node = std::make_shared<formation_mpc::LeaderMPC>("leader_mpc_node");
  RCLCPP_WARN(leader_node->get_logger(), "Node object ==> constructed");
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(leader_node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
