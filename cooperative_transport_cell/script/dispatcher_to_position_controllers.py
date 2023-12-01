import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from formation_msgs.action import FollowFormationLeaderTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray, Float64

base__pub: Publisher
manip__pub: Publisher

def dispatch(msg: FollowFormationLeaderTrajectory.Feedback):
  base:Float64MultiArray; manip: Float64MultiArray
  base.data = []
  manip.data = []
  for idx in range(3):
    base.data[idx] = msg.joints.positions[idx]

  for idx in range(3,len(msg.joints.positions)):
    manip.data.append(msg.joints.positions[idx])

  base__pub.publish(base)
  manip__pub.publish(manip)

if __name__ == "__main__":
  rclpy.init()
  node: Node = Node("dispatcher_to_position_controllers")
  base__pub = node.create_publisher(Float64MultiArray, "/base_position_controller/commands", 1)
  manip__pub = node.create_publisher(Float64MultiArray, "/ur_position_controller/commands", 1)
  cmd__sub: Subscription = node.create_subscription(FollowFormationLeaderTrajectory.Feedback,
                                                    "/follow_leader_trajectory/_action/feedback",
                                                    dispatch, 10)
  rclpy.spin(node)
  rclpy.shutdown()
