import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from tf2_ros import StaticTransformBroadcaster
from moveit_msgs.msg import CartesianTrajectory, CartesianTrajectoryPoint
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
import tf2_ros

tf_bcast: StaticTransformBroadcaster

def cartesian__cb(msg: CartesianTrajectory) -> None:
  for idx, pnt in enumerate(msg.points):
    pnt: CartesianTrajectoryPoint

    tfstamp: TransformStamped = TransformStamped()
    tfstamp.transform.translation.x = pnt.point.pose.position.x
    tfstamp.transform.translation.y = pnt.point.pose.position.y
    tfstamp.transform.translation.z = pnt.point.pose.position.z
    tfstamp.transform.rotation.x = pnt.point.pose.orientation.x
    tfstamp.transform.rotation.y = pnt.point.pose.orientation.y
    tfstamp.transform.rotation.z = pnt.point.pose.orientation.z
    tfstamp.transform.rotation.w = pnt.point.pose.orientation.w
    tfstamp.header.frame_id = "map"
    prefix=""
    if msg.tracked_frame == "":
      prefix = "tool0_"
    else:
      prefix = f"{msg.tracked_frame}_"
    tfstamp.child_frame_id = prefix+str(idx)

    tf_bcast.sendTransform(tfstamp)

if __name__ == "__main__":
  rclpy.init()
  node: Node = rclpy.create_node("publish_tf_trajectory")

  cartesian_trj__sub: Subscription = node.create_subscription(msg_type=CartesianTrajectory, topic="/cartesian_trajectory", callback=cartesian__cb, qos_profile=10)
  tf_bcast = StaticTransformBroadcaster(node=node)
  rclpy.spin(node)

  