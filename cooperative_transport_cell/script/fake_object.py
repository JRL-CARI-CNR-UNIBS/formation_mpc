# This Python file uses the following encoding: utf-8

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time as rclTime
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

from scipy.spatial.transform import Rotation as R

if __name__ == '__main__':
  rclpy.init()
  node = Node("fake_object_node")

  node.declare_parameter(name="center_to_leader", value=1.0)
  node.declare_parameter(name="center_to_follower", value=1.0)
  node.declare_parameter(name="leader_tool_frame", value="ur/tool0")
  node.declare_parameter(name="follower_tool_frame", value="/follower/grasp_point")
  node.declare_parameter(name="object_frame", value="object")
  node.declare_parameter(name="map_frame", value="map")

  center_to_leader = node.get_parameter("center_to_leader").value
  center_to_follower = node.get_parameter("center_to_follower").value
  leader_frame = node.get_parameter("leader_tool_frame").value
  follower_frame = node.get_parameter("follower_tool_frame").value
  object_frame = node.get_parameter("object_frame").value
  map_frame = node.get_parameter("map_frame").value

  tf2_buffer = Buffer()
  tf2_listener = TransformListener(tf2_buffer, node)
  tf2_static_bcast = StaticTransformBroadcaster(node)

  # GET: Map -> leader
  future_from_leader_to_map = tf2_buffer.wait_for_transform_async(target_frame=leader_frame, source_frame=map_frame,time=rclTime(seconds=0))
  rclpy.spin_until_future_complete(node=node, future=future_from_leader_to_map)
  if(not future_from_leader_to_map.done() or not future_from_leader_to_map.result()):
    node.get_logger().error("Non dovresti essere qui!")
    exit(1)
  T_from_leader_to_map: TransformStamped = tf2_buffer.lookup_transform(target_frame=leader_frame, source_frame=map_frame, time=rclTime(seconds=0))

  # SEND: leader -> object
  msg__lead_to_obj = TransformStamped()
  msg__lead_to_obj.header.stamp = node.get_clock().now().to_msg()
  msg__lead_to_obj.header.frame_id = leader_frame
  msg__lead_to_obj.child_frame_id = object_frame
  msg__lead_to_obj.transform.rotation.x = T_from_leader_to_map.transform.rotation.x
  msg__lead_to_obj.transform.rotation.y = T_from_leader_to_map.transform.rotation.y
  msg__lead_to_obj.transform.rotation.z = T_from_leader_to_map.transform.rotation.z
  msg__lead_to_obj.transform.rotation.w = T_from_leader_to_map.transform.rotation.w
  msg__lead_to_obj.transform.translation.x = 0.0
  msg__lead_to_obj.transform.translation.y = 0.0
  msg__lead_to_obj.transform.translation.z = center_to_leader
  tf2_static_bcast.sendTransform(msg__lead_to_obj)

  ## GET: follower/tool0 -> object
  future_from_obj_to_follow = tf2_buffer.wait_for_transform_async(source_frame="follower/ur/tool0", target_frame=object_frame,time=rclTime(seconds=0))
  rclpy.spin_until_future_complete(node=node, future=future_from_obj_to_follow)
  if(not future_from_obj_to_follow.done() or not future_from_obj_to_follow.result()):
    node.get_logger().error("Non dovresti essere qui!")
    exit(1)
  T_from_obj_to_follow: TransformStamped = tf2_buffer.lookup_transform(source_frame="follower/ur/tool0", target_frame=object_frame, time=rclTime(seconds=0))

  ## SEND: object -> follower/grasp
  msg__obj_to_fgrasp = TransformStamped()
  msg__obj_to_fgrasp.header.stamp = node.get_clock().now().to_msg()
  msg__obj_to_fgrasp.header.frame_id = object_frame
  msg__obj_to_fgrasp.child_frame_id = follower_frame
  msg__obj_to_fgrasp.transform.rotation.x = T_from_obj_to_follow.transform.rotation.x
  msg__obj_to_fgrasp.transform.rotation.y = T_from_obj_to_follow.transform.rotation.y
  msg__obj_to_fgrasp.transform.rotation.z = T_from_obj_to_follow.transform.rotation.z
  msg__obj_to_fgrasp.transform.rotation.w = T_from_obj_to_follow.transform.rotation.w
  msg__obj_to_fgrasp.transform.translation.x = 0.0
  msg__obj_to_fgrasp.transform.translation.y = -center_to_follower
  msg__obj_to_fgrasp.transform.translation.z = 0.0
  # tf2_static_bcast.sendTransform(msg__obj_to_fgrasp)
  tf2_static_bcast.sendTransform([msg__lead_to_obj, msg__obj_to_fgrasp])

  print("###### Script end! ######")

  rclpy.spin(node)
  rclpy.shutdown()
