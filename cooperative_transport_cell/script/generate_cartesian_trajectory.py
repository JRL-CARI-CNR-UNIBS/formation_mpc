import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time as rclTime
from std_srvs.srv import Trigger
from formation_msgs.action import FollowFormationLeaderTrajectory
import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose, Quaternion, TwistStamped, PoseStamped
from moveit_msgs.msg import CartesianTrajectory, CartesianTrajectoryPoint
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation, Slerp
import random

class TrajectoryCreation(Node):
  def __init__(self):
    super().__init__("generate_trajectory_node")
    rclpy.logging.set_logger_level(self.get_logger().name, rclpy.logging.LoggingSeverity.DEBUG)

    self.declare_parameter('frame',          'object')
    self.declare_parameter('world_frame',    'map')
    self.declare_parameter('axis',           [1,1,1,0,0,0])
    self.declare_parameter('duration',       30)

    self.max_delta = [10,10,0.1,np.deg2rad(0),np.deg2rad(0),np.deg2rad(0)]
    self.srv = self.create_service(Trigger, 'generate_cartesian_trajectory', self.generate__cb)
    self.act_client = ActionClient(self, FollowFormationLeaderTrajectory, '/leader_mpc/follow_leader_trajectory')

    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    self.trj__pub: Publisher    = self.create_publisher(msg_type=CartesianTrajectory, topic="/cartesian_trajectory",qos_profile=1)
    self.interp__pub: Publisher = self.create_publisher(msg_type=TwistStamped, topic="/trajectory_generation_script/trj_vel", qos_profile=10)
    self.p_interp__pub: Publisher = self.create_publisher(msg_type=PoseStamped, topic="/trajectory_generation_script/trj_pose", qos_profile=10)

    self.rng = np.random.default_rng()

  def generate__cb(self, req, res):
    to_move__str = self.get_parameter('frame').value
    world_frame = self.get_parameter('world_frame').value
    axis = self.get_parameter('axis').value
    duration = self.get_parameter('duration').value
    print(f"On frame: {to_move__str}")
    print(f"duration: {duration}s")

    final_pose_delta = np.array([self.rng.choice([-1,1]) * (self.rng.random()*self.max_delta[idx]) * ax for idx, ax in enumerate(axis)])
    print(f"delta: {final_pose_delta[:3]}")
    print(f"delta: {final_pose_delta[3:]}")
    to_move__tf = TransformStamped()
    now = self.get_clock().now()
    while(True):
      try:
        to_move__tf = self.tf_buffer.lookup_transform(source_frame=to_move__str, target_frame=world_frame, time=rclTime(seconds=0))
        break
      except:
        self.get_logger().error(f"Cannot get transform from {world_frame} to {to_move__str}")
        # return res
    
    to_move_xyz__vec = np.array([to_move__tf.transform.translation.x, to_move__tf.transform.translation.y, to_move__tf.transform.translation.z])
    print(f"initial pose: {to_move_xyz__vec}")
    final_pose_xyz__vec = to_move_xyz__vec + final_pose_delta[:3]
    print(f"final pose: {final_pose_xyz__vec}")

    to_move_rot__scipy = Rotation.from_quat([to_move__tf.transform.rotation.x, to_move__tf.transform.rotation.y, to_move__tf.transform.rotation.z, to_move__tf.transform.rotation.w])
    final_pose_rot__scipy = Rotation.from_euler(seq="xyz", degrees=False, angles=to_move_rot__scipy.as_euler(seq="xyz", degrees=False) + np.array(final_pose_delta[3:]))
    # final_pose_rot__scipy = to_move_rot__scipy

    trj = self.interpolate(to_move_xyz__vec,
                           final_pose_xyz__vec,
                           to_move_rot__scipy,
                           final_pose_rot__scipy,
                           duration)
    
    plot_trj(trj)
    self.trj__pub.publish(trj)

    goal_trj = FollowFormationLeaderTrajectory.Goal()
    goal_trj.payload_cartesian_trajectory = trj

    self.act_client.wait_for_server()
    self.future = self.act_client.send_goal_async(goal_trj)
    
    res.success = True
    return res
    
    

  def interpolate(self, t_p0, t_p1, t_rot0: Rotation, t_rot1: Rotation, t_duration) -> CartesianTrajectory:
    n = 20
    assert(n > 2)
    all_t = np.linspace(0, t_duration, n)
    dt = all_t[1] - all_t[0]
    p0 = t_p0
    p1 = t_p1
#    v0 = np.array([0,0,0])
#    v1 = np.array([0,0,0])
    v0 = np.random.random(size=(3,)) * 0.2 # np.array([0,0,0])
    v1 = np.random.random(size=(3,)) * 0.2 # np.array([0,0,0])
    # t = 0.1

    # p_intp = np.zeros([n,3])
    trj = CartesianTrajectory()

    slerp = Slerp([all_t[0], all_t[-1]], Rotation.concatenate([t_rot0, t_rot1]))
    p_rot_old = t_rot0.as_quat()
    for idx, time in enumerate(all_t):
      t = (time - all_t[0])/(all_t[-1] - all_t[0])
      Dt = (all_t[-1] - all_t[0])
      c3 = 2*p0 + Dt*v0 - 2*p1 + Dt*v1
      c2 = -3*p0 + 3*p1 -2*Dt*v0 - Dt*v1
      c1 = Dt*v0
      c0 = p0

      p_intp = c3*t**3 + c2*t**2 + c1*t + c0

      p_rot = slerp(t).as_quat()
      pnt = CartesianTrajectoryPoint()
      pnt.time_from_start = Duration(seconds=time).to_msg()
      pnt.point.pose.position.x = p_intp[0]
      pnt.point.pose.position.y = p_intp[1]
      pnt.point.pose.position.z = p_intp[2]
      # pnt.point.pose.orientation.x = p_rot[0]
      # pnt.point.pose.orientation.y = p_rot[1]
      # pnt.point.pose.orientation.z = p_rot[2]
      # pnt.point.pose.orientation.w = p_rot[3]
      pnt.point.pose.orientation.x = t_rot0.as_quat()[0]
      pnt.point.pose.orientation.y = t_rot0.as_quat()[1]
      pnt.point.pose.orientation.z = t_rot0.as_quat()[2]
      pnt.point.pose.orientation.w = t_rot0.as_quat()[3]

      msg2: PoseStamped = PoseStamped()
      msg2.header.stamp = rclpy.time.Time(seconds=time).to_msg()
      msg2.pose.position.x = p_intp[0]
      msg2.pose.position.y = p_intp[1]
      msg2.pose.position.z = p_intp[2]
      self.p_interp__pub.publish(msg2)

      trj.points.append(pnt)

    for idx, time in enumerate(all_t):
      v_intp = np.array([0.0,0.0,0.0])
      if(idx == 0):
        v_intp[0] = (trj.points[1].point.pose.position.x - trj.points[0].point.pose.position.x)/(all_t[1] - all_t[0])/2
        v_intp[1] = (trj.points[1].point.pose.position.y - trj.points[0].point.pose.position.y)/(all_t[1] - all_t[0])/2
        v_intp[2] = (trj.points[1].point.pose.position.z - trj.points[0].point.pose.position.z)/(all_t[1] - all_t[0])/2
      elif(idx == len(all_t)-1):
        v_intp[0] = (trj.points[-1].point.pose.position.x - trj.points[-2].point.pose.position.x)/(all_t[-1] - all_t[-2])/2
        v_intp[1] = (trj.points[-1].point.pose.position.y - trj.points[-2].point.pose.position.y)/(all_t[-1] - all_t[-2])/2
        v_intp[2] = (trj.points[-1].point.pose.position.z - trj.points[-2].point.pose.position.z)/(all_t[-1] - all_t[-2])/2
      else:
        v_intp[0] = ((trj.points[idx+1].point.pose.position.x - trj.points[idx].point.pose.position.x)/(all_t[idx+1] - all_t[idx]) + (trj.points[idx].point.pose.position.x - trj.points[idx-1].point.pose.position.x)/(all_t[idx] - all_t[idx-1]))/2
        v_intp[1] = ((trj.points[idx+1].point.pose.position.y - trj.points[idx].point.pose.position.y)/(all_t[idx+1] - all_t[idx]) + (trj.points[idx].point.pose.position.y - trj.points[idx-1].point.pose.position.y)/(all_t[idx] - all_t[idx-1]))/2
        v_intp[2] = ((trj.points[idx+1].point.pose.position.z - trj.points[idx].point.pose.position.z)/(all_t[idx+1] - all_t[idx]) + (trj.points[idx].point.pose.position.z - trj.points[idx-1].point.pose.position.z)/(all_t[idx] - all_t[idx-1]))/2
      trj.points[idx].point.velocity.linear.x = v_intp[0]
      trj.points[idx].point.velocity.linear.y = v_intp[1]
      trj.points[idx].point.velocity.linear.z = v_intp[2]
      trj.points[idx].point.velocity.angular.x = 0.0 # ang_vel[0]
      trj.points[idx].point.velocity.angular.y = 0.0 # ang_vel[1]
      trj.points[idx].point.velocity.angular.z = 0.0 # ang_vel[2]
    
      msg: TwistStamped = TwistStamped()
      msg.header.stamp = rclpy.time.Time(seconds=time).to_msg()
      msg.twist.linear.x = v_intp[0]
      msg.twist.linear.y = v_intp[1]
      msg.twist.linear.z = v_intp[2]
      self.interp__pub.publish(msg)

    return trj

def plot_trj(trj: CartesianTrajectory):
  print("Plotting...")
  ax = plt.figure().add_subplot(projection='3d')
  x = np.array([pnt.point.pose.position.x for pnt in trj.points])
  y = np.array([pnt.point.pose.position.y for pnt in trj.points])
  z = np.array([pnt.point.pose.position.z for pnt in trj.points])
  ax.plot(x,y,z)
  plt.draw()
  # plt.show()


if __name__ == '__main__':
  rclpy.init()
  node = TrajectoryCreation()
  rclpy.spin(node)  
