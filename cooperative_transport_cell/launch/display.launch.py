# This Python file uses the following encoding: utf-8

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, OpaqueFunction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

  arguments = [
    DeclareLaunchArgument('rviz',            default_value='true'),
    DeclareLaunchArgument('prefix_follower', default_value='follower/'),
    DeclareLaunchArgument('prefix_leader',   default_value='')
    ]
  
  return LaunchDescription([*arguments, OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
  
  rviz =            LaunchConfiguration('rviz')
  prefix_follower = LaunchConfiguration('prefix_follower')
  prefix_leader =   LaunchConfiguration('prefix_leader')

  leader_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare("leader_mpc"),
        "launch",
        "leader.launch.py"
      ])
    ),
    launch_arguments={'xyz' : '"0.0 1.5 0.0"',
                      'rpy' : '"0.0 0.0 0.0"',
                      'prefix' : prefix_leader.perform(context)}.items(),
  )

  follower_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare("follower_mpc"),
        "launch",
        "follower.launch.py"
      ])
    ),
    launch_arguments={'xyz' : '"0.0 -1.5 0.0"',
                      'rpy' : '"0.0 0.0 0.0"',
                      'prefix' : prefix_follower.perform(context),
                      'rviz' : 'false'}.items(),
  )

  start_follower = TimerAction(
    actions=[follower_launch],
    period=1.0
  )

  ########################
  ## Active Controllers ##
  ###########################
  active_leader_mpc = ExecuteProcess(
    cmd=["ros2","control","set_controller_state","-c",f"{prefix_leader.perform(context)}controller_manager", "leader_mpc", "active"],
    output="screen"
  )

  active_follower_mpc = ExecuteProcess(
    cmd=["ros2","control","set_controller_state","-c",f"{prefix_follower.perform(context)}controller_manager", "follower_mpc", "active"],
    output="screen"
  )

  timed_active_controllers = TimerAction(
    actions=[active_leader_mpc, active_follower_mpc],
    period=5.0
  )
  ###########################

  ############
  ## Others ##
  ###########################
  generate_trj = ExecuteProcess(
    cmd=["python3",
         PathJoinSubstitution([FindPackageShare('cooperative_transport_cell'), "script", "generate_cartesian_trajectory.py"])],
    output="screen"
  )
  publish_tf = ExecuteProcess(
    cmd=["python3",
         PathJoinSubstitution([FindPackageShare('cooperative_transport_cell'), "script", "publish_tf_trajectory.py"])],
    output="screen"
  )
  fake_object = ExecuteProcess(
    cmd=["python3",
         PathJoinSubstitution([FindPackageShare('cooperative_transport_cell'), "script", "fake_object.py"])],
    output="screen"
  )
  delay_after_mm_spawn = TimerAction(
    actions=[generate_trj, publish_tf, fake_object],
    period=3.0
  )
  ###########################
  ##

  rviz_config = PathJoinSubstitution([FindPackageShare("cooperative_transport_cell"), "rviz", "cell.rviz"])
  rviz2 = Node(
    condition=IfCondition(rviz),
    package="rviz2",
    executable="rviz2",
    arguments=["--display-config", rviz_config]
  )

  return [leader_launch, start_follower, delay_after_mm_spawn, timed_active_controllers, rviz2]

  

# if __name__ == "__main__":
#     pass
