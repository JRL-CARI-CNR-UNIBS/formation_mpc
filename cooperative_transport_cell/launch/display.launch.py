# This Python file uses the following encoding: utf-8

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  leader_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare("leader_mpc"),
        "launch",
        "leader.launch.py"
      ])
    ),
    launch_arguments={'xyz' : '"0.0 1.0 0.0"',
                      'rpy' : '"0.0 0.0 0.0"'}.items(),
  )

  follower_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([
        FindPackageShare("follower_mpc"),
        "launch",
        "follower.launch.py"
      ])
    ),
    launch_arguments={'xyz' : '"0.0 -1.0 0.0"',
                      'rpy' : '"0.0 0.0 0.0"',
                      'prefix' : 'follower/'}.items(),
  )

  start_follower = TimerAction(
    actions=[follower_launch],
    period=3.0
  )

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
    period=5.0
  )
  ###########################
  ##

  to_launch = [leader_launch, start_follower, delay_after_mm_spawn]

  return LaunchDescription(to_launch)

# if __name__ == "__main__":
#     pass
