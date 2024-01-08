# This Python file uses the following encoding: utf-8
from launch import LaunchDescription

from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import EmitEvent, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  rviz_arg = DeclareLaunchArgument('rviz', default_value='False')
  rviz = LaunchConfiguration('rviz')

  prefix_arg = DeclareLaunchArgument('prefix', default_value='follower/')
  prefix = LaunchConfiguration('prefix')

  world_frame_arg = DeclareLaunchArgument('map', default_value='map')
  world_frame = LaunchConfiguration('map')

  create_world_frame_arg = DeclareLaunchArgument('create_world_frame', default_value='true')
  create_world_frame = LaunchConfiguration('create_world_frame')

  use_PPR_arg = DeclareLaunchArgument('use_PPR', default_value='true')
  use_PPR = LaunchConfiguration('use_PPR')

  xyz_arg = DeclareLaunchArgument('xyz', default_value='"0.0 0.0 0.0"')
  xyz = LaunchConfiguration('xyz')

  rpy_arg = DeclareLaunchArgument('rpy', default_value='"0.0 0.0 0.0"')
  rpy = LaunchConfiguration('rpy')

  launch_arguments = [rviz_arg, prefix_arg, world_frame_arg, create_world_frame_arg, use_PPR_arg, xyz_arg, rpy_arg]

  robot_description_content = Command(
      [
          PathJoinSubstitution([FindExecutable(name="xacro")]),
          " ",
          PathJoinSubstitution(
              [
                  FindPackageShare("cooperative_transport_cell"),
                  "urdf",
                  "imm.xacro",
              ]
          ),
          " name:=",                "formation_follower",
          " prefix:=",              prefix,
          " world_frame:=",         world_frame,
          " create_world_frame:=",  create_world_frame,
          " use_PPR:=",             use_PPR,
          " color_base:=",          '"0.0 0.0 1.0 1.0"', # Blue
          " xyz:=",                 xyz,
          " rpy:=",                 rpy,
          " initial_positions_file:=", os.path.join(get_package_share_directory("cooperative_transport_cell"), "config", "initial_position_follower.yaml"),
      ]
  )

  robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

  mpc_param = PathJoinSubstitution([FindPackageShare("follower_mpc"), "config", "follower_parameters.yaml"])
  control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    namespace=prefix,
    parameters=[robot_description, mpc_param],
    output="screen"
    )

  spawn_controller_mpc = Node(
      package="controller_manager",
      executable="spawner",
      namespace=prefix,
      output="screen",
      arguments=["follower_mpc"]
    )

  spawn_controller_bcast = Node(
      package="controller_manager",
      executable="spawner",
      namespace=prefix,
      output="screen",
      arguments=["follower_joint_bcast"]
    )

  start_mpc_on_bcast_activation = RegisterEventHandler(
      OnProcessExit(
        target_action=spawn_controller_bcast,
        on_exit=[spawn_controller_mpc]
      )
  )

  robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    namespace=prefix,
    parameters=[robot_description],
    remappings=[("/follower/joint_states", "/follower/follower_joint_bcast/joint_states")]
  )

  rviz_node__maybe = Node(
    package="rviz2",
    executable="rviz2",
    condition=IfCondition(rviz),
    # arguments=["--display-config", rviz_config],
  )

  node_to_launch = [
    control_node,
    start_mpc_on_bcast_activation,
    spawn_controller_bcast,
    robot_state_publisher,
    rviz_node__maybe,
  ]

  return LaunchDescription(launch_arguments + node_to_launch)
