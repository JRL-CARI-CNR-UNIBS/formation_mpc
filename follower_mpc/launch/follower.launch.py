# This Python file uses the following encoding: utf-8
from launch import LaunchDescription

from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import EmitEvent, RegisterEventHandler, DeclareLaunchArgument, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  launch_arguments = [
    DeclareLaunchArgument('rviz',               default_value='False'),
    DeclareLaunchArgument('prefix',             default_value='follower/'),
    DeclareLaunchArgument('map',                default_value='map'),
    DeclareLaunchArgument('create_world_frame', default_value='true'),
    DeclareLaunchArgument('use_PPR',            default_value='true'),
    DeclareLaunchArgument('xyz',                default_value='"0.0 0.0 0.0"'),
    DeclareLaunchArgument('rpy',                default_value='"0.0 0.0 0.0"')
  ]

  return LaunchDescription([*launch_arguments, OpaqueFunction(function=launch_setup)])

def launch_setup(context, *args, **kwargs):

  rviz               = LaunchConfiguration('rviz')
  prefix             = LaunchConfiguration('prefix')
  world_frame        = LaunchConfiguration('map')
  create_world_frame = LaunchConfiguration('create_world_frame')
  use_PPR            = LaunchConfiguration('use_PPR')
  xyz                = LaunchConfiguration('xyz')
  rpy                = LaunchConfiguration('rpy')

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
    output="screen",
    remappings=[(f'/{prefix.perform(context)}leader/cmd_vel','/leader/cmd_vel')]
  )

  spawn_controller_mpc = Node(
      package="controller_manager",
      executable="spawner",
      namespace=prefix,
      output="screen",
      arguments=["follower_mpc", "--inactive"], # "--inactive"
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

  return [
    control_node,
    start_mpc_on_bcast_activation,
    spawn_controller_bcast,
    robot_state_publisher,
    rviz_node__maybe,
  ]
