from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg

import launch.events

def generate_launch_description():

  # Get URDF via xacro
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
          " name:=",                "formation_leader",
          " prefix:=",              "",
          " world_frame:=",         "map",
          " create_world_frame:=",  "true",
          " use_PPR:=",             "true"
      ]
  )
  robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
  
  mpc_param = PathJoinSubstitution([FindPackageShare("leader_mpc"), "config", "leader_parameters.yaml"])
  control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[robot_description, mpc_param],
    output="screen",
#    prefix=['valgrind --leak-check=yes --keep-debuginfo=yes -q --num-callers']
    )

  spawn_controller_mpc = Node(
    package="controller_manager",
    executable="spawner",
    output="screen",
    arguments=["leader_mpc"]
  )

  spawn_controller_bcast = Node(
    package="controller_manager",
    executable="spawner",
    output="screen",
    arguments=["leader_joint_bcast"]
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
    parameters=[robot_description],
    remappings=[("/joint_states", "/leader_joint_bcast/joint_states")]
  )

  ###########
  ## Debug ##
  ###########
  # robot_joint_state_publisher = Node(
  #   package="joint_state_publisher_gui",
  #   executable="joint_state_publisher_gui",
  #   parameters=[robot_description],
  # )
  ###########

#  robot_joint_state_publisher = Node(
#    package="joint_state_publisher",
#    executable="joint_state_publisher",
#    parameters=[robot_description,
#                {"zeros": {
#                            "joint_x":0.0,
#                            "joint_y":0.0,
#                            "joint_rz":0.0,
#                            "ur/shoulder_pan_joint":0.0,
#                            "ur/shoulder_lift_joint":-2.356,
#                            "ur/elbow_joint": 1.57,
#                            "ur/wrist_1_joint": 0.0,
#                            "ur/wrist_2_joint": 0.0,
#                            "ur/wrist_3_joint": 0.0
#                          }},
#  )

  rviz_config = PathJoinSubstitution([FindPackageShare("cooperative_transport_cell"), "rviz", "leader_only.rviz"])
  rviz2 = Node(
    package="rviz2",
    executable="rviz2",
    arguments=["--display-config", rviz_config],
  )

  node_to_launch = [
    control_node,
    start_mpc_on_bcast_activation,
    spawn_controller_bcast,
    robot_state_publisher,
    rviz2,
#    robot_joint_state_publisher
  ]

  return LaunchDescription(node_to_launch)
