from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

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
  leader_mpc_node = Node(
    package="leader_mpc",
    executable="leader_mpc_node",
    parameters=[mpc_param, robot_description],
    output="screen"
  )

  controller_parameters = PathJoinSubstitution([FindPackageShare("cooperative_transport_cell"), "config", "controllers_parameters.yaml"])
  control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[robot_description, controller_parameters],
    output="screen"
    )

  spawn_controller = Node(
    package="controller_manager",
    executable="spawner",
    output="screen",
    arguments=["fake_imm_controller"]
  )

  robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[robot_description],
  )

  robot_joint_state_publisher = Node(
    package="joint_state_publisher_gui",
    executable="joint_state_publisher_gui",
  )

  rviz2 = Node(
    package="rviz2",
    executable="rviz2"
  )

  node_to_launch = [
    control_node,
    spawn_controller,
    robot_state_publisher,
    rviz2,
    robot_joint_state_publisher,
    leader_mpc_node
  ]

  return LaunchDescription(node_to_launch)