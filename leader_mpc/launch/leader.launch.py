from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
  ld = LaunchDescription()

  

  # Get URDF via xacro
  robot_description_content = Command(
      [
          PathJoinSubstitution([FindExecutable(name="xacro")]),
          " ",
          PathJoinSubstitution(
              [
                  FindPackageShare("omron_app"),
                  "urdf",
                  "system.urdf.xacro",
              ]
          ),
      ]
  )
  robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
  fake_base_robot_description_content = Command(
    [
      PathJoinSubstitution([FindExecutable(name="xacro")]),
      " ",
      PathJoinSubstitution(
        [
          FindPackageShare("leader_mpc"),
          "urdf",
          "fake_base.xacro"
        ]
      )
    ]
  )
  fake_base_robot_description = {"fake_base_robot_description": ParameterValue(fake_base_robot_description_content, value_type=str)}

  mpc_param = PathJoinSubstitution([FindPackageShare("leader_mpc"), "config", "leader_parameters.yaml"])
  leader_mpc_node = Node(
    package="leader_mpc",
    executable="leader_mpc",
    parameters=[mpc_param],
    output="screen"
  )

  controller_parameters = PathJoinSubstitution([FindPackageShare("cooperative_transport_cell"), "config", "controller_parameters.yaml"])

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
#     parameters=[robot_description, controller_parameters]
    arguments=["omron_velocity_controller"]
  )
