from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

  launch_arguments = [
    DeclareLaunchArgument('prefix',             default_value=''),
    DeclareLaunchArgument('map',                default_value='map'),
    DeclareLaunchArgument('create_world_frame', default_value='true'),
    DeclareLaunchArgument('use_PPR',            default_value='true'),
    DeclareLaunchArgument('xyz',                default_value='"0.0 0.0 0.0"'),
    DeclareLaunchArgument('rpy',                default_value='"0.0 0.0 0.0"')
  ]

  return LaunchDescription([*launch_arguments, OpaqueFunction(function=launch_setup)])

def launch_setup(context, *args, **kwargs):

  prefix             = LaunchConfiguration('prefix')
  world_frame        = LaunchConfiguration('map')
  create_world_frame = LaunchConfiguration('create_world_frame')
  use_PPR            = LaunchConfiguration('use_PPR')
  xyz                = LaunchConfiguration('xyz')
  rpy                = LaunchConfiguration('rpy')

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
          " prefix:=",              prefix,
          " world_frame:=",         world_frame,
          " create_world_frame:=",  create_world_frame,
          " use_PPR:=",             use_PPR,
          " xyz:=",                 xyz,
          " rpy:=",                 rpy
      ]
  )
  robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
  
  mpc_param = PathJoinSubstitution([FindPackageShare("leader_mpc"), "config", "leader_parameters.yaml"])
  control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[robot_description, mpc_param],
    output="screen",
    arguments=['--ros-args', '--log-level', "info"]
#    prefix=['valgrind --leak-check=yes --keep-debuginfo=yes -q --num-callers']
    )

  spawn_controller_mpc = Node(
    package="controller_manager",
    executable="spawner",
    output="screen",
    arguments=["leader_mpc", "--inactive"]
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

  # rviz_config = PathJoinSubstitution([FindPackageShare("cooperative_transport_cell"), "rviz", "leader_only.rviz"])
  # rviz2 = Node(
  #   package="rviz2",
  #   executable="rviz2",
  #   arguments=["--display-config", rviz_config],
  # )

  return [
    control_node,
    start_mpc_on_bcast_activation,
    spawn_controller_bcast,
    robot_state_publisher,
    # rviz2,
#    robot_joint_state_publisher
  ]
