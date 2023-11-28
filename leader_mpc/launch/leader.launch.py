from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

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
  leader_mpc_node = LifecycleNode(
    package="leader_mpc",
    executable="leader_mpc_node",
    name="leader_mpc_node",
    namespace="",
    parameters=[mpc_param, robot_description],
    output="screen"
  )

##### Handle transitions
  register_event_handler_for_mpc_reaches_inactive_state = launch.actions.RegisterEventHandler(
          OnStateTransition(
              target_lifecycle_node=leader_mpc_node, goal_state='inactive',
              entities=[
                  EmitEvent(event=ChangeState(
                      lifecycle_node_matcher=launch.events.matches_action(leader_mpc_node),
                      transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                  )),
              ],
          )
      )

  # Make the mpc node take the 'configure' transition.
  emit_event_to_request_that_mpc_does_configure_transition = launch.actions.EmitEvent(
      event=ChangeState(
          lifecycle_node_matcher=launch.events.matches_action(leader_mpc_node),
          transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
      )
  )
##### End transitions

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

  ###########
  ## Debug ##
  ###########

  ###########

  # robot_joint_state_publisher = Node(
  #   package="joint_state_publisher_gui",
  #   executable="joint_state_publisher_gui",
  #   parameters=[robot_description],
  # )
  robot_joint_state_publisher = Node(
    package="joint_state_publisher",
    executable="joint_state_publisher",
    parameters=[robot_description,{"zeros": {
                                      "joint_x":0.0, 
                                      "joint_y":0.0,
                                      "joint_rz":0.0,
                                      "ur/shoulder_pan_joint":0.0,
                                      "ur/shoulder_lift_joint":-2.356,
                                      "ur/elbow_joint": 1.57,
                                      "ur/wrist_1_joint": 0.0,
                                      "ur/wrist_2_joint": 0.0,
                                      "ur/wrist_3_joint": 0.0
                                    }}],
  )

  rviz_config = PathJoinSubstitution([FindPackageShare("cooperative_transport_cell"), "rviz", "leader_only.rviz"])
  rviz2 = Node(
    package="rviz2",
    executable="rviz2",
    arguments=["--display-config", rviz_config],
  )

  node_to_launch = [
    control_node,
    spawn_controller,
    robot_state_publisher,
    rviz2,
    robot_joint_state_publisher,
    leader_mpc_node,
    emit_event_to_request_that_mpc_does_configure_transition,
    register_event_handler_for_mpc_reaches_inactive_state
  ]

  return LaunchDescription(node_to_launch)
