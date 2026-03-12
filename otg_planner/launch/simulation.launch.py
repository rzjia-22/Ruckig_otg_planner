"""Launch Gazebo, Panda, controllers, and the joint-space planner stack."""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def _start_next_or_shutdown(next_actions, stage_name):
    def _handler(event, _context):
        if event.returncode == 0:
            return list(next_actions)
        return [
            LogInfo(
                msg=(
                    f'[otg_planner] {stage_name} exited with code '
                    f'{event.returncode}, stopping launch.'
                )
            ),
            EmitEvent(event=Shutdown(reason=f'{stage_name} failed')),
        ]

    return _handler


def _require_package(package_name, apt_hint=''):
    try:
        return get_package_share_directory(package_name)
    except PackageNotFoundError as exc:
        message = f"Required ROS package '{package_name}' not found."
        if apt_hint:
            message = f'{message} {apt_hint}'
        raise RuntimeError(message) from exc


def _require_package_prefix(package_name, apt_hint=''):
    try:
        return get_package_prefix(package_name)
    except PackageNotFoundError as exc:
        message = f"Required ROS package '{package_name}' not found."
        if apt_hint:
            message = f'{message} {apt_hint}'
        raise RuntimeError(message) from exc


def generate_launch_description():
    pkg_name = 'otg_planner'
    pkg_share = _require_package(pkg_name)
    ros_gz_sim_share = _require_package(
        'ros_gz_sim',
        'Try: sudo apt install ros-humble-ros-gz-sim',
    )
    moveit_panda_share = _require_package(
        'moveit_resources_panda_description',
        'Try: sudo apt install ros-humble-moveit-resources-panda-description',
    )

    planner_control_cycle = LaunchConfiguration('planner_control_cycle')
    planner_command_horizon = LaunchConfiguration('planner_command_horizon')
    planner_speed_scale = LaunchConfiguration('planner_speed_scale')
    planner_path_scale = LaunchConfiguration('planner_path_scale')
    planner_max_iterations = LaunchConfiguration('planner_max_iterations')
    enable_demo = LaunchConfiguration('enable_demo')
    enable_obstacle_scenario = LaunchConfiguration('enable_obstacle_scenario')
    obstacle_scenario = LaunchConfiguration('obstacle_scenario')
    obstacle_duration = LaunchConfiguration('obstacle_duration')
    obstacle_appear_delay = LaunchConfiguration('obstacle_appear_delay')

    xacro_file = os.path.join(pkg_share, 'urdf', 'panda.urdf.xacro')
    config_file = os.path.join(pkg_share, 'config', 'controllers.yaml')
    gz_ros2_control_lib = os.path.join(
        _require_package_prefix(
            'gz_ros2_control',
            'Try: sudo apt install ros-humble-gz-ros2-control',
        ),
        'lib',
        'libgz_ros2_control-system.so',
    )
    robot_doc = xacro.process_file(
        xacro_file,
        mappings={
            'controllers_file': config_file,
            'gz_ros2_control_plugin': gz_ros2_control_lib,
        },
    )
    robot_description = {'robot_description': robot_doc.toxml()}

    moveit_resource_root = os.path.dirname(moveit_panda_share)
    env_gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            TextSubstitution(text=f'{moveit_resource_root}:'),
            EnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', default_value=''),
        ],
    )
    env_ign_resource = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            TextSubstitution(text=f'{moveit_resource_root}:'),
            EnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', default_value=''),
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'panda', '-topic', 'robot_description', '-z', '0.6'],
        output='screen',
    )

    controller_bootstrap = ExecuteProcess(
        cmd=[
            'bash',
            os.path.join(pkg_share, 'scripts', 'spawn_controllers.sh'),
            config_file,
            '45',
        ],
        output='screen',
    )

    planner_node = Node(
        package='otg_planner',
        executable='planner',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {
                'control_cycle': ParameterValue(planner_control_cycle, value_type=float),
                'command_horizon': ParameterValue(planner_command_horizon, value_type=float),
                'speed_scale': ParameterValue(planner_speed_scale, value_type=float),
                'planning_max_iterations': ParameterValue(
                    planner_max_iterations,
                    value_type=int,
                ),
            },
        ],
        remappings=[('/joint_trajectory', '/panda_arm_controller/joint_trajectory')],
    )

    demo_client = Node(
        package='otg_planner',
        executable='demo_client',
        output='screen',
        condition=IfCondition(enable_demo),
        parameters=[
            {'use_sim_time': True},
            {
                'path_scale': ParameterValue(planner_path_scale, value_type=float),
                'startup_delay': 2.5,
            },
        ],
    )

    obstacle_publisher = Node(
        package='otg_planner',
        executable='obstacle_scenario',
        output='screen',
        condition=IfCondition(enable_obstacle_scenario),
        parameters=[
            {'use_sim_time': True},
            {
                'scenario': obstacle_scenario,
                'duration': ParameterValue(obstacle_duration, value_type=float),
                'appear_delay': ParameterValue(obstacle_appear_delay, value_type=float),
            },
        ],
    )

    start_controller_bootstrap_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=_start_next_or_shutdown([controller_bootstrap], 'spawn_entity'),
        )
    )
    start_runtime_after_bootstrap = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_bootstrap,
            on_exit=_start_next_or_shutdown(
                [planner_node, demo_client, obstacle_publisher],
                'controller bootstrap',
            ),
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'planner_control_cycle',
                default_value='0.05',
                description='Planner feedback and replan loop period in seconds.',
            ),
            DeclareLaunchArgument(
                'planner_command_horizon',
                default_value='0.10',
                description='Minimum time_from_start assigned to planned trajectory points.',
            ),
            DeclareLaunchArgument(
                'planner_speed_scale',
                default_value='0.35',
                description='Velocity scaling used to time the planned trajectory.',
            ),
            DeclareLaunchArgument(
                'planner_path_scale',
                default_value='1.0',
                description='Scale applied to the demo A->B displacement.',
            ),
            DeclareLaunchArgument(
                'planner_max_iterations',
                default_value='3500',
                description='Maximum RRT-Connect iterations per planning segment.',
            ),
            DeclareLaunchArgument(
                'enable_demo',
                default_value='true',
                description='Whether to start the demo action client.',
            ),
            DeclareLaunchArgument(
                'enable_obstacle_scenario',
                default_value='false',
                description='Whether to publish obstacle markers for replanning demos.',
            ),
            DeclareLaunchArgument(
                'obstacle_scenario',
                default_value='static',
                description='Obstacle scenario: static, moving, or mixed.',
            ),
            DeclareLaunchArgument(
                'obstacle_duration',
                default_value='12.0',
                description='Obstacle scenario duration in seconds.',
            ),
            DeclareLaunchArgument(
                'obstacle_appear_delay',
                default_value='2.0',
                description='Delay before obstacles appear.',
            ),
            env_gz_resource,
            env_ign_resource,
            SetParameter(name='use_sim_time', value=True),
            gazebo,
            clock_bridge,
            robot_state_publisher,
            spawn_entity,
            start_controller_bootstrap_after_spawn,
            start_runtime_after_bootstrap,
        ]
    )
