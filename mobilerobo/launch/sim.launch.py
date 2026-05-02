"""
sim.launch.py — Main simulation launcher for the Ackerman CV-control robot.

Starts:
  - Gazebo Ignition (server + GUI) with the shapes.sdf world
  - robot-spawn.launch.py  (robot state publisher + gz spawn)
  - ros_gz_bridge          (camera + controller topics)
  - joint_state_broadcaster spawner
  - ackermann_steering_controller spawner (after JSB is live)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = FindPackageShare(package='mobilerobo').find('mobilerobo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # ── Launch arguments ──────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='shapes.sdf',
        description='World file name inside mobilerobo/worlds/'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock'
    )
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Robot spawn X')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Robot spawn Y')

    world_file = PathJoinSubstitution([pkg_share, 'worlds', LaunchConfiguration('world')])

    # ── Environment ───────────────────────────────────────────────────────────
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_share, 'models')
    )

    # ── Gazebo server ─────────────────────────────────────────────────────────
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ── Gazebo GUI ────────────────────────────────────────────────────────────
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    # ── Robot spawn ───────────────────────────────────────────────────────────
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot-spawn.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
        }.items()
    )

    # ── GZ Bridge ─────────────────────────────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'gz-sim-bridge.yaml')
        }],
        output='screen'
    )

    # ── Controllers ───────────────────────────────────────────────────────────
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    ackermann_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Start ackermann controller only after joint_state_broadcaster is active
    delayed_ackermann = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[ackermann_spawner],
        )
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        x_arg,
        y_arg,
        set_gz_resource_path,
        gz_server,
        gz_client,
        robot_spawn,
        bridge,
        jsb_spawner,
        delayed_ackermann,
    ])
