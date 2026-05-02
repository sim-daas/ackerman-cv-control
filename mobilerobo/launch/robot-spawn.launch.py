"""
robot-spawn.launch.py — Converts the Ackerman URDF xacro to SDF and spawns it in Gazebo.

Also starts robot_state_publisher so /tf and /joint_states are live.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare('mobilerobo').find('mobilerobo')
    robot_desc_dir = get_package_share_directory('robobase_description')

    # ack.urdf.xacro lives in robobase_description/models/
    urdf_xacro = os.path.join(robot_desc_dir, 'models', 'ack.urdf.xacro')
    urdf_tmp = '/tmp/ackerman_robot.urdf'
    sdf_tmp  = '/tmp/ackerman_robot.sdf'

    # ── Launch arguments ──────────────────────────────────────────────────────
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock'
    )
    robot_name = DeclareLaunchArgument(
        'robot_name', default_value='ackerman_robot',
        description='Robot name in Gazebo'
    )
    x   = DeclareLaunchArgument('x',   default_value='0.0',  description='Spawn X')
    y   = DeclareLaunchArgument('y',   default_value='0.0',  description='Spawn Y')
    z   = DeclareLaunchArgument('z',   default_value='0.15', description='Spawn Z')
    yaw = DeclareLaunchArgument('yaw', default_value='0.0',  description='Spawn Yaw')

    # ── Convert xacro → URDF → SDF ────────────────────────────────────────────
    convert_to_sdf = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f"xacro '{urdf_xacro}' -o '{urdf_tmp}' && gz sdf -p '{urdf_tmp}' > '{sdf_tmp}'"
        ],
        output='screen'
    )

    # ── Spawn robot in Gazebo ─────────────────────────────────────────────────
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-file', sdf_tmp,
            '-allow_renaming', 'true',
            '-x',   LaunchConfiguration('x'),
            '-y',   LaunchConfiguration('y'),
            '-z',   LaunchConfiguration('z'),
            '-Y',   LaunchConfiguration('yaw'),
        ],
        output='screen'
    )

    # ── Robot State Publisher ────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_xacro]),
                value_type=str
            )
        }]
    )

    return LaunchDescription([
        use_sim_time,
        robot_name,
        x, y, z, yaw,
        convert_to_sdf,
        robot_state_publisher,
        spawn_entity,
    ])
