"""
perception_control.launch.py — Launches the CV perception node and the visual-servoing
control node.

Run this AFTER sim.launch.py (or include it from sim.launch.py once sim is stable).

Usage:
    ros2 launch mobilerobo perception_control.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'
    )

    perception_node = Node(
        package='mobilerobo',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # HSV range for red (OpenCV hue 0-179)
            # Red wraps around hue=0/179, so we use two ranges merged in the node.
            'h_low1':   0,   's_low1': 100, 'v_low1': 60,
            'h_high1': 10,   's_high1': 255, 'v_high1': 255,
            'h_low2':  160,  's_low2': 100, 'v_low2': 60,
            'h_high2': 179,  's_high2': 255, 'v_high2': 255,
            # Minimum contour area (px²) to be a valid detection
            'min_contour_area': 300,
        }]
    )

    control_node = Node(
        package='mobilerobo',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # PID gains — Steering (lateral pixel error → angular.z)
            'steer_kp':  0.003,
            'steer_ki':  0.0,
            'steer_kd':  0.0005,
            # PID gains — Speed (box area error → linear.x)
            'speed_kp':  0.000015,
            'speed_ki':  0.0,
            'speed_kd':  0.000002,
            # Target bounding-box area (px²) where the robot should stop
            'target_area': 18000,
            # Maximum forward speed (m/s)
            'max_linear_speed': 0.4,
            # Maximum angular speed (rad/s)
            'max_angular_speed': 0.5,
            # Minimum forward speed to enforce while steering (Ackerman constraint)
            'min_linear_speed': 0.05,
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        perception_node,
        control_node,
    ])
