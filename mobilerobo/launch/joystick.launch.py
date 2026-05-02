from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            remappings=[
                ('/cmd_vel', '/cmd_vel_joy'),
                ('/joy', '/joy')
            ],
            parameters=[{
                'axis_linear': {'x': 1},
                'axis_angular': {'yaw': 0},
                'scale_linear': {'x': 1.0},
                'scale_angular': {'yaw': 0.5},
                'scale_linear_turbo': {'x': 1.8},
                'scale_angular_turbo': {'x': 0.5},
                'publish_stamped_twist': True,
                'enable_button': 10,
                'enable_turbo_button': 9
            }]
        ),
    ])