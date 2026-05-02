#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import time

class AutoNavigator(Node):
    def __init__(self):
        super().__init__('auto_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, w):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = w
        
        self.get_logger().info(f'Waiting for action server to send goal: x={x}, y={y}')
        self._action_client.wait_for_server()
        
        self.get_logger().info(f'Sending goal request...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        self.goal_done = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Uncomment to see distance remaining:
        # self.get_logger().info(f'Distance remaining: {feedback.distance_remaining}')

def main(args=None):
    rclpy.init(args=args)
    navigator = AutoNavigator()

    # The assignment requires entering, stopping at a rack, and returning.
    # Initial spawn in Gazebo is at x=0, y=5.0, but map origin (0,0) is at Gazebo (0, 10.0).
    # Therefore, Map Frame coordinates:
    # Spawn/Return = (0, -5.0)
    # Rack location = Gazebo (0.0, 0.0) -> Map (0.0, -10.0)
    
    # 1. Start / Enter
    print("Navigating to Rack (Map Frame: x=0.0, y=-10.0)")
    navigator.goal_done = False
    navigator.send_goal(0.0, -15.0, 1.0)
    
    while rclpy.ok() and not navigator.goal_done:
        rclpy.spin_once(navigator)
        
    print("Reached Rack. Waiting for 5 seconds...")
    time.sleep(5)
    
    # 2. Return to Original Position (Map Frame: x=0.0, y=-5.0)
    print("Navigating back to entry point (Map Frame: x=0.0, y=-5.0)")
    navigator.goal_done = False
    navigator.send_goal(0.0, -5.0, 1.0)
    
    while rclpy.ok() and not navigator.goal_done:
        rclpy.spin_once(navigator)
        
    print("Mission Complete!")
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
