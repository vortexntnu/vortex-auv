#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from vortex_msgs.action import NavigateWaypoints
from geometry_msgs.msg import PoseStamped
import numpy as np
import asyncio

class TestGuidanceActionClient(Node):
    def __init__(self):
        super().__init__('test_guidance_client')
        
        # Use ReentrantCallbackGroup for better async handling
        callback_group = ReentrantCallbackGroup()
        
        # Create action client
        self._action_client = ActionClient(
            self,
            NavigateWaypoints,
            'navigate_waypoints',
            callback_group=callback_group
        )
        
        self._goal_handle = None
        self._result_future = None
        
        # Test waypoints
        self.test_waypoints = [
            self.create_pose(5.0, 3.0, 2.0),  # Forward and down
            self.create_pose(7.0, 5.0, 0.0),  # Forward right and up
            self.create_pose(4.0, 3.0, 1.0),  # Back left
            self.create_pose(4.0, 7.0, 2.0),  # Forward left and down
        ]

    def create_pose(self, x: float, y: float, z: float) -> PoseStamped:
        """Create a PoseStamped message."""
        pose = PoseStamped()
        pose.header.frame_id = "world_ned"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # Set orientation to identity quaternion
        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        return pose

    def feedback_callback(self, feedback):
        """Process feedback from action server."""
        current_waypoint = feedback.feedback.current_waypoint_index
        self.get_logger().info(f'Current waypoint index: {current_waypoint}')

    async def wait_for_server(self):
        """Wait for action server to become available."""
        self.get_logger().info('Waiting for action server...')
        
        # Wait for up to 10 seconds
        for _ in range(20):  # 20 * 0.5 seconds = 10 seconds
            if self._action_client.server_is_ready():
                self.get_logger().info('Action server is available!')
                return True
            await asyncio.sleep(0.5)
        
        self.get_logger().error('Action server not available after waiting')
        return False

    async def send_goal(self):
        """Send waypoints goal to action server."""
        self.get_logger().info('Sending goal...')
        
        # Create goal message
        goal_msg = NavigateWaypoints.Goal()
        goal_msg.waypoints = self.test_waypoints
        
        # Send goal
        send_goal_future = await self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        if not send_goal_future.accepted:
            self.get_logger().error('Goal rejected!')
            return False

        self.get_logger().info('Goal accepted!')
        self._goal_handle = send_goal_future
        
        return True

    async def get_result(self):
        """Wait for and process the action result."""
        if self._goal_handle is None:
            self.get_logger().error('No goal handle available')
            return False

        self.get_logger().info('Waiting for result...')
        result_future = await self._goal_handle.get_result_async()
        
        if result_future.result.success:
            self.get_logger().info('Goal succeeded! All waypoints reached.')
            return True
        else:
            self.get_logger().error('Goal failed!')
            return False

    async def execute_waypoint_navigation(self):
        """Execute complete waypoint navigation sequence."""
        # Wait for server
        if not await self.wait_for_server():
            return False

        # Send goal
        if not await self.send_goal():
            return False

        # Wait for result while keeping node alive
        try:
            return await self.get_result()
        except Exception as e:
            self.get_logger().error(f'Error during execution: {str(e)}')
            return False

async def spin_node(node):
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            await asyncio.sleep(0.1)
    except Exception as e:
        node.get_logger().error(f'Error in spin: {str(e)}')

async def main(args=None):
    rclpy.init(args=args)
    action_client = TestGuidanceActionClient()
    
    try:
        # Create spin task
        spin_task = asyncio.create_task(spin_node(action_client))
        
        # Execute waypoint navigation
        success = await action_client.execute_waypoint_navigation()
        
        if success:
            action_client.get_logger().info('Navigation completed successfully')
        else:
            action_client.get_logger().error('Navigation failed')
            
        # Cancel spin task
        spin_task.cancel()
        try:
            await spin_task
        except asyncio.CancelledError:
            pass
            
    except Exception as e:
        action_client.get_logger().error(f'Error occurred: {str(e)}')
    finally:
        # Cleanup
        action_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())