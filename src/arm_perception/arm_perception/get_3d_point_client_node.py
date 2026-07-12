"""
Get 3d Point (Action Client)

Requests a 3D point of a desired object from action server and prints the result with feedback every second.

use this node as a client or send goal directly using action send goal command:
ros2 action send_goal /get_3d_point arm_interfaces/action/Get3DPoint "{class_name: 'cups'}" --feedback
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Point

from arm_interfaces.action import Get3DPoint

class Get3DPointClient(Node):
    def __init__(self):
        super().__init__('get_3d_point_client_node')

        # Create action client
        self._action_client = ActionClient(self, Get3DPoint, 'get_3d_point')

    def send_request(self, class_name):
        """Send a request to the Get3DPoint action server."""
        goal_msg = Get3DPoint.Goal()
        goal_msg.class_name = class_name

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.current_status}')

    def get_result_callback(self, future):
        """Handle the result from the action server."""
        result = future.result().result
        if result.success:
            position_3d = result.position_3d
            self.get_logger().info(f'3D position of object: {position_3d}')
        else:
            self.get_logger().info(f'Failed to get 3D position: {result.message}')

def main(args=None):
    rclpy.init(args=args)

    get_3d_point_client = Get3DPointClient()

    # Example usage: Request 3D point for an object with class name "cups"
    get_3d_point_client.send_request("cups")

    rclpy.spin(get_3d_point_client)

    get_3d_point_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
