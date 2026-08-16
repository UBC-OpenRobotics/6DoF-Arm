"""
Get 3d Point Node (Action Server)

Get 3d point of a detected object from 3d localization node.

send goal through CLI (no feedback for now):
ros2 action send\_goal /get\_3d\_point arm\_interfaces/action/Get3DPoint "{class\_name: 'cups'}"
"""
import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer

from geometry_msgs.msg import Point

from arm_interfaces.msg import DetectedObject, DetectedObjectArray
from arm_interfaces.action import Get3DPoint

class Get3DPointActionNode(Node):
    def __init__(self):
        super().__init__('get_3d_point_action_node')

        self.declare_parameter('localization_3d_topic', '/perception/detections_3d')

        localization_3d_topic = self.get_parameter('localization_3d_topic').value

        # Subscribe to 3D localization topic
        self._localization_3d_sub = self.create_subscription(
            DetectedObjectArray,
            localization_3d_topic,
            self._localization_3d_callback,
            10
        )

        # Create action server
        self._action_server = ActionServer(
            self,
            Get3DPoint,
            'get_3d_point',
            self._handle_get_3d_point
        )

        # Store the latest 3D detections
        self._latest_detections = None
        self._latest_seen_cups = None

    def _localization_3d_callback(self, msg):
        """Callback for receiving 3D detections."""
        self._latest_detections = msg #fill in the latest detections with the received message

        #TODO: store them as last seen array for each class name.

        #only store cups detections for now

        for detected_object in msg.objects:
            if detected_object.class_name == "cups":
                self._latest_seen_cups = detected_object
        

    def _handle_get_3d_point(self, goal_handle):
        """Handle the Get3DPoint Goal."""
        request = goal_handle.request
        response = Get3DPoint.Result()

        if self._latest_detections is None:
            response.success = False
            response.message = "No 3D detections available."
            goal_handle.abort()  # Mark the goal as aborted
            return response
        
        # Find the requested object (could only be cups for now)
        if self._latest_seen_cups is None:
            response.success = False
            response.message = "No detected cups."
            goal_handle.abort()  # Mark the goal as aborted
            return response
        
        else:
            response.success = True
            response.position_3d = self._latest_seen_cups.position_3d

            goal_handle.succeed()  # Mark the goal as succeeded
            return response


        # --Find the requested object in latest detections--
        # start_time = time.perf_counter()
        # for detected_object in self._latest_detections.objects:
        #     if detected_object.class_name == request.class_name:
        #         response.success = True
        #         response.position_3d = detected_object.position_3d
                
        #         goal_handle.succeed()  # Mark the goal as succeeded
        #         return response
            
        #     end_time = time.perf_counter()
        #     elapsed_time = end_time - start_time
        #     if elapsed_time > 1.0:  # Timeout after 1 second
        #         #publish feedback
        #         feedback_msg = Get3DPoint.Feedback()
        #         feedback_msg.current_status = f"{request.class_name} still not found"
        #         goal_handle.publish_feedback(feedback_msg)

        response.success = False
        response.message = f"Object with class name {request.class_name} not found."
        goal_handle.abort()  # Mark the goal as aborted
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Get3DPointActionNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
