"""
Get 3D Point Node, send the 3d position of a requested object in base_link frame (TF happens in localization_3d_node)
"""
import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer

from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String

from arm_interfaces.msg import DetectedObject, DetectedObjectArray
from arm_interfaces.action import Get3DPoint

class Get3DPointActionNode(Node):
    def __init__(self):
        super().__init__('vision_orchestrator')

        self.declare_parameter('localization_3d_topic', '/perception/detections_3d')
        # Object request/response topics
        self.declare_parameter('object_request_topic', '/vision/find_request')
        self.declare_parameter('object_point_topic', '/vision/object_point')

        localization_3d_topic = self.get_parameter('localization_3d_topic').value
        object_request_topic = self.get_parameter('object_request_topic').value
        object_point_topic = self.get_parameter('object_point_topic').value

        # Subscribe to 3D localization topic
        self._localization_3d_sub = self.create_subscription(
            DetectedObjectArray,
            localization_3d_topic,
            self._localization_3d_callback,
            10
        )

        # Subscribe to object request topic
        self._object_request_sub = self.create_subscription(
            String,
            object_request_topic,
            self._object_request_callback,
            10
        )

        # Publishers
        self._object_point_pub = self.create_publisher(
            PointStamped,
            object_point_topic,
            10
        )

        # Store the latest 3D detections
        self._latest_detections = None
        self._latest_seen_cup = None #TODO: modify to store last seen array for each class name (cup, bell, canister_nozzle)

        self.get_logger().info(
            f"VisionOrchestrator initialized."
        )



    # -- Callbacks --
    def _localization_3d_callback(self, msg):
        """Callback for receiving 3D detections."""
        self._latest_detections = msg #fill in the latest detections with the received message

        #TODO: store them as last seen array for each class name.

        #only store cups detections for now

        for detected_object in msg.objects:
            if detected_object.class_name == "cup":
                self._latest_seen_cup = detected_object
    

    def _object_request_callback(self, msg): 
        """Send the 3D point for the requested object."""
        # request = goal_handle.request
        # response = Get3DPoint.Result()

        request = msg.data
        response = PointStamped()

        if self._latest_detections is None:
            # response.success = False
            # response.message = "No 3D detections available."
            # goal_handle.abort()  # Mark the goal as aborted

            self.get_logger().warn("No 3D detections available.")
        
        # Find the requested object (could only be cups for now)
        if self._latest_seen_cup is None:
            # response.success = False
            # response.message = "No detected cups."
            # goal_handle.abort()  # Mark the goal as aborted
            self.get_logger().warn("No detected cup.")
        
        else:
            # response.success = True
            # response.position_3d = self._latest_seen_cup.position_3d

            # goal_handle.succeed()  # Mark the goal as succeeded

            response.header.frame_id = self._latest_seen_cup.header.frame_id
            response.header.stamp = self._latest_seen_cup.header.stamp
            response.point = self._latest_seen_cup.position_3d


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

        # response.success = False
        # response.message = f"Object with class name {request.class_name} not found."
        # goal_handle.abort()  # Mark the goal as aborted

        # Publish the requested 3D point
        self._object_point_pub.publish(response)

def main(args=None):
    rclpy.init(args=args)
    node = Get3DPointActionNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
