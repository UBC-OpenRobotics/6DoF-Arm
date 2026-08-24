"""YOLOv8 object detection node.

Subscribes to preprocessed color images, runs YOLO inference, publishes DetectedObjectArray.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from arm_interfaces.msg import DetectedObject, DetectedObjectArray
from arm_interfaces.srv import DetectObjects


class YOLODetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('device', 'cpu') # TODO: consider gpu if available
        self.declare_parameter('input_topic', '/perception/color_preprocessed') #TODO: CHECK TOPIC
        self.declare_parameter('output_topic', '/perception/detections')

        model_path = self.get_parameter('model_path').value
        self._conf_thresh = self.get_parameter('confidence_threshold').value
        self._device = self.get_parameter('device').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self._bridge = CvBridge() #create a CvBridge instance for converting ROS images to OpenCV format
        self._model = None #to be loaded
        self._latest_frame = None #to be populated
        self._latest_header = None #header of the latest frame

        # Load YOLO model
        try:
            from ultralytics import YOLO
            self._model = YOLO(model_path)
            self.get_logger().info(
                f'YOLO model loaded: {model_path} (device={self._device})'
            )
        except ImportError:
            self.get_logger().error(
                'ultralytics not installed. Run: pip install ultralytics'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')

        # Subscriber and publisher
        self.create_subscription(Image, input_topic, self._image_callback, 1)
        self._det_pub = self.create_publisher(DetectedObjectArray, output_topic, 10)

        # Service for one-shot detection / detecting an object using only a single frame
        self.create_service(
            DetectObjects,
            'detect_objects',
            self._detect_objects_callback,
        )

        self.get_logger().info('YOLO detector node started')

    def _image_callback(self, msg: Image):
        # check if YOLO model is laoded
        if self._model is None:
            return

        # Convert ROS Image to OpenCV format and store the latest frame for service use
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._latest_frame = frame
        self._latest_header = msg.header

        detections = self._run_detection(frame, msg.header) # returns a list of DetectedObject messages

        det_array = DetectedObjectArray()
        det_array.header = msg.header
        det_array.objects = detections
        self._det_pub.publish(det_array)

    def _run_detection(self, frame, header=None):
        """Run YOLO inference on a single frame (in OpenCV format)."""
        results = self._model.predict(
            frame,
            conf=self._conf_thresh,
            device=self._device,
            verbose=False,
        )

        detections = [] # will be populated with DetectedObject messages to return
        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for i in range(len(boxes)):
                det = DetectedObject()
                if header is not None:
                    det.header = header
                det.class_name = self._model.names[int(boxes.cls[i])]
                det.confidence = float(boxes.conf[i])

                xyxy = boxes.xyxy[i].cpu().numpy().astype(int)
                det.bbox_2d = [int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])] # [left, top, right, bottom]

                # position_3d left as zeros - will be filled by localization_3d_node
                detections.append(det)

        return detections

    def _detect_objects_callback(self, request, response):
        """Service handler for one-shot detection."""
        # if we have a latest frame and the model is loaded, run detection
        if self._latest_frame is not None and self._model is not None:
            response.objects = self._run_detection(
                self._latest_frame, self._latest_header)
        else:
            self.get_logger().warn('No frame available for detection')
            response.objects = []
        return response


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
