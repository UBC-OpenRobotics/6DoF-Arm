"""RGB Preprocessing Node

Subscribes to RGB images from RealSence camera, applies preprocessing (filters, CLAHE) through OpenCV
and possibly synchronizes 2 camera streams, and publishes preprocessed Image messages.
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ColorPreprocessingNode(Node):
    def __init__(self):
        super().__init__('color_preprocessing_node')

        self.declare_parameter('filter', 'none')
        self.declare_parameter('light_processing', 'none') 
        self.declare_parameter('input_topic', '/camera/camera/color/image_raw') #TODO: CHECK TOPIC
        self.declare_parameter('output_topic', '/perception/color_preprocessed')

        self.filter_type = self.get_parameter('filter').value
        self.light_proc = self.get_parameter('light_processing').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self._bridge = CvBridge()

        # Subscriber and publisher
        self.create_subscription(Image, input_topic, self._apply_preprocessing, 1)
        self.color_pub = self.create_publisher(Image, output_topic, 1) 

        self.get_logger().info('Color preprocessing node started with filter: '
            f'{self.filter_type}, light processing: {self.light_proc}')
    
    def _apply_preprocessing(self, msg: Image):
        # Convert ROS Image to OpenCV format
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Apply filters (TODO: make filter parameters configurable)
        # Higher values mean more smoothing/blurring. Kernel sizes must be odd.
        if self.filter_type == 'bilateral':
            frame = cv2.bilateralFilter(frame, d=9, sigmaColor=35, sigmaSpace=35)
        elif self.filter_type == 'median':
            frame = cv2.medianBlur(frame, ksize=5)
        elif self.filter_type == 'gaussian':
            frame = cv2.GaussianBlur(frame, ksize=(5, 5), sigmaX=0, sigmaY=0)

        # Apply lighting processing
        if self.light_proc == 'clahe':
            # Convert to LAB color space
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)

            # higher clipLimit means more contrast. smaller tileGridSize means more local contrast but can also amplify noise.
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            enhanced_l = clahe.apply(l)
            merged_lab = cv2.merge((enhanced_l, a, b))

            # Convert back to BGR color space
            frame = cv2.cvtColor(merged_lab, cv2.COLOR_LAB2BGR)

        # Convert back to ROS Image and publish
        preprocessed_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        preprocessed_msg.header = msg.header
        self.color_pub.publish(preprocessed_msg)

def main(args=None):
    rclpy.init(args=args)

    node = ColorPreprocessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    