#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        
        self.camera_id = self.get_parameter('camera_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        
        # Initialize Camera
        self.cap = cv2.VideoCapture(self.camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open camera {self.camera_id}")
        else:
            self.get_logger().info(f"Camera {self.camera_id} initialized at {self.width}x{self.height}")

        self.bridge = CvBridge()
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Services
        self.capture_srv = self.create_service(Trigger, '/camera/capture', self.capture_callback)
        
        # Timer for continuous publishing (optional, can be on-demand)
        # For chess, we might only want to capture when requested, but streaming is good for debug
        self.timer = self.create_timer(1.0/self.fps, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            self.image_pub.publish(msg)

    def capture_callback(self, request, response):
        # Flush buffer
        for _ in range(5):
            self.cap.read()
            
        ret, frame = self.cap.read()
        if ret:
            response.success = True
            response.message = "Image captured"
            # We could also publish this specific frame to a 'captured' topic if needed
        else:
            response.success = False
            response.message = "Failed to capture image"
        return response

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
