#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from chess_perception.msg import BoardState
from cv_bridge import CvBridge
import cv2
import numpy as np

class PieceDetectorNode(Node):
    def __init__(self):
        super().__init__('piece_detector_node')
        
        self.bridge = CvBridge()
        
        # State
        self.latest_image = None
        self.board_corners = None
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        self.geo_sub = self.create_subscription(
            BoardState,
            '/perception/board_geometry',
            self.geometry_callback,
            10)
            
        # Publishers
        self.state_pub = self.create_publisher(BoardState, '/perception/board_state', 10)
        self.debug_pub = self.create_publisher(Image, '/perception/piece_debug', 10)
        
        self.get_logger().info("Piece Detector Node Started")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_board()
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")

    def geometry_callback(self, msg):
        # Extract corners
        if len(msg.corners) == 4:
            pts = []
            for p in msg.corners:
                pts.append([p.x, p.y])
            self.board_corners = np.array(pts, dtype="float32")

    def process_board(self):
        if self.latest_image is None or self.board_corners is None:
            return

        # 1. Perspective Transform
        width, height = 400, 400 # Fixed size for analysis
        dst = np.array([
            [0, 0],
            [width - 1, 0],
            [width - 1, height - 1],
            [0, height - 1]], dtype="float32")

        M = cv2.getPerspectiveTransform(self.board_corners, dst)
        warped = cv2.warpPerspective(self.latest_image, M, (width, height))
        
        # 2. Grid Analysis
        square_h = height // 8
        square_w = width // 8
        
        pieces = []
        
        # Debug image
        debug_img = warped.copy()
        
        # Simple color heuristic: 
        # Check average intensity of center of square.
        # This is a placeholder. Real logic needs color calibration.
        
        for row in range(8):
            for col in range(8):
                y1 = row * square_h
                y2 = (row + 1) * square_h
                x1 = col * square_w
                x2 = (col + 1) * square_w
                
                # ROI (Region of Interest) - center 50%
                roi = warped[y1+10:y2-10, x1+10:x2-10]
                
                # Calculate average color
                avg_color_per_row = np.average(roi, axis=0)
                avg_color = np.average(avg_color_per_row, axis=0)
                
                # Heuristic: 
                # If saturation is high -> likely a piece (if pieces are colored)
                # If intensity is very high/low -> white/black piece
                # Here we just output 0 (empty) for now as placeholder logic
                # You would add your specific color thresholds here.
                
                piece_id = 0 # Empty
                
                # Example: Detect black vs white based on threshold
                gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                mean_val = np.mean(gray_roi)
                
                # Very naive detection
                if mean_val < 50:
                    piece_id = 2 # Black
                    cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                elif mean_val > 200:
                    piece_id = 1 # White
                    cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                pieces.append(piece_id)

        # 3. Publish State
        state_msg = BoardState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.pieces = [int(p) for p in pieces]
        state_msg.fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1" # Placeholder FEN
        
        self.state_pub.publish(state_msg)
        
        # Publish Debug
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = PieceDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
