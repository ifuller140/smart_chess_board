#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from chess_perception.msg import BoardState
from chess_logic.srv import RequestMove
# We will assume a service or action for motion planning exists
# from gantry_control.srv import PickAndPlace (We will define this next)

class GameManagerNode(Node):
    def __init__(self):
        super().__init__('game_manager_node')
        
        # State
        self.state = "IDLE" # IDLE, WAITING_PLAYER, PROCESSING, MOVING
        self.current_fen = chess.STARTING_FEN if 'chess' in globals() else "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        
        # Subscribers
        self.clock_sub = self.create_subscription(Bool, '/limit_switch/clock_hit', self.clock_callback, 10)
        self.board_sub = self.create_subscription(BoardState, '/perception/board_state', self.board_callback, 10)
        
        # Clients
        self.capture_client = self.create_client(Trigger, '/camera/capture')
        self.engine_client = self.create_client(RequestMove, '/chess_engine/request_move')
        
        # Publishers
        self.state_pub = self.create_publisher(String, '/game_manager/state', 10)
        
        self.get_logger().info("Game Manager Started. State: IDLE")

    def clock_callback(self, msg):
        if msg.data and self.state in ["IDLE", "WAITING_PLAYER"]:
            self.get_logger().info("Clock hit! Player made a move.")
            self.state = "PROCESSING"
            self.trigger_perception()

    def board_callback(self, msg):
        # We only care about board state updates when we are processing
        # In a real app, we'd compare this msg.fen to self.current_fen to validate the move
        pass

    def trigger_perception(self):
        if not self.capture_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Camera service not available")
            return
            
        req = Trigger.Request()
        future = self.capture_client.call_async(req)
        future.add_done_callback(self.perception_done)

    def perception_done(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info("Board captured. Requesting engine move...")
                self.request_engine_move()
            else:
                self.get_logger().error("Capture failed")
                self.state = "WAITING_PLAYER"
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def request_engine_move(self):
        if not self.engine_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Engine service not available")
            return

        req = RequestMove.Request()
        req.fen = self.current_fen # In reality, we'd use the NEW FEN from perception
        
        future = self.engine_client.call_async(req)
        future.add_done_callback(self.engine_done)

    def engine_done(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"Engine wants to move: {res.best_move_uci}")
                self.state = "MOVING"
                # Here we would call the Motion Planner
                # self.execute_move(res.best_move_uci)
                # For now, just log it and go back to waiting
                self.state = "WAITING_PLAYER"
            else:
                self.get_logger().warn("Engine failed to find move")
                self.state = "WAITING_PLAYER"
        except Exception as e:
            self.get_logger().error(f"Engine call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GameManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import chess # Lazy import for default FEN
    main()
