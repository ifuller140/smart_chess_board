#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from chess_interfaces.srv import RequestMove
import chess
import chess.engine
import random

class ChessEngineNode(Node):
    def __init__(self):
        super().__init__('chess_engine_node')
        
        self.declare_parameter('engine_path', '/usr/games/stockfish') # Default linux path
        self.declare_parameter('difficulty', 1) # 0=Random, 1-20=Stockfish Level
        
        self.engine_path = self.get_parameter('engine_path').value
        self.difficulty = self.get_parameter('difficulty').value
        
        self.engine = None

        # Try to init stockfish
        try:
            self.engine = chess.engine.SimpleEngine.popen_uci(self.engine_path)
            if self.difficulty > 0:
                skill = min(20, max(0, self.difficulty))
                self.engine.configure({"Skill Level": skill})
            self.get_logger().info(
                f"Stockfish engine started: {self.engine_path} "
                f"(skill={self.difficulty if self.difficulty > 0 else 'random'})")
        except Exception as e:
            self.get_logger().warn(f"Could not start Stockfish: {e}. Will use Random/Simple fallback.")

        self.srv = self.create_service(RequestMove, '/chess_engine/request_move', self.move_callback)

    def move_callback(self, request, response):
        board = chess.Board(request.fen)
        
        if board.is_game_over():
            response.success = False
            response.best_move_uci = ""
            return response

        best_move = None
        
        if self.engine and self.difficulty > 0:
            try:
                think_time = float(request.think_time_s) if request.think_time_s > 0 else 1.0
                think_time = max(0.5, think_time)
                result = self.engine.play(board, chess.engine.Limit(time=think_time))
                best_move = result.move
            except Exception as e:
                self.get_logger().error(f"Engine error: {e}")
        
        used_fallback = False
        if best_move is None:
            # Fallback: Random legal move
            used_fallback = True
            legal_moves = list(board.legal_moves)
            if legal_moves:
                best_move = random.choice(legal_moves)

        response.used_fallback = used_fallback
        if best_move:
            response.success = True
            response.best_move_uci = best_move.uci()
            if used_fallback:
                self.get_logger().warn(f"Stockfish unavailable — random fallback move: {best_move.uci()}")
            else:
                self.get_logger().info(f"Engine suggests: {best_move.uci()}")
        else:
            response.success = False
        
        return response

    def destroy_node(self):
        if self.engine:
            self.engine.quit()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ChessEngineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
