#!/usr/bin/env python3
"""
Motion Planner Node.

Converts chess move commands (UCI notation e.g. "e2e4") into a sequence
of gantry moves: go to source square → engage magnet → go to target
square → release magnet.

The path from source to target is planned as:
  source → target (direct, using gantry_kinematics_node's trapezoidal profile)

For future improvement: waypoint routing around pieces.

Interfaces:
  Listens:  /motion/command  (std_msgs/String, e.g. "e2e4")
  Uses:     /gantry/move     (chess_interfaces/MoveGantry action)
  Uses:     /servo/engage    (std_srvs/Trigger)
  Uses:     /servo/release   (std_srvs/Trigger)
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from chess_interfaces.action import MoveGantry


# Column letter → 0-indexed column
COL_MAP = {'a': 0, 'b': 1, 'c': 2, 'd': 3, 'e': 4, 'f': 5, 'g': 6, 'h': 7}


class MotionPlannerNode(Node):

    def __init__(self):
        super().__init__('motion_planner_node')

        # Board geometry parameters — defaults match board_map.yaml
        # Coordinate system: origin at bottom-right (homing position)
        # +X = LEFT (toward a-file), +Y = UP (toward rank 8 / black's side)
        self.declare_parameter('square_size_mm', 25.0)
        self.declare_parameter('board_origin_x_mm', 200.0)   # X of center of a1
        self.declare_parameter('board_origin_y_mm', 20.0)    # Y of center of a1
        self.declare_parameter('move_speed_mm_s', 50.0)

        self.sq_size = self.get_parameter('square_size_mm').value
        self.origin_x = self.get_parameter('board_origin_x_mm').value
        self.origin_y = self.get_parameter('board_origin_y_mm').value
        self.move_speed = self.get_parameter('move_speed_mm_s').value

        # Action client for gantry movement
        self._action_client = ActionClient(self, MoveGantry, '/gantry/move')

        # Servo service clients
        self._servo_engage = self.create_client(Trigger, '/servo/engage')
        self._servo_release = self.create_client(Trigger, '/servo/release')

        # Command subscription
        self.create_subscription(String, '/motion/command', self._command_cb, 10)

        self.get_logger().info(
            f'Motion Planner ready — square={self.sq_size}mm, '
            f'origin=({self.origin_x}, {self.origin_y}), speed={self.move_speed}mm/s'
        )

    # ------------------------------------------------------------------
    # Coordinate math
    # ------------------------------------------------------------------

    def _square_to_mm(self, square: str):
        """
        Convert a chess square (e.g. "e2") to gantry coordinates in mm.

        Coordinate system:
          Origin (0,0) = bottom-right corner (homing position)
          +X = LEFT toward a-file   → a-file has the highest X
          +Y = UP toward rank 8     → rank 8 has the highest Y

        Formula:
          col_index: a=0, b=1, ..., h=7
          x = origin_x (a1 center) - col_index * square_size_mm
          y = origin_y (a1 center) + (rank - 1) * square_size_mm

        Returns (x_mm, y_mm) at the CENTER of the square.
        """
        col = COL_MAP.get(square[0].lower())
        row = int(square[1]) - 1   # 0-indexed, rank 1 = row 0
        if col is None or not (0 <= row <= 7):
            raise ValueError(f'Invalid square: {square!r}')
        # +X is LEFT, so a-file (col=0) is at origin_x, h-file (col=7) is further right (lower X)
        x = self.origin_x - col * self.sq_size
        # +Y is UP, so rank 1 (row=0) is at origin_y, rank 8 (row=7) is further back (higher Y)
        y = self.origin_y + row * self.sq_size
        return x, y

    # ------------------------------------------------------------------
    # Command handler
    # ------------------------------------------------------------------

    def _command_cb(self, msg: String):
        """Handle a UCI move string e.g. "e2e4"."""
        uci = msg.data.strip()
        if len(uci) < 4:
            self.get_logger().error(f'Invalid command: {uci!r} (need at least 4 chars)')
            return

        source = uci[:2]
        target = uci[2:4]

        try:
            self.get_logger().info(f'Executing move: {source} → {target}')
            self._execute_move(source, target)
            self.get_logger().info(f'Move {uci} complete')
        except Exception as e:
            self.get_logger().error(f'Move failed: {e}')

    # ------------------------------------------------------------------
    # Move sequence
    # ------------------------------------------------------------------

    def _execute_move(self, source: str, target: str):
        """
        Full piece-move sequence:
          1. Move to source square
          2. Lower and engage magnet
          3. Move to target square
          4. Release magnet (raise servo)
        """
        sx, sy = self._square_to_mm(source)
        tx, ty = self._square_to_mm(target)

        # 1. Move to source
        self.get_logger().info(f'  → Moving to {source} ({sx:.0f}, {sy:.0f}) mm')
        self._send_move(sx, sy)

        # 2. Engage magnet
        self.get_logger().info('  → Engaging magnet')
        self._call_servo(self._servo_engage)

        # 3. Move to target
        self.get_logger().info(f'  → Moving to {target} ({tx:.0f}, {ty:.0f}) mm')
        self._send_move(tx, ty)

        # 4. Release magnet
        self.get_logger().info('  → Releasing magnet')
        self._call_servo(self._servo_release)

    def _send_move(self, x_mm: float, y_mm: float, speed: float = None):
        """
        Send an action goal to /gantry/move and block until complete.
        """
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('/gantry/move action server not available')

        goal = MoveGantry.Goal()
        goal.x_mm = x_mm
        goal.y_mm = y_mm
        goal.speed_mm_s = speed if speed is not None else self.move_speed

        send_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb,
        )
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            raise RuntimeError(f'Goal to ({x_mm:.0f}, {y_mm:.0f}) rejected')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if not result.success:
            raise RuntimeError(f'Move failed: {result.message}')

    def _feedback_cb(self, feedback_msg):
        f = feedback_msg.feedback
        self.get_logger().debug(
            f'  Progress: {f.percent_complete:.0f}%  '
            f'pos=({f.current_x_mm:.1f}, {f.current_y_mm:.1f})'
        )

    def _call_servo(self, client):
        """Call a servo trigger service (engage or release)."""
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Servo service not available — skipping')
            return
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp and not resp.success:
            self.get_logger().warn(f'Servo call returned: {resp.message}')


def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
