#!/usr/bin/env python3
"""
Clock Display Node — drives two TM1637 4-digit 7-segment displays.

Display 1 (left):  White's remaining time (MM:SS)
Display 2 (right): Black's remaining time (MM:SS)

Uses the TM1637 bit-banging driver from chess_hw_interface.testing.test_display.
Per pins.yaml:
  Display 1: CLK=GPIO25, DIO=GPIO8
  Display 2: CLK=GPIO7,  DIO=GPIO1
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32

from chess_hw_interface.testing.test_display import create_display


class ClockDisplayNode(Node):
    def __init__(self):
        super().__init__('clock_display_node')

        # Parameters (from pins.yaml clock_display section)
        self.declare_parameter('display1_pins', [25, 8])
        self.declare_parameter('display2_pins', [7, 1])
        self.declare_parameter('brightness', 7)

        d1_pins = self.get_parameter('display1_pins').value
        d2_pins = self.get_parameter('display2_pins').value
        brightness = self.get_parameter('brightness').value

        # State
        self.white_time = 0.0
        self.black_time = 0.0
        self.emergency_stop = False
        self.flash_text = None
        self.flash_until = 0.0

        # Initialize displays
        try:
            # Use factory to support pigpio (non-root) backend
            self.display1 = create_display("tm1637",
                clk_pin=d1_pins[0], dio_pin=d1_pins[1], brightness=brightness)
            self.display2 = create_display("tm1637",
                clk_pin=d2_pins[0], dio_pin=d2_pins[1], brightness=brightness)
            self.get_logger().info(
                f'Clock displays initialized: D1(CLK={d1_pins[0]}, DIO={d1_pins[1]}), '
                f'D2(CLK={d2_pins[0]}, DIO={d2_pins[1]}), brightness={brightness}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize displays: {e}')
            self.display1 = None
            self.display2 = None

        # Subscribers
        self.create_subscription(
            Float32, '/clock/white_time', self.white_time_cb, 10)
        self.create_subscription(
            Float32, '/clock/black_time', self.black_time_cb, 10)
        self.create_subscription(
            String, '/game_manager/state', self.game_state_cb, 10)
        self.create_subscription(
            Bool, '/emergency_stop', self.stop_cb, 10)

        # Colon blink state (toggles at 1Hz for active player)
        self.colon_on = True

        # Refresh display at 2Hz (enough for clock, avoids bit-bang bus contention)
        self.create_timer(0.5, self.refresh_display)

        self.get_logger().info('Clock Display Node ready')

    def white_time_cb(self, msg):
        self.white_time = max(0.0, msg.data)

    def black_time_cb(self, msg):
        self.black_time = max(0.0, msg.data)

    def game_state_cb(self, msg):
        """Flash a status message on both displays briefly."""
        state = msg.data.upper()
        flash_map = {
            'CHECK': 'CHK ',
            'CHECKMATE': 'MATE',
            'STALEMATE': 'DRAW',
            'GAME_OVER': 'DONE',
            'IDLE': 'IDLE',
        }
        text = flash_map.get(state)
        if text:
            self.flash_text = text
            # Flash for 2 seconds (4 refresh cycles at 2Hz)
            self.flash_until = self.get_clock().now().nanoseconds / 1e9 + 2.0

    def stop_cb(self, msg):
        if msg.data:
            self.emergency_stop = True
            self.get_logger().warn('EMERGENCY STOP: showing STOP on displays')

    def _show_time(self, display, seconds, colon):
        """Show MM:SS on a display."""
        if display is None:
            return
        total_s = int(seconds)
        minutes = min(99, total_s // 60)
        secs = total_s % 60
        display.show_time(minutes, secs, colon=colon)

    def refresh_display(self):
        """Timer callback — update both displays."""
        if self.display1 is None or self.display2 is None:
            return

        # Emergency stop overrides everything
        if self.emergency_stop:
            self.display1.show_text('STOP')
            self.display2.show_text('STOP')
            return

        # Flash text overrides time display
        now = self.get_clock().now().nanoseconds / 1e9
        if self.flash_text and now < self.flash_until:
            self.display1.show_text(self.flash_text)
            self.display2.show_text(self.flash_text)
            return
        else:
            self.flash_text = None

        # Normal: show times with blinking colon
        self.colon_on = not self.colon_on
        self._show_time(self.display1, self.white_time, self.colon_on)
        self._show_time(self.display2, self.black_time, self.colon_on)

    def destroy_node(self):
        if self.display1:
            self.display1.cleanup()
        if self.display2:
            self.display2.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ClockDisplayNode()
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
