#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import time

# Try to import RPi.GPIO, fallback to Mock if not available
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

class LimitSwitchNode(Node):
    def __init__(self):
        super().__init__('limit_switch_node')
        
        # Parameters
        self.declare_parameter('limit_switch_pins.x_min', 6)
        self.declare_parameter('limit_switch_pins.y_min', 13)
        self.declare_parameter('limit_switch_pins.clock_hit', 19)
        self.declare_parameter('debounce_ms', 20)
        
        self.pin_x = self.get_parameter('limit_switch_pins.x_min').value
        self.pin_y = self.get_parameter('limit_switch_pins.y_min').value
        self.pin_clock = self.get_parameter('limit_switch_pins.clock_hit').value
        self.debounce = self.get_parameter('debounce_ms').value / 1000.0
        
        # GPIO Setup
        if GPIO_AVAILABLE:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.pin_x, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.pin_y, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.pin_clock, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Add interrupts
            GPIO.add_event_detect(self.pin_x, GPIO.FALLING, callback=self.switch_callback, bouncetime=int(self.debounce*1000))
            GPIO.add_event_detect(self.pin_y, GPIO.FALLING, callback=self.switch_callback, bouncetime=int(self.debounce*1000))
            GPIO.add_event_detect(self.pin_clock, GPIO.FALLING, callback=self.switch_callback, bouncetime=int(self.debounce*1000))
            
            self.get_logger().info(f"Limit Switches Initialized: X={self.pin_x}, Y={self.pin_y}, Clock={self.pin_clock}")
        else:
            self.get_logger().warn("RPi.GPIO not found. Running in MOCK mode.")

        # Publishers
        self.x_pub = self.create_publisher(Bool, '/limit_switch/x_min', 10)
        self.y_pub = self.create_publisher(Bool, '/limit_switch/y_min', 10)
        self.clock_pub = self.create_publisher(Bool, '/limit_switch/clock_hit', 10)
        self.stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)

    def switch_callback(self, channel):
        if channel == self.pin_x:
            self.get_logger().info("X Limit Hit")
            self.x_pub.publish(Bool(data=True))
            # If limit hit unexpectedly, could trigger stop
            # self.stop_pub.publish(Bool(data=True))
            
        elif channel == self.pin_y:
            self.get_logger().info("Y Limit Hit")
            self.y_pub.publish(Bool(data=True))
            
        elif channel == self.pin_clock:
            self.get_logger().info("Clock Button Pressed")
            self.clock_pub.publish(Bool(data=True))

    def destroy_node(self):
        if GPIO_AVAILABLE:
            GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LimitSwitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
