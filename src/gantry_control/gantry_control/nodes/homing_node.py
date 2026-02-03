#!/usr/bin/env python3
"""
Homing Node for CoreXY Gantry with NEMA 11 + A4988 Drivers.

Implements Prusa-style homing:
1. Approach limit switch at fast speed
2. Back off slowly
3. Re-approach at precision speed for accuracy

Uses A4988 STEP/DIR control with proper timing.
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point
import RPi.GPIO as GPIO
import time
import signal


class HomingNode(Node):
    """
    Handles homing sequence for CoreXY gantry.
    
    Implements multi-speed homing with limit switch feedback for accurate
    origin finding using A4988 drivers with STEP/DIR control.
    """
    
    # A4988 Pin Configuration (BCM numbering)
    MOTOR_A_DIR_PIN = 27
    MOTOR_A_STEP_PIN = 22
    MOTOR_B_DIR_PIN = 6
    MOTOR_B_STEP_PIN = 5
    
    # Limit switch pins
    X_LIMIT_PIN = 10
    Y_LIMIT_PIN = 9
    
    # Timing constants (matching calibration.py)
    DIR_SETUP_US = 5          # Microseconds for DIR to stabilize
    STEP_PULSE_US = 20        # Microseconds for STEP pulse width
    
    # Speed delays (ms between steps)
    SPEED_FAST_DELAY_MS = 3.0     # ~90% speed - fast approach
    SPEED_SLOW_DELAY_MS = 20.0    # ~50% speed - calibration
    SPEED_PRECISION_DELAY_MS = 50.0  # ~20% speed - final homing
    
    # Homing parameters
    BACKOFF_STEPS = 200       # Steps to back off after hitting limit
    MAX_HOMING_STEPS = 50000  # Maximum steps before giving up
    
    def __init__(self):
        super().__init__('homing_node')
        
        # Declare parameters
        self.declare_parameter('motorA_dir_pin', self.MOTOR_A_DIR_PIN)
        self.declare_parameter('motorA_step_pin', self.MOTOR_A_STEP_PIN)
        self.declare_parameter('motorB_dir_pin', self.MOTOR_B_DIR_PIN)
        self.declare_parameter('motorB_step_pin', self.MOTOR_B_STEP_PIN)
        self.declare_parameter('x_limit_pin', self.X_LIMIT_PIN)
        self.declare_parameter('y_limit_pin', self.Y_LIMIT_PIN)
        self.declare_parameter('backoff_steps', self.BACKOFF_STEPS)
        
        # Get parameters
        self.motorA_dir = self.get_parameter('motorA_dir_pin').get_parameter_value().integer_value
        self.motorA_step = self.get_parameter('motorA_step_pin').get_parameter_value().integer_value
        self.motorB_dir = self.get_parameter('motorB_dir_pin').get_parameter_value().integer_value
        self.motorB_step = self.get_parameter('motorB_step_pin').get_parameter_value().integer_value
        self.x_limit_pin = self.get_parameter('x_limit_pin').get_parameter_value().integer_value
        self.y_limit_pin = self.get_parameter('y_limit_pin').get_parameter_value().integer_value
        self.backoff_steps = self.get_parameter('backoff_steps').get_parameter_value().integer_value
        
        # State
        self.is_homed = False
        self.emergency_stop = False
        
        # Setup GPIO
        self._setup_gpio()
        
        # Create service
        self.home_service = self.create_service(Trigger, '/gantry/home', self.home_callback)
        
        # Publisher for status
        self.status_pub = self.create_publisher(String, '/gantry/status', 10)
        
        # Subscribe to emergency stop
        self.estop_sub = self.create_subscription(Bool, '/emergency_stop', self.estop_callback, 10)
        
        # Signal handling
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        self.get_logger().info('Homing Node initialized')
        self.get_logger().info(f'Motor A: DIR={self.motorA_dir}, STEP={self.motorA_step}')
        self.get_logger().info(f'Motor B: DIR={self.motorB_dir}, STEP={self.motorB_step}')
        self.get_logger().info(f'Limits: X={self.x_limit_pin}, Y={self.y_limit_pin}')
        self.get_logger().info('Service available: /gantry/home')
    
    def _setup_gpio(self):
        """Initialize GPIO pins for A4988 drivers."""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Motor pins as outputs
        GPIO.setup(self.motorA_dir, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.motorA_step, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.motorB_dir, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.motorB_step, GPIO.OUT, initial=GPIO.LOW)
        
        # Limit switches with pull-ups
        GPIO.setup(self.x_limit_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.y_limit_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    def _signal_handler(self, sig, frame):
        """Handle shutdown signals."""
        self.get_logger().info('Shutdown signal received')
        self._cleanup()
    
    def _cleanup(self):
        """Clean up GPIO on shutdown."""
        GPIO.output(self.motorA_step, GPIO.LOW)
        GPIO.output(self.motorB_step, GPIO.LOW)
        GPIO.cleanup()
    
    def _read_x_limit(self) -> bool:
        """Read X limit switch (active low)."""
        return GPIO.input(self.x_limit_pin) == GPIO.LOW
    
    def _read_y_limit(self) -> bool:
        """Read Y limit switch (active low)."""
        return GPIO.input(self.y_limit_pin) == GPIO.LOW
    
    def _step_pulse(self, step_pin: int):
        """Generate a single step pulse."""
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(self.STEP_PULSE_US / 1_000_000)
        GPIO.output(step_pin, GPIO.LOW)
    
    def _move_x(self, steps: int, delay_ms: float) -> bool:
        """
        Move in X direction using CoreXY kinematics.
        X movement: Both motors move SAME direction.
        
        Returns True if completed, False if limit triggered.
        """
        dir_value = GPIO.HIGH if steps > 0 else GPIO.LOW
        
        # Set direction for both motors (same direction for X)
        GPIO.output(self.motorA_dir, dir_value)
        GPIO.output(self.motorB_dir, dir_value)
        time.sleep(self.DIR_SETUP_US / 1_000_000)
        
        delay_sec = delay_ms / 1000.0
        
        for _ in range(abs(steps)):
            if self.emergency_stop:
                return False
            if self._read_x_limit():
                return False
            
            # Step both motors
            GPIO.output(self.motorA_step, GPIO.HIGH)
            GPIO.output(self.motorB_step, GPIO.HIGH)
            time.sleep(self.STEP_PULSE_US / 1_000_000)
            GPIO.output(self.motorA_step, GPIO.LOW)
            GPIO.output(self.motorB_step, GPIO.LOW)
            
            time.sleep(delay_sec)
        
        return True
    
    def _move_y(self, steps: int, delay_ms: float) -> bool:
        """
        Move in Y direction using CoreXY kinematics.
        Y movement: Motors move OPPOSITE directions.
        
        Returns True if completed, False if limit triggered.
        """
        # For +Y: Motor A positive, Motor B negative
        dir_a = GPIO.HIGH if steps > 0 else GPIO.LOW
        dir_b = GPIO.LOW if steps > 0 else GPIO.HIGH
        
        GPIO.output(self.motorA_dir, dir_a)
        GPIO.output(self.motorB_dir, dir_b)
        time.sleep(self.DIR_SETUP_US / 1_000_000)
        
        delay_sec = delay_ms / 1000.0
        
        for _ in range(abs(steps)):
            if self.emergency_stop:
                return False
            if self._read_y_limit():
                return False
            
            # Step both motors
            GPIO.output(self.motorA_step, GPIO.HIGH)
            GPIO.output(self.motorB_step, GPIO.HIGH)
            time.sleep(self.STEP_PULSE_US / 1_000_000)
            GPIO.output(self.motorA_step, GPIO.LOW)
            GPIO.output(self.motorB_step, GPIO.LOW)
            
            time.sleep(delay_sec)
        
        return True
    
    def _home_x(self) -> bool:
        """
        Home X axis using Prusa-style homing.
        
        1. Fast approach until limit triggered
        2. Back off slowly
        3. Precision approach for final position
        """
        self.get_logger().info('Homing X axis...')
        
        # Phase 1: Fast approach (negative direction toward limit)
        self.get_logger().info('  Phase 1: Fast approach')
        already_at_limit = self._read_x_limit()
        if not already_at_limit:
            # Move negative (toward limit) until triggered
            self._move_x(-self.MAX_HOMING_STEPS, self.SPEED_FAST_DELAY_MS)
        
        if not self._read_x_limit():
            self.get_logger().error('  X limit switch not triggered after max steps')
            return False
        
        self.get_logger().info('  X limit triggered')
        
        # Phase 2: Back off slowly (positive direction)
        self.get_logger().info('  Phase 2: Backing off')
        self._move_x(self.backoff_steps, self.SPEED_SLOW_DELAY_MS)
        
        if self._read_x_limit():
            # Still on limit, back off more
            self._move_x(self.backoff_steps, self.SPEED_SLOW_DELAY_MS)
        
        # Phase 3: Precision approach
        self.get_logger().info('  Phase 3: Precision approach')
        self._move_x(-self.MAX_HOMING_STEPS, self.SPEED_PRECISION_DELAY_MS)
        
        if self._read_x_limit():
            self.get_logger().info('  X homing complete')
            return True
        else:
            self.get_logger().error('  X precision homing failed')
            return False
    
    def _home_y(self) -> bool:
        """
        Home Y axis using Prusa-style homing.
        
        1. Fast approach until limit triggered
        2. Back off slowly
        3. Precision approach for final position
        """
        self.get_logger().info('Homing Y axis...')
        
        # Phase 1: Fast approach
        self.get_logger().info('  Phase 1: Fast approach')
        already_at_limit = self._read_y_limit()
        if not already_at_limit:
            self._move_y(-self.MAX_HOMING_STEPS, self.SPEED_FAST_DELAY_MS)
        
        if not self._read_y_limit():
            self.get_logger().error('  Y limit switch not triggered after max steps')
            return False
        
        self.get_logger().info('  Y limit triggered')
        
        # Phase 2: Back off slowly
        self.get_logger().info('  Phase 2: Backing off')
        self._move_y(self.backoff_steps, self.SPEED_SLOW_DELAY_MS)
        
        if self._read_y_limit():
            self._move_y(self.backoff_steps, self.SPEED_SLOW_DELAY_MS)
        
        # Phase 3: Precision approach
        self.get_logger().info('  Phase 3: Precision approach')
        self._move_y(-self.MAX_HOMING_STEPS, self.SPEED_PRECISION_DELAY_MS)
        
        if self._read_y_limit():
            self.get_logger().info('  Y homing complete')
            return True
        else:
            self.get_logger().error('  Y precision homing failed')
            return False
    
    def home_callback(self, request, response):
        """Handle homing service request."""
        self.get_logger().info('Homing request received')
        
        # Publish status
        status_msg = String()
        status_msg.data = 'HOMING_STARTED'
        self.status_pub.publish(status_msg)
        
        self.emergency_stop = False
        
        # Home X first, then Y
        x_success = self._home_x()
        if not x_success:
            response.success = False
            response.message = 'X homing failed'
            status_msg.data = 'HOMING_FAILED'
            self.status_pub.publish(status_msg)
            return response
        
        y_success = self._home_y()
        if not y_success:
            response.success = False
            response.message = 'Y homing failed'
            status_msg.data = 'HOMING_FAILED'
            self.status_pub.publish(status_msg)
            return response
        
        self.is_homed = True
        response.success = True
        response.message = 'Homing complete'
        
        status_msg.data = 'HOMED'
        self.status_pub.publish(status_msg)
        
        self.get_logger().info('Homing sequence complete!')
        return response
    
    def estop_callback(self, msg):
        """Handle emergency stop."""
        if msg.data:
            self.get_logger().warn('Emergency stop triggered!')
            self.emergency_stop = True


def main(args=None):
    rclpy.init(args=args)
    node = HomingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        node._cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
