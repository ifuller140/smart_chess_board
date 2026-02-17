#!/usr/bin/env python3
"""
Homing Node for CoreXY Gantry with NEMA 11 + A4988 Drivers.

Implements Prusa-style homing using pigpio DMA for step pulses:
1. Disengage magnet (raise servo) before homing
2. Approach limit switch at fast speed
3. Back off slowly
4. Re-approach at precision speed for accuracy

Both motors always step in the same DMA wave pulse — no inter-motor
lag during homing moves.

Physical Layout:
- Motor A at bottom-left corner
- Motor B at top-right corner
- Origin (0,0) at bottom-left (where limit switches are)

CoreXY Kinematics for this layout (inverted DIR pins):
- +X (right): A CCW, B CW = OPPOSITE directions
- +Y (up): A CCW, B CCW = SAME direction

Limit switches: active HIGH (pressed = 5V → GPIO reads HIGH, pull-down).

Requirements:
- pigpio library: pip install pigpio
- pigpio daemon running: sudo pigpiod
"""
import time
import signal

import pigpio
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, String


class HomingNode(Node):
    """
    Handles homing sequence for CoreXY gantry.

    Uses pigpio DMA wave chains for synchronized motor stepping and
    small-batch moves with limit switch checks for safe homing.
    """

    # A4988 Pin Configuration (BCM numbering)
    MOTOR_A_DIR_PIN = 27
    MOTOR_A_STEP_PIN = 22
    MOTOR_B_DIR_PIN = 6
    MOTOR_B_STEP_PIN = 5
    MOTOR_ENABLE_PIN = 17

    # Limit switch pins (active HIGH with pull-down)
    X_LIMIT_PIN = 10
    Y_LIMIT_PIN = 9

    # Servo pin for magnet (Z-axis) — still uses pigpio hardware PWM
    SERVO_PIN = 12
    SERVO_RELEASE_DUTY = 7.5   # Raised position (disengaged)
    SERVO_ENGAGE_DUTY = 2.5    # Lowered position (engaged)
    SERVO_FREQ = 50            # 50Hz for servo

    # Homing speed percentages (not configurable, homing-specific)
    SPEED_FAST = 70            # Fast approach
    SPEED_SLOW = 30            # Back off
    SPEED_PRECISION = 15       # Final approach

    # Homing parameters
    BACKOFF_STEPS = 200        # Steps to back off after hitting limit
    MAX_HOMING_STEPS = 50000   # Maximum steps before giving up
    BATCH_SIZE = 25            # Steps per DMA batch during homing

    def __init__(self):
        super().__init__('homing_node')

        # Declare parameters
        self.declare_parameter('motorA_dir_pin', self.MOTOR_A_DIR_PIN)
        self.declare_parameter('motorA_step_pin', self.MOTOR_A_STEP_PIN)
        self.declare_parameter('motorB_dir_pin', self.MOTOR_B_DIR_PIN)
        self.declare_parameter('motorB_step_pin', self.MOTOR_B_STEP_PIN)
        self.declare_parameter('motor_enable_pin', self.MOTOR_ENABLE_PIN)
        self.declare_parameter('x_limit_pin', self.X_LIMIT_PIN)
        self.declare_parameter('y_limit_pin', self.Y_LIMIT_PIN)
        self.declare_parameter('servo_pin', self.SERVO_PIN)
        self.declare_parameter('backoff_steps', self.BACKOFF_STEPS)

        # Timing/speed parameters (shared with stepper_driver via pins.yaml)
        self.declare_parameter('dir_setup_us', 5)
        self.declare_parameter('step_pulse_us', 10)
        self.declare_parameter('max_speed', 800)
        self.declare_parameter('min_speed', 150)
        self.declare_parameter('accel_ramp_steps', 60)

        # Get parameters
        self.motorA_dir = self.get_parameter('motorA_dir_pin').get_parameter_value().integer_value
        self.motorA_step = self.get_parameter('motorA_step_pin').get_parameter_value().integer_value
        self.motorB_dir = self.get_parameter('motorB_dir_pin').get_parameter_value().integer_value
        self.motorB_step = self.get_parameter('motorB_step_pin').get_parameter_value().integer_value
        self.motor_enable = self.get_parameter('motor_enable_pin').get_parameter_value().integer_value
        self.x_limit_pin = self.get_parameter('x_limit_pin').get_parameter_value().integer_value
        self.y_limit_pin = self.get_parameter('y_limit_pin').get_parameter_value().integer_value
        self.servo_pin = self.get_parameter('servo_pin').get_parameter_value().integer_value
        self.backoff_steps = self.get_parameter('backoff_steps').get_parameter_value().integer_value

        # Timing/speed
        self.dir_setup_us = self.get_parameter('dir_setup_us').get_parameter_value().integer_value
        self.step_pulse_us = self.get_parameter('step_pulse_us').get_parameter_value().integer_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().integer_value
        self.min_speed = self.get_parameter('min_speed').get_parameter_value().integer_value
        self.accel_ramp_steps = self.get_parameter('accel_ramp_steps').get_parameter_value().integer_value

        # State
        self.is_homed = False
        self.emergency_stop = False

        # Connect to pigpio daemon
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().fatal(
                "Cannot connect to pigpiod daemon. "
                "Start it with: sudo pigpiod"
            )
            raise RuntimeError("pigpiod not running")

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

        self.get_logger().info('Homing Node initialized (pigpio DMA)')
        self.get_logger().info(f'Motor A (bottom-left): DIR={self.motorA_dir}, STEP={self.motorA_step}')
        self.get_logger().info(f'Motor B (top-right): DIR={self.motorB_dir}, STEP={self.motorB_step}')
        self.get_logger().info(f'Motor enable (active LOW): EN={self.motor_enable}')
        self.get_logger().info(f'Limits (active HIGH): X={self.x_limit_pin}, Y={self.y_limit_pin}')
        self.get_logger().info(f'Servo (magnet): PIN={self.servo_pin}')
        self.get_logger().info('Service available: /gantry/home')

    def _setup_gpio(self):
        """Initialize GPIO pins via pigpio."""
        # Motor pins as outputs
        for pin in [self.motorA_dir, self.motorA_step,
                    self.motorB_dir, self.motorB_step, self.motor_enable]:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, 0)

        # Disable motors initially (A4988 ENABLE is active LOW)
        self.pi.write(self.motor_enable, 1)

        # Limit switches: active HIGH with pull-down
        for pin in [self.x_limit_pin, self.y_limit_pin]:
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pigpio.PUD_DOWN)

        # Servo: use pigpio hardware PWM (no jitter)
        self.pi.set_mode(self.servo_pin, pigpio.OUTPUT)

    def _signal_handler(self, sig, frame):
        """Handle shutdown signals."""
        self.get_logger().info('Shutdown signal received')
        self._cleanup()

    def _cleanup(self):
        """Clean up GPIO on shutdown."""
        try:
            self.pi.wave_tx_stop()
        except Exception:
            pass
        self.pi.write(self.motor_enable, 1)
        self.pi.write(self.motorA_step, 0)
        self.pi.write(self.motorB_step, 0)
        # Stop servo PWM
        self.pi.set_servo_pulsewidth(self.servo_pin, 0)
        try:
            self.pi.wave_clear()
        except Exception:
            pass
        if self.pi.connected:
            self.pi.stop()

    def _disengage_magnet(self):
        """Raise the magnet (disengage) before homing using pigpio hardware servo."""
        self.get_logger().info('Disengaging magnet (raising servo)...')
        # SG90 servo: 500-2500µs pulse width at 50Hz
        # Release position ~ 1500µs (neutral/raised)
        self.pi.set_servo_pulsewidth(self.servo_pin, 1500)
        time.sleep(0.5)
        # Stop sending pulses to avoid jitter
        self.pi.set_servo_pulsewidth(self.servo_pin, 0)

    def _read_x_limit(self) -> bool:
        """Read X limit switch (active HIGH — pressed = 5V)."""
        return self.pi.read(self.x_limit_pin) == 1

    def _read_y_limit(self) -> bool:
        """Read Y limit switch (active HIGH — pressed = 5V)."""
        return self.pi.read(self.y_limit_pin) == 1

    def _enable_motors(self):
        self.pi.write(self.motor_enable, 0)
        time.sleep(0.001)

    def _disable_motors(self):
        self.pi.write(self.motor_enable, 1)

    # ------------------------------------------------------------------
    # DMA wave chain helpers (same approach as stepper_driver_node)
    # ------------------------------------------------------------------

    def _calculate_speed_profile(self, total_steps, speed_percent):
        """Calculate trapezoidal speed profile."""
        if total_steps <= 0:
            return []

        target_speed = int(self.min_speed + (speed_percent / 100.0) * (self.max_speed - self.min_speed))

        accel = min(self.accel_ramp_steps, total_steps // 3)
        if total_steps <= accel * 2:
            delay_us = int(1_000_000 / self.min_speed)
            return [(total_steps, delay_us)]

        decel = accel
        cruise_steps = total_steps - accel - decel

        profile = []
        for i in range(accel):
            t = (i + 1) / accel
            speed = self.min_speed + t * (target_speed - self.min_speed)
            profile.append((1, int(1_000_000 / speed)))
        if cruise_steps > 0:
            profile.append((cruise_steps, int(1_000_000 / target_speed)))
        for i in range(decel):
            t = (i + 1) / decel
            speed = target_speed - t * (target_speed - self.min_speed)
            profile.append((1, int(1_000_000 / speed)))

        return profile

    def _step_both(self, steps_a: int, steps_b: int, speed_percent: int):
        """
        Step both motors simultaneously via DMA wave chain.

        Both motors' step edges are in the same pulse.
        """
        if steps_a == 0 and steps_b == 0:
            return

        # INVERTED DIR pins — positive steps → DIR LOW for this hardware
        self.pi.write(self.motorA_dir, 0 if steps_a >= 0 else 1)
        self.pi.write(self.motorB_dir, 0 if steps_b >= 0 else 1)
        time.sleep(self.dir_setup_us / 1_000_000)

        abs_a = abs(steps_a)
        abs_b = abs(steps_b)
        max_steps = max(abs_a, abs_b)
        if max_steps == 0:
            return

        profile = self._calculate_speed_profile(max_steps, speed_percent)

        # Flatten
        per_step_delays = []
        for cnt, delay in profile:
            per_step_delays.extend([delay] * cnt)

        # Bresenham
        step_plan = []
        err_a = 0
        err_b = 0
        for idx in range(max_steps):
            do_a = False
            do_b = False
            err_a += abs_a
            if err_a >= max_steps:
                err_a -= max_steps
                do_a = True
            err_b += abs_b
            if err_b >= max_steps:
                err_b -= max_steps
                do_b = True
            if do_a or do_b:
                step_plan.append((do_a, do_b, per_step_delays[idx]))

        if not step_plan:
            return

        # Group
        groups = []
        cur = (step_plan[0][0], step_plan[0][1], step_plan[0][2])
        cnt = 1
        for entry in step_plan[1:]:
            p = (entry[0], entry[1], entry[2])
            if p == cur:
                cnt += 1
            else:
                groups.append((cur, cnt))
                cur = p
                cnt = 1
        groups.append((cur, cnt))

        # Build waves
        wave_ids = []
        p2w = {}

        try:
            self.pi.wave_clear()

            for pattern, _ in groups:
                if pattern in p2w:
                    continue
                do_a, do_b, delay_us = pattern
                wait_us = max(1, delay_us - self.step_pulse_us)

                set_m = 0
                clr_m = 0
                if do_a:
                    set_m |= (1 << self.motorA_step)
                    clr_m |= (1 << self.motorA_step)
                if do_b:
                    set_m |= (1 << self.motorB_step)
                    clr_m |= (1 << self.motorB_step)

                self.pi.wave_add_generic([
                    pigpio.pulse(set_m, 0, self.step_pulse_us),
                    pigpio.pulse(0, clr_m, wait_us),
                ])
                wid = self.pi.wave_create()
                if wid < 0:
                    raise RuntimeError(f"wave_create failed ({wid})")
                p2w[pattern] = wid
                wave_ids.append(wid)

            chain = []
            for pattern, c in groups:
                wid = p2w[pattern]
                if c == 1:
                    chain.append(wid)
                else:
                    chain.extend([255, 0, wid, 255, 1, c & 0xFF, (c >> 8) & 0xFF])

            self.pi.wave_chain(chain)
            while self.pi.wave_tx_busy():
                if self.emergency_stop:
                    self.pi.wave_tx_stop()
                    break
                time.sleep(0.001)

        finally:
            for wid in wave_ids:
                try:
                    self.pi.wave_delete(wid)
                except Exception:
                    pass

    # ------------------------------------------------------------------
    # CoreXY motion (small-batch with limit checks for homing)
    # ------------------------------------------------------------------

    def _move_x(self, steps: int, speed_percent: int) -> bool:
        """
        Move in X direction using CoreXY kinematics in small batches.

        +X (right): A+, B-

        Returns True if completed, False if limit triggered or e-stop.
        """
        sign = 1 if steps > 0 else -1
        remaining = abs(steps)

        self._enable_motors()
        try:
            while remaining > 0 and not self.emergency_stop:
                if self._read_x_limit() and sign < 0:
                    return False
                batch = min(self.BATCH_SIZE, remaining)
                dx = sign * batch
                a = dx
                b = -dx
                self._step_both(a, b, speed_percent)
                remaining -= batch
        finally:
            self._disable_motors()

        return not self.emergency_stop

    def _move_y(self, steps: int, speed_percent: int) -> bool:
        """
        Move in Y direction using CoreXY kinematics in small batches.

        +Y (up): A+, B+

        Returns True if completed, False if limit triggered or e-stop.
        """
        sign = 1 if steps > 0 else -1
        remaining = abs(steps)

        self._enable_motors()
        try:
            while remaining > 0 and not self.emergency_stop:
                if self._read_y_limit() and sign < 0:
                    return False
                batch = min(self.BATCH_SIZE, remaining)
                dy = sign * batch
                a = dy
                b = dy
                self._step_both(a, b, speed_percent)
                remaining -= batch
        finally:
            self._disable_motors()

        return not self.emergency_stop

    # ------------------------------------------------------------------
    # Homing sequences
    # ------------------------------------------------------------------

    def _home_x(self) -> bool:
        """
        Home X axis using Prusa-style homing.

        Move in -X direction until limit triggered, back off, precision approach.
        """
        self.get_logger().info('Homing X axis (moving left toward origin)...')

        # Phase 1: Fast approach
        self.get_logger().info('  Phase 1: Fast approach')
        if not self._read_x_limit():
            self._move_x(-self.MAX_HOMING_STEPS, self.SPEED_FAST)

        if not self._read_x_limit():
            self.get_logger().error('  X limit switch not triggered after max steps')
            return False

        self.get_logger().info('  X limit triggered')

        # Phase 2: Back off
        self.get_logger().info('  Phase 2: Backing off')
        self._move_x(self.backoff_steps, self.SPEED_SLOW)

        if self._read_x_limit():
            self._move_x(self.backoff_steps, self.SPEED_SLOW)

        # Phase 3: Precision approach
        self.get_logger().info('  Phase 3: Precision approach')
        self._move_x(-self.MAX_HOMING_STEPS, self.SPEED_PRECISION)

        if self._read_x_limit():
            self.get_logger().info('  X homing complete')
            return True
        else:
            self.get_logger().error('  X precision homing failed')
            return False

    def _home_y(self) -> bool:
        """
        Home Y axis using Prusa-style homing.

        Move in -Y direction until limit triggered, back off, precision approach.
        """
        self.get_logger().info('Homing Y axis (moving down toward origin)...')

        # Phase 1: Fast approach
        self.get_logger().info('  Phase 1: Fast approach')
        if not self._read_y_limit():
            self._move_y(-self.MAX_HOMING_STEPS, self.SPEED_FAST)

        if not self._read_y_limit():
            self.get_logger().error('  Y limit switch not triggered after max steps')
            return False

        self.get_logger().info('  Y limit triggered')

        # Phase 2: Back off
        self.get_logger().info('  Phase 2: Backing off')
        self._move_y(self.backoff_steps, self.SPEED_SLOW)

        if self._read_y_limit():
            self._move_y(self.backoff_steps, self.SPEED_SLOW)

        # Phase 3: Precision approach
        self.get_logger().info('  Phase 3: Precision approach')
        self._move_y(-self.MAX_HOMING_STEPS, self.SPEED_PRECISION)

        if self._read_y_limit():
            self.get_logger().info('  Y homing complete')
            return True
        else:
            self.get_logger().error('  Y precision homing failed')
            return False

    def home_callback(self, request, response):
        """Handle homing service request."""
        self.get_logger().info('Homing request received')

        status_msg = String()
        status_msg.data = 'HOMING_STARTED'
        self.status_pub.publish(status_msg)

        self.emergency_stop = False

        # SAFETY: Disengage magnet before homing
        self._disengage_magnet()

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
