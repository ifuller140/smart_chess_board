#!/usr/bin/env python3
"""
Homing Node for CoreXY Gantry with NEMA 11 + A4988 Drivers.

Implements Prusa-style homing using pigpio DMA for step pulses:
  1. Disengage magnet (raise servo) before homing
  2. Fast approach to limit switch
  3. Back off slowly away from switch
  4. Re-approach at precision speed with small batches for accuracy

Physical coordinate system (origin at bottom-left from player's perspective):
  - Player sits at the front (low Y side)
  - Camera / electronics tower is at the back (high Y side)
  - (0, 0) = bottom-left corner of the work area
  - +X = rightward (toward h-file, toward X limit switch)
  - +Y = backward / upward (toward rank 8, away from player)

Limit switches:
  - X limit: at X_MAX (rightmost point, far right from player)
  - Y limit: at Y_MIN = 0 (frontmost point, closest to player)

Homing sequence:
  1. Home Y first (move in -Y toward player until Y limit triggers)
  2. Home X (move in +X toward right side until X limit triggers)
  3. After both limits found: drive X back to X=0 (origin at left)
  4. Reset stepper driver position counter to (0, 0)

Limit switches: active HIGH (pressed = 5V → GPIO reads HIGH, pull-down).

Requirements:
  - pigpio library: pip install pigpio
  - pigpio daemon running: sudo pigpiod
"""
import time
import signal

import pigpio
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, String

from chess_hw_interface.gpio_lock import GantryPinLock

# SG90 pulse-width range, matches chess_hw_interface.nodes.servo_node
# and code/test_z_servo.py's calibration sweep
MIN_PULSE_US = 500
MAX_PULSE_US = 2500


def angle_to_pulsewidth(degrees: float) -> int:
    """Convert a 0-180 degree servo angle to a pigpio pulse width in microseconds."""
    return int(MIN_PULSE_US + (degrees / 180.0) * (MAX_PULSE_US - MIN_PULSE_US))


class HomingNode(Node):
    """
    Handles homing sequence for CoreXY gantry.

    Uses pigpio DMA wave chains for synchronized motor stepping and
    small-batch moves with limit switch checks for safe homing.
    """

    # Default pin assignments (BCM) — overridden by pins.yaml ROS parameters
    _DEFAULT_MOTOR_A_DIR_PIN  = 27
    _DEFAULT_MOTOR_A_STEP_PIN = 22
    _DEFAULT_MOTOR_B_DIR_PIN  = 6
    _DEFAULT_MOTOR_B_STEP_PIN = 5
    _DEFAULT_MOTOR_ENABLE_PIN = 17
    _DEFAULT_X_LIMIT_PIN      = 10
    _DEFAULT_Y_LIMIT_PIN      = 9
    _DEFAULT_SERVO_PIN        = 12

    def __init__(self):
        super().__init__('homing_node')

        # ── Motor / pin parameters ─────────────────────────────────────────
        self.declare_parameter('motorA_dir_pin',    self._DEFAULT_MOTOR_A_DIR_PIN)
        self.declare_parameter('motorA_step_pin',   self._DEFAULT_MOTOR_A_STEP_PIN)
        self.declare_parameter('motorB_dir_pin',    self._DEFAULT_MOTOR_B_DIR_PIN)
        self.declare_parameter('motorB_step_pin',   self._DEFAULT_MOTOR_B_STEP_PIN)
        self.declare_parameter('motor_enable_pin',  self._DEFAULT_MOTOR_ENABLE_PIN)
        self.declare_parameter('x_limit_pin',       self._DEFAULT_X_LIMIT_PIN)
        self.declare_parameter('y_limit_pin',       self._DEFAULT_Y_LIMIT_PIN)
        self.declare_parameter('servo_pin',         self._DEFAULT_SERVO_PIN)
        self.declare_parameter('release_angle_deg', 170.0)  # Clear position — matches servo_node

        # ── Timing / speed parameters ──────────────────────────────────────
        self.declare_parameter('dir_setup_us',      5)
        self.declare_parameter('step_pulse_us',     10)
        self.declare_parameter('max_speed',         800)
        self.declare_parameter('min_speed',         150)
        self.declare_parameter('accel_ramp_steps',  60)

        # ── Homing behaviour parameters ────────────────────────────────────
        self.declare_parameter('speed_fast_pct',    70)    # % for fast approach
        self.declare_parameter('speed_slow_pct',    30)    # % for back-off
        self.declare_parameter('speed_prec_pct',    12)    # % for precision approach
        self.declare_parameter('backoff_steps',     200)   # steps to retreat after first contact
        self.declare_parameter('max_homing_steps',  50000) # fail-safe step limit
        self.declare_parameter('batch_size_fast',   25)    # steps per batch during fast approach
        self.declare_parameter('batch_size_prec',   4)     # steps per batch during precision (≈1mm)

        # ── Coordinate parameters ──────────────────────────────────────────
        # x_max_mm: physical X travel from origin (bottom-left) to X limit switch
        # Must match gantry_kinematics_node's x_max_mm — see board_map.yaml.
        self.declare_parameter('x_max_mm',          250.0)
        self.declare_parameter('steps_per_mm',      5.0)

        # ── Read all parameters ────────────────────────────────────────────
        self.motorA_dir    = self.get_parameter('motorA_dir_pin').value
        self.motorA_step   = self.get_parameter('motorA_step_pin').value
        self.motorB_dir    = self.get_parameter('motorB_dir_pin').value
        self.motorB_step   = self.get_parameter('motorB_step_pin').value
        self.motor_enable  = self.get_parameter('motor_enable_pin').value
        self.x_limit_pin   = self.get_parameter('x_limit_pin').value
        self.y_limit_pin   = self.get_parameter('y_limit_pin').value
        self.servo_pin     = self.get_parameter('servo_pin').value
        self.servo_release_pw = angle_to_pulsewidth(self.get_parameter('release_angle_deg').value)

        self.dir_setup_us      = self.get_parameter('dir_setup_us').value
        self.step_pulse_us     = self.get_parameter('step_pulse_us').value
        self.max_speed         = self.get_parameter('max_speed').value
        self.min_speed         = self.get_parameter('min_speed').value
        self.accel_ramp_steps  = self.get_parameter('accel_ramp_steps').value

        self.SPEED_FAST   = self.get_parameter('speed_fast_pct').value
        self.SPEED_SLOW   = self.get_parameter('speed_slow_pct').value
        self.SPEED_PREC   = self.get_parameter('speed_prec_pct').value
        self.backoff_steps     = self.get_parameter('backoff_steps').value
        self.MAX_HOMING_STEPS  = self.get_parameter('max_homing_steps').value
        self.BATCH_FAST        = self.get_parameter('batch_size_fast').value
        self.BATCH_PREC        = self.get_parameter('batch_size_prec').value

        self._x_max_mm    = self.get_parameter('x_max_mm').value
        self._steps_per_mm = self.get_parameter('steps_per_mm').value

        # ── State ──────────────────────────────────────────────────────────
        self.is_homed      = False
        self.emergency_stop = False

        # ── pigpio connection ──────────────────────────────────────────────
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().fatal(
                "Cannot connect to pigpiod daemon. Start it with: sudo pigpiod")
            raise RuntimeError("pigpiod not running")

        self._setup_gpio()

        # ── Gantry pin mutex ──────────────────────────────────────────────
        # Shared lock: compatible with stepper_driver_node's own shared lock.
        # Fails only if a bare-metal hardware test is holding the pins
        # exclusively — in that rare case we still start, but warn.
        self._pin_lock = GantryPinLock()
        if not self._pin_lock.acquire_shared():
            self.get_logger().error(
                'Could not acquire gantry GPIO pin lock — a raw hardware test '
                '(raw_motor/timing_sweep) may currently be driving these pins '
                'directly. Homing may race with it.')

        # ── Callback groups ──────────────────────────────────────────────
        # /emergency_stop must be able to preempt a blocking home_callback,
        # so it lives in its own group.
        self._estop_cb_group = ReentrantCallbackGroup()
        self._homing_cb_group = MutuallyExclusiveCallbackGroup()

        # ── ROS interfaces ─────────────────────────────────────────────────
        self.home_service = self.create_service(
            Trigger, '/gantry/home', self.home_callback,
            callback_group=self._homing_cb_group)

        self.status_pub = self.create_publisher(String, '/gantry/status', 10)

        # Publish Bool True here to reset stepper driver position counter to (0,0)
        self._reset_pos_pub = self.create_publisher(
            Bool, '/stepper/reset_position', 10)

        self.estop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.estop_callback, 10,
            callback_group=self._estop_cb_group)

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self.get_logger().info('Homing Node initialized (pigpio DMA)')
        self.get_logger().info(
            f'  Motor A: DIR={self.motorA_dir} STEP={self.motorA_step}  '
            f'Motor B: DIR={self.motorB_dir} STEP={self.motorB_step}  '
            f'EN={self.motor_enable}')
        self.get_logger().info(
            f'  X limit (at X_MAX, right): GPIO{self.x_limit_pin}  '
            f'Y limit (at Y=0, front): GPIO{self.y_limit_pin}')
        self.get_logger().info(
            f'  x_max_mm={self._x_max_mm:.0f}  steps_per_mm={self._steps_per_mm}')
        self.get_logger().info('  Service: /gantry/home')

    # ──────────────────────────────────────────────────────────────────────
    # GPIO setup
    # ──────────────────────────────────────────────────────────────────────

    def _setup_gpio(self):
        for pin in [self.motorA_dir, self.motorA_step,
                    self.motorB_dir, self.motorB_step, self.motor_enable]:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, 0)

        # Motors start disabled (A4988 ENABLE is active LOW)
        self.pi.write(self.motor_enable, 1)

        # Limit switches: active HIGH with pull-down
        for pin in [self.x_limit_pin, self.y_limit_pin]:
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pigpio.PUD_DOWN)

        self.pi.set_mode(self.servo_pin, pigpio.OUTPUT)

    # ──────────────────────────────────────────────────────────────────────
    # Cleanup / signals
    # ──────────────────────────────────────────────────────────────────────

    def _signal_handler(self, sig, frame):
        self.get_logger().info('Shutdown signal received')
        self._cleanup()

    def _cleanup(self):
        if getattr(self, 'pi', None) is None:
            return
        try:
            try:
                self.pi.wave_tx_stop()
            except Exception:
                pass
            self.pi.write(self.motor_enable, 1)
            self.pi.write(self.motorA_step, 0)
            self.pi.write(self.motorB_step, 0)
            self.pi.set_servo_pulsewidth(self.servo_pin, 0)
            try:
                self.pi.wave_clear()
            except Exception:
                pass
            if self.pi.connected:
                self.pi.stop()
        except Exception as e:
            self.get_logger().debug(f'Ignored exception during cleanup: {e}')
        if getattr(self, '_pin_lock', None) is not None:
            self._pin_lock.release()

    # ──────────────────────────────────────────────────────────────────────
    # Servo (magnet safety)
    # ──────────────────────────────────────────────────────────────────────

    def _disengage_magnet(self):
        """Raise the magnet (release position) before homing."""
        self.get_logger().info('  Disengaging magnet (raising servo)...')
        self.pi.set_servo_pulsewidth(self.servo_pin, self.servo_release_pw)
        time.sleep(0.5)
        self.pi.set_servo_pulsewidth(self.servo_pin, 0)

    # ──────────────────────────────────────────────────────────────────────
    # Limit switch readers
    # ──────────────────────────────────────────────────────────────────────

    def _read_x_limit(self) -> bool:
        """X limit is at X_MAX (right side). active HIGH."""
        return self.pi.read(self.x_limit_pin) == 1

    def _read_y_limit(self) -> bool:
        """Y limit is at Y=0 (front/closest to player). active HIGH."""
        return self.pi.read(self.y_limit_pin) == 1

    # ──────────────────────────────────────────────────────────────────────
    # Motor enable
    # ──────────────────────────────────────────────────────────────────────

    def _enable_motors(self):
        self.pi.write(self.motor_enable, 0)
        time.sleep(0.001)

    def _disable_motors(self):
        self.pi.write(self.motor_enable, 1)

    # ──────────────────────────────────────────────────────────────────────
    # DMA wave chain
    # ──────────────────────────────────────────────────────────────────────

    def _calculate_speed_profile(self, total_steps, speed_percent):
        if total_steps <= 0:
            return []

        target_speed = int(
            self.min_speed + (speed_percent / 100.0) * (self.max_speed - self.min_speed))

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
        """Step both motors simultaneously via DMA wave chain."""
        if steps_a == 0 and steps_b == 0:
            return

        # Positive steps → DIR LOW (inverted per this hardware)
        self.pi.write(self.motorA_dir, 0 if steps_a >= 0 else 1)
        self.pi.write(self.motorB_dir, 0 if steps_b >= 0 else 1)
        time.sleep(self.dir_setup_us / 1_000_000)

        abs_a = abs(steps_a)
        abs_b = abs(steps_b)
        max_steps = max(abs_a, abs_b)
        if max_steps == 0:
            return

        profile = self._calculate_speed_profile(max_steps, speed_percent)

        per_step_delays = []
        for cnt, delay in profile:
            per_step_delays.extend([delay] * cnt)

        # Bresenham interleave
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

        # Group consecutive identical patterns
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
                    raise RuntimeError(f'wave_create failed ({wid})')
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

    # ──────────────────────────────────────────────────────────────────────
    # CoreXY motion helpers (small-batch with limit checks)
    # ──────────────────────────────────────────────────────────────────────

    def _move_x(self, steps: int, speed_percent: int, batch_size: int = None) -> bool:
        """
        Move in the X direction in small batches, checking X limit each batch.

        CoreXY kinematics:
          +X (rightward, toward X limit): Motor A positive, Motor B negative
          -X (leftward, toward origin):   Motor A negative, Motor B positive

        X limit switch is at X_MAX (right side).
        Stops WITHOUT error when limit is triggered while moving in +X direction.
        Returns True if all steps completed, False if limit triggered or e-stop.
        """
        if batch_size is None:
            batch_size = self.BATCH_FAST

        sign = 1 if steps > 0 else -1
        remaining = abs(steps)

        self._enable_motors()
        try:
            while remaining > 0 and not self.emergency_stop:
                # Stop gracefully when hitting X limit while moving toward it (+X)
                if self._read_x_limit() and sign > 0:
                    return False
                batch = min(batch_size, remaining)
                dx = sign * batch
                # CoreXY: +X → A=+dx, B=-dx
                self._step_both(dx, -dx, speed_percent)
                remaining -= batch
        finally:
            self._disable_motors()

        return not self.emergency_stop

    def _move_y(self, steps: int, speed_percent: int, batch_size: int = None) -> bool:
        """
        Move in the Y direction in small batches, checking Y limit each batch.

        CoreXY kinematics:
          +Y (backward/away from player, toward rank 8): Motor A positive, Motor B positive
          -Y (forward/toward player, toward Y limit):    Motor A negative, Motor B negative

        Y limit switch is at Y=0 (front, closest to player).
        Stops WITHOUT error when limit is triggered while moving in -Y direction.
        Returns True if all steps completed, False if limit triggered or e-stop.
        """
        if batch_size is None:
            batch_size = self.BATCH_FAST

        sign = 1 if steps > 0 else -1
        remaining = abs(steps)

        self._enable_motors()
        try:
            while remaining > 0 and not self.emergency_stop:
                # Stop gracefully when hitting Y limit while moving toward it (-Y)
                if self._read_y_limit() and sign < 0:
                    return False
                batch = min(batch_size, remaining)
                dy = sign * batch
                # CoreXY: +Y → A=+dy, B=+dy
                self._step_both(dy, dy, speed_percent)
                remaining -= batch
        finally:
            self._disable_motors()

        return not self.emergency_stop

    # ──────────────────────────────────────────────────────────────────────
    # Prusa-style homing for each axis
    # ──────────────────────────────────────────────────────────────────────

    def _home_x(self) -> bool:
        """
        Home X axis (Prusa-style).

        X limit switch is at X_MAX (rightmost position).
        Approach from the left (+X direction) until switch triggers.
        Back off, then precision approach.
        After homing, gantry is at X = X_MAX.
        """
        self.get_logger().info('  Homing X (moving RIGHT toward X limit)...')

        # ── Phase 1: fast approach ────────────────────────────────────────
        self.get_logger().info('    Phase 1: fast approach')
        if not self._read_x_limit():
            self._move_x(self.MAX_HOMING_STEPS, self.SPEED_FAST)

        if not self._read_x_limit():
            self.get_logger().error('    X limit not triggered after max steps — check wiring!')
            return False
        self.get_logger().info('    X limit triggered (fast)')

        # ── Phase 2: back off ─────────────────────────────────────────────
        self.get_logger().info('    Phase 2: backing off')
        self._move_x(-self.backoff_steps, self.SPEED_SLOW)

        # If still triggered (backoff wasn't enough), back off more
        if self._read_x_limit():
            self.get_logger().info('    Still triggered — backing off further')
            self._move_x(-self.backoff_steps, self.SPEED_SLOW)

        if self._read_x_limit():
            self.get_logger().error('    X limit still triggered after double backoff — hardware issue?')
            return False

        # ── Phase 3: precision approach ───────────────────────────────────
        self.get_logger().info('    Phase 3: precision approach (small batches)')
        self._move_x(self.MAX_HOMING_STEPS, self.SPEED_PREC,
                     batch_size=self.BATCH_PREC)

        if self._read_x_limit():
            self.get_logger().info('    X homed successfully (at X_MAX)')
            return True
        else:
            self.get_logger().error('    X precision approach did not re-trigger limit')
            return False

    def _home_y(self) -> bool:
        """
        Home Y axis (Prusa-style).

        Y limit switch is at Y=0 (frontmost position, closest to player).
        Approach from the back (-Y direction) until switch triggers.
        Back off, then precision approach.
        After homing, gantry is at Y = 0.
        """
        self.get_logger().info('  Homing Y (moving FORWARD toward Y limit at front)...')

        # ── Phase 1: fast approach ────────────────────────────────────────
        self.get_logger().info('    Phase 1: fast approach')
        if not self._read_y_limit():
            self._move_y(-self.MAX_HOMING_STEPS, self.SPEED_FAST)

        if not self._read_y_limit():
            self.get_logger().error('    Y limit not triggered after max steps — check wiring!')
            return False
        self.get_logger().info('    Y limit triggered (fast)')

        # ── Phase 2: back off ─────────────────────────────────────────────
        self.get_logger().info('    Phase 2: backing off')
        self._move_y(self.backoff_steps, self.SPEED_SLOW)

        if self._read_y_limit():
            self.get_logger().info('    Still triggered — backing off further')
            self._move_y(self.backoff_steps, self.SPEED_SLOW)

        if self._read_y_limit():
            self.get_logger().error('    Y limit still triggered after double backoff — hardware issue?')
            return False

        # ── Phase 3: precision approach ───────────────────────────────────
        self.get_logger().info('    Phase 3: precision approach (small batches)')
        self._move_y(-self.MAX_HOMING_STEPS, self.SPEED_PREC,
                     batch_size=self.BATCH_PREC)

        if self._read_y_limit():
            self.get_logger().info('    Y homed successfully (at Y=0)')
            return True
        else:
            self.get_logger().error('    Y precision approach did not re-trigger limit')
            return False

    def _drive_to_origin(self):
        """
        After homing, gantry is at (X_MAX, 0).
        Drive X back to 0 (bottom-left corner = logical origin).
        """
        x_steps = int(self._x_max_mm * self._steps_per_mm)
        self.get_logger().info(
            f'  Driving to origin: moving -{x_steps} steps in X '
            f'({self._x_max_mm:.0f} mm leftward)...')
        self._move_x(-x_steps, self.SPEED_SLOW)
        self.get_logger().info('  At origin (0, 0)')

    # ──────────────────────────────────────────────────────────────────────
    # Service callback
    # ──────────────────────────────────────────────────────────────────────

    def home_callback(self, request, response):
        self.get_logger().info('=== Homing sequence started ===')

        self._pub_status('HOMING_STARTED')
        self.emergency_stop = False

        # Safety: raise magnet before any movement
        self._disengage_magnet()

        # ── Home Y first (front limit, closest to player) ─────────────────
        self.get_logger().info('Step 1/3: Home Y axis')
        if not self._home_y():
            self._pub_status('HOMING_FAILED')
            response.success = False
            response.message = 'Y homing failed — limit not reached'
            return response

        # ── Home X (right-side limit) ─────────────────────────────────────
        self.get_logger().info('Step 2/3: Home X axis')
        if not self._home_x():
            self._pub_status('HOMING_FAILED')
            response.success = False
            response.message = 'X homing failed — limit not reached'
            return response

        # ── Drive to coordinate origin (0, 0) = bottom-left ──────────────
        # We're currently at (X_MAX, 0). Drive X leftward to X=0.
        self.get_logger().info('Step 3/3: Drive to coordinate origin')
        self._drive_to_origin()

        # ── Reset stepper driver step counter ─────────────────────────────
        # Gantry is now physically at (0, 0). Tell stepper driver.
        self._reset_pos_pub.publish(Bool(data=True))
        time.sleep(0.05)   # let the stepper driver process the reset

        self.is_homed = True
        self._pub_status('HOMED')
        self.get_logger().info('=== Homing complete — origin (0, 0) set ===')

        response.success = True
        response.message = 'Homing complete. Gantry at (0, 0).'
        return response

    def _pub_status(self, status: str):
        self.status_pub.publish(String(data=status))

    def estop_callback(self, msg):
        if msg.data:
            self.get_logger().warn('Emergency stop triggered!')
            self.emergency_stop = True
        elif self.emergency_stop:
            self.emergency_stop = False
            self.get_logger().info('Emergency stop cleared')


def main(args=None):
    rclpy.init(args=args)
    node = HomingNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        node._cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
