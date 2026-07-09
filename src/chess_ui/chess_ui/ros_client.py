"""The ROS 2 client node backing Chess OS. Every subscription here just
writes into `state._state` (or pushes into a queue for SSE streaming);
all actual robot control/perception logic lives in the ROS packages —
this is a thin client, not a second implementation of any of it."""

import json
import queue
import time

import cv2
import numpy as np

from . import camera, state

# ── Optional ROS2 ─────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Bool, Float32
    from sensor_msgs.msg import Image, CompressedImage
    from geometry_msgs.msg import Point as RosPoint, Twist
    from std_srvs.srv import Trigger
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

try:
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    HAS_RCL_PARAMS = True
except ImportError:
    HAS_RCL_PARAMS = False

# Set by app.main() once ROS is (or isn't) initialized; read by routes.py.
ros_node = None


if HAS_ROS:
    class RosNode(Node):
        def __init__(self):
            super().__init__("chess_os")

            # ── Subscriptions ──────────────────────────────────────────
            self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self._on_img, 10)
            self.create_subscription(String,  "/game_manager/board_fen",     self._on_fen,            10)
            self.create_subscription(String,  "/game_manager/state",         self._on_state,          10)
            self.create_subscription(String,  "/game_manager/turn",          self._on_turn,           10)
            self.create_subscription(String,  "/perception/changed_squares", self._on_changed_squares,10)
            self.create_subscription(Bool,    "/limit_switch/x_min",         self._on_lim_x,          10)
            self.create_subscription(Bool,    "/limit_switch/y_min",         self._on_lim_y,          10)
            self.create_subscription(Bool,    "/limit_switch/clock_hit",     self._on_lim_clock,      10)
            self.create_subscription(RosPoint,"/gantry/pose",                self._on_gantry_pose,    10)
            self.create_subscription(String,  "/servo/state",                self._on_servo_state,    10)
            self.create_subscription(String,  "/gantry/status",              self._on_gantry_status,  10)
            self.create_subscription(String,  "/stepper/status",             self._on_stepper_status, 10)
            self.create_subscription(Float32, "/clock/white_time",           self._on_white_time,     10)
            self.create_subscription(Float32, "/clock/black_time",           self._on_black_time,     10)
            # Game control / perception data from ROS
            self.create_subscription(String,  "/game_manager/move_history",  self._on_move_history,   10)
            self.create_subscription(String,  "/game_manager/result",        self._on_result,         10)
            self.create_subscription(String,  "/perception/square_scores",   self._on_square_scores,  10)
            self.create_subscription(Image,   "/perception/piece_debug",     self._on_diff_img,       10)
            self.create_subscription(String,  "/perception/reference_status",self._on_ref_status,     10)
            self.create_subscription(String,  "/game_manager/capture_progress", self._on_capture_progress, 10)

            # ── Publishers ─────────────────────────────────────────────
            self.vel_pub       = self.create_publisher(Twist,    "/stepper/velocity",       10)
            self.cmd_pub       = self.create_publisher(RosPoint, "/stepper/command",        10)
            self.estop_pub     = self.create_publisher(Bool,     "/emergency_stop",         10)
            self.reset_pos_pub = self.create_publisher(Bool,     "/stepper/reset_position", 10)
            self.set_time_pub  = self.create_publisher(Float32,  "/clock/set_time",         10)

            # ── Service clients ────────────────────────────────────────
            self._svc_engage       = self.create_client(Trigger, "/servo/engage")
            self._svc_release      = self.create_client(Trigger, "/servo/release")
            self._svc_home         = self.create_client(Trigger, "/gantry/home")
            self._svc_clock        = self.create_client(Trigger, "/clock/hit")
            self._svc_clock_reset  = self.create_client(Trigger, "/clock/reset")
            self._svc_clock_pause  = self.create_client(Trigger, "/clock/pause")
            self._svc_clock_resume = self.create_client(Trigger, "/clock/resume")
            # Game control services
            self._svc_game_start    = self.create_client(Trigger, "/game/start")
            self._svc_game_new      = self.create_client(Trigger, "/game/new_game")
            self._svc_game_resign   = self.create_client(Trigger, "/game/resign")
            self._svc_cap_reference = self.create_client(Trigger, "/perception/capture_premove")
            # Detector parameter client (optional — requires rcl_interfaces)
            self._svc_detector_params = None
            self._svc_game_manager_params = None
            self._svc_motion_planner_params = None
            self._svc_camera_params = None
            if HAS_RCL_PARAMS:
                self._svc_detector_params = self.create_client(
                    SetParameters, "/piece_detector_node/set_parameters")
                self._svc_game_manager_params = self.create_client(
                    SetParameters, "/game_manager_node/set_parameters")
                self._svc_motion_planner_params = self.create_client(
                    SetParameters, "/motion_planner_node/set_parameters")
                self._svc_camera_params = self.create_client(
                    SetParameters, "/camera_node/set_parameters")

            # Pending param updates (set by Flask thread, applied by ROS timer)
            self._pending_detector_params = None
            self._pending_game_manager_params = None
            self._pending_motion_planner_params = None
            self._pending_camera_params = None
            self.create_timer(0.2, self._check_pending)

            # ── Action client for point-to-point gantry moves ─────────
            try:
                from chess_interfaces.action import MoveGantry as _MGA
                from rclpy.action import ActionClient as _AClient
                self._move_ac = _AClient(self, _MGA, '/gantry/move')
                self._has_move_action = True
            except ImportError:
                self._move_ac = None
                self._has_move_action = False

            # ── Action client for hardware test orchestration ─────────
            # test_runner_node (chess_hw_interface) owns the actual subprocess;
            # chess_os is just a client streaming its feedback into the UI.
            self._test_queue      = queue.Queue()
            self._test_running    = False
            self._test_last_run   = None
            self._test_goal_handle = None
            try:
                from chess_interfaces.action import RunHardwareTest as _RHT
                from rclpy.action import ActionClient as _AClient2
                self._test_ac = _AClient2(self, _RHT, '/hw_test/run')
                self._has_test_action = True
            except ImportError:
                self._test_ac = None
                self._has_test_action = False

            with state._lock:
                state._state["ros_connected"] = True
            self.get_logger().info("ChessOS ROS2 node ready")

        def send_goto(self, x_mm: float, y_mm: float, speed: float = 40.0) -> dict:
            """Fire-and-forget MoveGantry action. Position updates via /gantry/pose."""
            if not self._has_move_action or self._move_ac is None:
                return {"ok": False, "msg": "chess_interfaces not installed"}
            if not self._move_ac.wait_for_server(timeout_sec=1.0):
                return {"ok": False, "msg": "/gantry/move action server not running"}
            try:
                from chess_interfaces.action import MoveGantry as _MGA
                goal = _MGA.Goal()
                goal.target_x_mm   = float(x_mm)
                goal.target_y_mm   = float(y_mm)
                goal.speed_mm_s    = float(speed)
                goal.engage_magnet = False
                self._move_ac.send_goal_async(goal)
                return {"ok": True, "msg": f"Moving to ({x_mm:.1f}, {y_mm:.1f}) mm"}
            except Exception as e:
                return {"ok": False, "msg": str(e)}

        def start_test(self, category: str, subtest: str) -> bool:
            if not self._has_test_action or self._test_ac is None:
                return False
            if self._test_running:
                return False
            if not self._test_ac.wait_for_server(timeout_sec=1.0):
                return False
            from chess_interfaces.action import RunHardwareTest as _RHT
            goal = _RHT.Goal()
            goal.category = category
            goal.subtest  = subtest
            # Drain any leftover items from a previous run whose SSE stream
            # disconnected mid-test — otherwise the next /api/tests/stream
            # client would immediately receive that stale result instead of
            # this run's live output.
            while not self._test_queue.empty():
                try:
                    self._test_queue.get_nowait()
                except queue.Empty:
                    break
            self._test_running     = True
            self._test_last_run    = f"{category}/{subtest}"
            self._test_goal_handle = None
            send_future = self._test_ac.send_goal_async(
                goal, feedback_callback=self._on_test_feedback)
            send_future.add_done_callback(self._on_test_goal_response)
            return True

        def stop_test(self):
            if self._test_goal_handle is not None:
                self._test_goal_handle.cancel_goal_async()

        def _on_test_goal_response(self, future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._test_queue.put({"type": "error",
                                      "line": "Goal rejected — test_runner_node busy or unavailable"})
                self._test_running = False
                return
            self._test_goal_handle = goal_handle
            goal_handle.get_result_async().add_done_callback(self._on_test_result)

        def _on_test_feedback(self, feedback_msg):
            self._test_queue.put({"type": "line", "line": feedback_msg.feedback.line})

        def _on_test_result(self, future):
            result = future.result().result
            self._test_queue.put({"type": "done", "line": result.message,
                                  "rc": result.return_code})
            self._test_running    = False
            self._test_goal_handle = None

        def stream_test_sse(self):
            while True:
                try:
                    item = self._test_queue.get(timeout=30)
                except queue.Empty:
                    yield 'data: {"type":"ping"}\n\n'
                    continue
                yield f"data: {json.dumps(item)}\n\n"
                if item.get("type") in ("done", "error"):
                    return

        def _on_img(self, msg):
            """/camera/image_raw/compressed — JPEG bytes, same topic board_detector_node
            and piece_detector_node already consume (the uncompressed /camera/image_raw
            topic exists in the graph but camera_ros never actually publishes frames on it)."""
            try:
                data  = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
                if frame is None:
                    raise ValueError("JPEG decode returned None")
                h, w = frame.shape[:2]
                camera.camera.update_raw(camera.to_jpeg(frame, 75))
                with state._lock:
                    state._state["raw_frame"] = frame
                    state._state["cam_info"]  = f"{w}×{h} jpeg"
            except Exception as e:
                with state._lock:
                    state._state["error"] = f"img: {e}"

        def _on_fen(self, msg):
            fen = msg.data.strip()
            if fen:
                with state._lock:
                    state._state["game_fen"]     = fen
                    state._state["fen_source"]   = "game_mgr"
                    state._state["last_updated"] = time.time()
                    state._state["frame_count"] += 1

        def _on_state(self, msg):
            with state._lock:
                state._state["game_state"] = msg.data.strip()

        def _on_turn(self, msg):
            with state._lock:
                state._state["game_turn"] = msg.data.strip()

        def _on_changed_squares(self, msg):
            raw = msg.data.strip()
            with state._lock:
                state._state["last_move"]    = raw
                state._state["last_updated"] = time.time()
                state._state["frame_count"] += 1

        def _on_lim_x(self, msg):
            with state._lock: state._state["limit_x"] = msg.data

        def _on_lim_y(self, msg):
            with state._lock: state._state["limit_y"] = msg.data

        def _on_lim_clock(self, msg):
            with state._lock: state._state["limit_clock"] = msg.data

        def _on_gantry_pose(self, msg):
            with state._lock:
                state._state["gantry_x"] = msg.x
                state._state["gantry_y"] = msg.y

        def _on_servo_state(self, msg):
            with state._lock:
                state._state["magnet_engaged"] = (msg.data == "engaged")

        def _on_gantry_status(self, msg):
            with state._lock:
                state._state["gantry_status"] = msg.data
                if msg.data == "HOMED":
                    state._state["gantry_homed"] = True

        def _on_stepper_status(self, msg):
            with state._lock: state._state["stepper_status"] = msg.data

        def _on_white_time(self, msg):
            with state._lock: state._state["white_time"] = msg.data

        def _on_black_time(self, msg):
            with state._lock: state._state["black_time"] = msg.data

        def _on_move_history(self, msg):
            uci_str = msg.data.strip()
            uci_list = uci_str.split() if uci_str else []
            with state._lock:
                state._state["move_history_uci"] = uci_list
                state._state["move_history"]     = uci_list

        def _on_result(self, msg):
            with state._lock:
                state._state["game_result"] = msg.data.strip()

        def _on_square_scores(self, msg):
            try:
                with state._lock:
                    state._state["square_scores"] = json.loads(msg.data)
            except Exception as e:
                self.get_logger().debug(f'square_scores decode failed: {e}')

        def _on_diff_img(self, msg):
            try:
                h, w = msg.height, msg.width
                data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                img  = data.reshape((h, w, 3))
                ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ok:
                    with state._lock:
                        state._state["diff_frame"] = bytes(buf)
            except Exception as e:
                self.get_logger().debug(f'diff_img decode failed: {e}')

        def _on_ref_status(self, msg):
            with state._lock:
                state._state["ref_status"] = msg.data

        def _on_capture_progress(self, msg):
            with state._lock:
                state._state["capture_progress"] = msg.data

        def request_detector_params(self, threshold, compensation,
                                    clump_enable=False, clump_keep=1,
                                    roi_bias=0.0, edge_weight=0.0):
            if HAS_RCL_PARAMS:
                self._pending_detector_params = {
                    "diff_threshold":            threshold,
                    "global_shift_compensation": compensation,
                    "clump_enable":              clump_enable,
                    "clump_keep_per_group":      clump_keep,
                    "roi_bias_fraction":         roi_bias,
                    "edge_diff_weight":          edge_weight,
                }

        def request_game_manager_params(self, think_time_s: float):
            if HAS_RCL_PARAMS:
                self._pending_game_manager_params = {
                    "engine_think_time_s": think_time_s,
                }

        def request_exposure_lock(self, locked: bool):
            """Lock (or unlock) camera_node's auto-exposure/auto-white-balance.

            Disabling AeEnable/AwbEnable freezes the sensor at whatever
            exposure/gain/white-balance it had converged to — killing the
            board-wide brightness/color drift between the pre-move reference
            and post-move captures that otherwise looks like phantom piece
            movement on every square. camera_ros's ExposureTime/AnalogueGain/
            ColourGains parameters can't be reliably read back (they report
            "not set" until explicitly written at least once, with no way to
            query the AE-computed live value), so this only toggles Ae/AwbEnable
            rather than trying to sample-then-pin specific values.
            """
            if HAS_RCL_PARAMS:
                self._pending_camera_params = {
                    "AeEnable":  not locked,
                    "AwbEnable": not locked,
                }

        def request_motion_planner_params(self, origin_x_mm, origin_y_mm, sq_size_mm):
            """Push calibrated board geometry to motion_planner_node — the node
            that actually drives the gantry during real games. Without this,
            calibrating via the Chess OS UI only affected chess_os's own local
            square_to_mm() (used by the manual Square Nav debug tool), leaving
            motion_planner_node on its uncalibrated declare_parameter defaults."""
            if HAS_RCL_PARAMS:
                self._pending_motion_planner_params = {
                    "board_origin_x_mm": origin_x_mm,
                    "board_origin_y_mm": origin_y_mm,
                    "square_size_mm":    sq_size_mm,
                }

        def _check_pending(self):
            if not HAS_RCL_PARAMS:
                return
            # Only clear each pending dict once _apply_pending_params actually
            # sent the request — if the target's SetParameters service isn't
            # ready yet (e.g. still starting up), keep it pending and retry
            # on the next 0.2s tick instead of silently discarding the push
            # (previously: unconditionally cleared here regardless of
            # whether the service was ready, so a push that arrived before
            # the target node was fully up was lost forever with no error).
            if self._apply_pending_params(
                self._pending_detector_params,
                self._svc_detector_params,
                "piece_detector_node",
                {
                    "diff_threshold":            ParameterType.PARAMETER_DOUBLE,
                    "global_shift_compensation": ParameterType.PARAMETER_DOUBLE,
                    "clump_enable":              ParameterType.PARAMETER_BOOL,
                    "clump_keep_per_group":      ParameterType.PARAMETER_INTEGER,
                    "roi_bias_fraction":         ParameterType.PARAMETER_DOUBLE,
                    "edge_diff_weight":          ParameterType.PARAMETER_DOUBLE,
                },
            ):
                self._pending_detector_params = None

            if self._apply_pending_params(
                self._pending_game_manager_params,
                self._svc_game_manager_params,
                "game_manager_node",
                {"engine_think_time_s": ParameterType.PARAMETER_DOUBLE},
            ):
                self._pending_game_manager_params = None

            if self._apply_pending_params(
                self._pending_motion_planner_params,
                self._svc_motion_planner_params,
                "motion_planner_node",
                {
                    "board_origin_x_mm": ParameterType.PARAMETER_DOUBLE,
                    "board_origin_y_mm": ParameterType.PARAMETER_DOUBLE,
                    "square_size_mm":    ParameterType.PARAMETER_DOUBLE,
                },
            ):
                self._pending_motion_planner_params = None

            if self._apply_pending_params(
                self._pending_camera_params,
                self._svc_camera_params,
                "camera_node",
                {
                    "AeEnable":  ParameterType.PARAMETER_BOOL,
                    "AwbEnable": ParameterType.PARAMETER_BOOL,
                },
            ):
                self._pending_camera_params = None

        def _apply_pending_params(self, params, client, target_name, type_map) -> bool:
            """Returns True once the update has been sent (or there was
            nothing to send) — False means "keep retrying", i.e. the caller
            must NOT clear the pending value yet."""
            if params is None or client is None:
                return True
            if not client.service_is_ready():
                return False
            req = SetParameters.Request()
            for name, value in params.items():
                pv = ParameterValue()
                ptype = type_map.get(name, ParameterType.PARAMETER_DOUBLE)
                pv.type = ptype
                if ptype == ParameterType.PARAMETER_DOUBLE:
                    pv.double_value = float(value)
                elif ptype == ParameterType.PARAMETER_BOOL:
                    pv.bool_value = bool(int(value))
                elif ptype == ParameterType.PARAMETER_INTEGER:
                    pv.integer_value = int(value)
                p = Parameter()
                p.name = name
                p.value = pv
                req.parameters.append(p)
            future = client.call_async(req)
            future.add_done_callback(
                lambda f, target=target_name: self._log_param_result(f, target))
            return True

        def _log_param_result(self, future, target_name: str):
            try:
                results = future.result().results
            except Exception as e:
                self.get_logger().warn(f'{target_name} param push failed: {e}')
                return
            for r in results:
                if not r.successful:
                    self.get_logger().warn(
                        f'{target_name} rejected a param push: {r.reason}')

        def call_svc(self, client, timeout: float = 2.0):
            if not client.wait_for_service(timeout_sec=0.5):
                return False, "Service not available"
            future = client.call_async(Trigger.Request())
            # Poll — the background rclpy.spin thread processes the response
            deadline = time.monotonic() + timeout
            while not future.done():
                if time.monotonic() > deadline:
                    return False, "Timeout"
                time.sleep(0.05)
            r = future.result()
            return r.success, r.message


def jog_loop():
    """20 Hz loop: publish velocity to /stepper/velocity while jog is active.

    Watchdog: if /api/gantry/jog/start hasn't refreshed last_refresh within
    state.JOG_WATCHDOG_TIMEOUT_S, auto-stop and zero the velocity — guards
    against a closed tab / dropped connection leaving the gantry jogging
    indefinitely (no mouseup/keyup/blur ever fires in that case)."""
    while True:
        time.sleep(0.05)
        with state._jog_lock:
            if not state._jog["active"]:
                continue
            stale = (time.time() - state._jog["last_refresh"]) > state.JOG_WATCHDOG_TIMEOUT_S
            if stale:
                state._jog["active"] = False
            else:
                dx, dy, spd = state._jog["dx"], state._jog["dy"], state._jog["speed"]
        node = ros_node
        if node is None:
            continue
        if stale:
            try:
                if HAS_ROS:
                    node.vel_pub.publish(Twist())
                    node.get_logger().warn(
                        'Jog watchdog: no refresh received, auto-stopping')
            except Exception:
                pass
            continue
        MAX_VEL = 600.0
        try:
            if HAS_ROS:
                msg = Twist()
                msg.linear.x = float(dx * MAX_VEL * spd / 100.0)
                msg.linear.y = float(dy * MAX_VEL * spd / 100.0)
                node.vel_pub.publish(msg)
        except Exception:
            pass
