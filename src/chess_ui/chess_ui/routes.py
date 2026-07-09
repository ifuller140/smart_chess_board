"""Flask route registration for Chess OS. Every route either reads/writes
`state._state` directly, or delegates to the current `ros_client.ros_node`
for anything that talks to ROS — no vision/coordinate logic lives here."""

import time

import cv2
from flask import Response, jsonify, render_template, request

from . import camera as camera_mod
from . import ros_client, state


def _call_svc(attr: str):
    node = ros_client.ros_node
    if node is None:
        return jsonify({"ok": False, "msg": "ROS not connected"}), 503
    ok, msg = node.call_svc(getattr(node, attr))
    return jsonify({"ok": ok, "msg": msg})


EXPECTED_NODES = [
    "stepper_driver_node", "servo_node", "limit_switch_node",
    "clock_display_node",  "camera_node", "board_detector_node",
    "piece_detector_node", "gantry_kinematics_node", "motion_planner_node",
    "homing_node",         "game_manager_node",       "chess_engine_node",
]


def register_routes(app):

    # ── Core routes ───────────────────────────────────────────────────────
    @app.route("/")
    def index():
        return render_template("index.html")

    @app.route("/api/stream/raw")
    def stream_raw():
        return Response(camera_mod.camera.mjpeg_stream(),
                        mimetype="multipart/x-mixed-replace; boundary=frame",
                        headers={"Cache-Control": "no-cache",
                                 "X-Accel-Buffering": "no"})

    @app.route("/api/status")
    def api_status():
        with state._lock:
            s = state._state
            return jsonify({
                "frame_count":    s["frame_count"],
                "last_updated":   s["last_updated"],
                "cam_info":       s["cam_info"],
                "error":          s["error"],
                "fen":            state.best_fen(),
                "fen_source":     s["fen_source"],
                "game_state":     s["game_state"],
                "game_turn":      s["game_turn"],
                "last_move":      s["last_move"],
                "move_history":   s["move_history"],
                "white_time":     s["white_time"],
                "black_time":     s["black_time"],
                "limit_x":        s["limit_x"],
                "limit_y":        s["limit_y"],
                "limit_clock":    s["limit_clock"],
                "gantry_x":       s["gantry_x"],
                "gantry_y":       s["gantry_y"],
                "gantry_homed":   s["gantry_homed"],
                "gantry_status":  s["gantry_status"],
                "stepper_status": s["stepper_status"],
                "magnet_engaged": s["magnet_engaged"],
                "estop_active":   s["estop_active"],
                "ros_connected":  s["ros_connected"],
                "calib_applied":  s["calib_applied"],
                "calib_a1_x":     s["calib_a1_x"],
                "calib_a1_y":     s["calib_a1_y"],
                "calib_h8_x":     s["calib_h8_x"],
                "calib_h8_y":     s["calib_h8_y"],
                "calib_sq_x":     s["calib_sq_x"],
                "calib_sq_y":     s["calib_sq_y"],
                # Game control
                "move_history_uci":           s["move_history_uci"],
                "game_result":                s["game_result"],
                "think_time_s":               s.get("think_time_s", 2.0),
                "session_reference_captured": s["session_reference_captured"],
                "ref_status":                 s["ref_status"],
                "exposure_locked":            s["exposure_locked"],
                "capture_progress":           s["capture_progress"],
            })

    @app.route("/api/snapshot")
    def api_snapshot():
        with state._lock:
            frame = state._state["raw_frame"]
        if frame is None:
            return Response(b'', status=204)
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        return Response(bytes(buf), mimetype="image/jpeg",
                        headers={"Content-Disposition":
                                 "attachment; filename=snapshot.jpg"})

    # ── FEN / game routes ─────────────────────────────────────────────────
    @app.route("/api/fen")
    def api_fen_get():
        with state._lock:
            return jsonify({"fen": state.best_fen(), "source": state._state["fen_source"]})

    @app.route("/api/fen", methods=["POST"])
    def api_fen_set():
        data = request.get_json(silent=True) or {}
        fen  = data.get("fen", "").strip()
        if not fen:
            return jsonify({"ok": False}), 400
        with state._lock:
            state._state["game_fen"]     = fen
            state._state["fen_source"]   = "local"
            state._state["last_updated"] = time.time()
            state._state["frame_count"] += 1
        return jsonify({"ok": True})

    # ── Hardware routes ───────────────────────────────────────────────────
    @app.route("/api/hw/servo/engage", methods=["POST"])
    def api_servo_engage():
        resp = _call_svc("_svc_engage")
        if isinstance(resp, tuple):
            return resp
        with state._lock:
            state._state["magnet_engaged"] = True
        return resp

    @app.route("/api/hw/servo/release", methods=["POST"])
    def api_servo_release():
        resp = _call_svc("_svc_release")
        if isinstance(resp, tuple):
            return resp
        with state._lock:
            state._state["magnet_engaged"] = False
        return resp

    @app.route("/api/hw/gantry/home", methods=["POST"])
    def api_gantry_home():
        resp = _call_svc("_svc_home")
        if isinstance(resp, tuple):
            return resp
        with state._lock:
            state._state["gantry_x"]     = 0.0
            state._state["gantry_y"]     = 0.0
            state._state["gantry_homed"] = True
        return resp

    @app.route("/api/hw/clock/hit", methods=["POST"])
    def api_clock_hit():
        return _call_svc("_svc_clock")

    @app.route("/api/clock/reset", methods=["POST"])
    def api_clock_reset():
        return _call_svc("_svc_clock_reset")

    @app.route("/api/clock/pause", methods=["POST"])
    def api_clock_pause():
        return _call_svc("_svc_clock_pause")

    @app.route("/api/clock/resume", methods=["POST"])
    def api_clock_resume():
        return _call_svc("_svc_clock_resume")

    @app.route("/api/hw/estop", methods=["POST"])
    def api_estop():
        with state._lock:
            state._state["estop_active"] = True
        with state._jog_lock:
            state._jog["active"] = False
        node = ros_client.ros_node
        if node is not None:
            try:
                if ros_client.HAS_ROS:
                    node.estop_pub.publish(ros_client.Bool(data=True))
                    node.vel_pub.publish(ros_client.Twist())
            except Exception as e:
                return jsonify({"ok": False, "msg": str(e)}), 500
        return jsonify({"ok": True, "msg": "EMERGENCY STOP sent"})

    @app.route("/api/hw/estop/clear", methods=["POST"])
    def api_estop_clear():
        """Clear the local UI flag AND the latched emergency_stop on every
        hardware node — previously this only did the former, so the gantry/
        servo stayed silently unresponsive after a real e-stop until the ROS
        processes were manually restarted."""
        with state._lock:
            state._state["estop_active"] = False
        node = ros_client.ros_node
        if node is not None:
            try:
                if ros_client.HAS_ROS:
                    node.estop_pub.publish(ros_client.Bool(data=False))
            except Exception as e:
                return jsonify({"ok": False, "msg": str(e)}), 500
        return jsonify({"ok": True})

    @app.route("/api/stepper/reset_position", methods=["POST"])
    def api_stepper_reset_position():
        """Reset stepper position counter to (0,0). Call after manual homing or calibration."""
        node = ros_client.ros_node
        if node is None:
            return jsonify({"ok": False, "msg": "ROS not connected"}), 503
        try:
            if ros_client.HAS_ROS:
                node.reset_pos_pub.publish(ros_client.Bool(data=True))
            with state._lock:
                state._state["gantry_x"] = 0.0
                state._state["gantry_y"] = 0.0
            return jsonify({"ok": True, "msg": "Position reset to (0, 0)"})
        except Exception as e:
            return jsonify({"ok": False, "msg": str(e)}), 500

    @app.route("/api/capture_premove", methods=["POST"])
    def api_capture_premove():
        """Backward-compatible alias for /api/perception/capture_reference."""
        return api_capture_reference()

    @app.route("/api/perception/capture_reference", methods=["POST"])
    def api_capture_reference():
        """Call /perception/capture_premove and mark session_reference_captured
        only if the service call actually reported success."""
        node = ros_client.ros_node
        if node is None:
            return jsonify({"ok": False, "msg": "ROS not connected"}), 503
        ok, msg = node.call_svc(node._svc_cap_reference)
        if ok:
            with state._lock:
                state._state["session_reference_captured"] = True
                state._state["ref_status"] = "Reference captured ✓"
        return jsonify({"ok": ok, "msg": msg})

    @app.route("/api/perception/exposure/lock", methods=["POST"])
    def api_exposure_lock():
        """Lock/unlock camera_node's auto-exposure/auto-white-balance.

        Continuous AE/AWB otherwise re-adjusts brightness/color between the
        pre-move reference and post-move captures even with nothing
        physically moved, showing up as board-wide phantom diff. Locking
        freezes the sensor at whatever it had converged to.
        """
        data   = request.get_json(silent=True) or {}
        locked = bool(data.get("locked", True))
        with state._lock:
            state._state["exposure_locked"] = locked
        node = ros_client.ros_node
        if node is not None and ros_client.HAS_RCL_PARAMS:
            node.request_exposure_lock(locked)
            return jsonify({"ok": True, "locked": locked})
        elif node is not None:
            return jsonify({"ok": False,
                            "msg": "rcl_interfaces not available — cannot push to camera_node"}), 503
        return jsonify({"ok": False, "msg": "ROS not connected"}), 503

    @app.route("/api/diff_frame")
    def api_diff_frame():
        """Piece detector LAB diff heatmap (from /perception/piece_debug)."""
        with state._lock:
            data = state._state.get("diff_frame")
        if not data:
            data = camera_mod.blank_jpeg(400, 400, "Waiting for piece_detector...")
        return Response(data, mimetype="image/jpeg",
                        headers={"Cache-Control": "no-store"})

    @app.route("/api/square_scores")
    def api_square_scores():
        with state._lock:
            return jsonify({
                "scores":               dict(state._state.get("square_scores", {})),
                "display_threshold":    state._params.get("display_threshold", 18.0),
                "shift_compensation":   state._params.get("shift_compensation", 1.0),
                "clump_enable":         int(state._params.get("clump_enable", 0)),
                "clump_keep_per_group": int(state._params.get("clump_keep_per_group", 1)),
            })

    @app.route("/api/detector_params", methods=["POST"])
    def api_detector_params():
        """Push diff threshold + compensation + clump settings to piece_detector_node."""
        data = request.get_json(silent=True) or {}
        thresh       = float(data.get("diff_threshold", 18.0))
        comp         = float(data.get("shift_compensation", 1.0))
        clump_enable = bool(data.get("clump_enable", False))
        clump_keep   = int(data.get("clump_keep_per_group", 1))
        with state._lock:
            state._params["display_threshold"]    = thresh
            state._params["shift_compensation"]   = comp
            state._params["clump_enable"]         = int(clump_enable)
            state._params["clump_keep_per_group"] = clump_keep
        node = ros_client.ros_node
        if node is not None and ros_client.HAS_RCL_PARAMS:
            node.request_detector_params(thresh, comp, clump_enable, clump_keep)
            return jsonify({"ok": True,
                            "msg": f"Applied threshold={thresh}, comp={comp}, "
                                   f"clump={clump_enable}(keep={clump_keep})"})
        elif node is not None:
            return jsonify({"ok": False,
                            "msg": "rcl_interfaces not available — params stored locally only"}), 503
        return jsonify({"ok": False, "msg": "ROS not connected"}), 503

    # ── Game control routes ─────────────────────────────────────────────────
    @app.route("/api/game/start", methods=["POST"])
    def api_game_start():
        """Trigger game start (same as pressing the physical clock from IDLE)."""
        return _call_svc("_svc_game_start")

    @app.route("/api/game/new", methods=["POST"])
    def api_game_new():
        """Reset to a fresh game."""
        with state._lock:
            state._state["game_result"]      = ""
            state._state["move_history_uci"] = []
            state._state["move_history"]     = []
        return _call_svc("_svc_game_new")

    @app.route("/api/game/resign", methods=["POST"])
    def api_game_resign():
        """Resign the current game."""
        return _call_svc("_svc_game_resign")

    @app.route("/api/game/settings", methods=["POST"])
    def api_game_settings():
        """Update think time and/or clock time per player."""
        data = request.get_json(silent=True) or {}
        msgs = []
        node = ros_client.ros_node
        if "think_time_s" in data:
            think_s = float(data["think_time_s"])
            with state._lock:
                state._state["think_time_s"] = think_s
            if node is not None and ros_client.HAS_RCL_PARAMS:
                node.request_game_manager_params(think_s)
                msgs.append(f"think_time={think_s}s (applied to game_manager_node)")
            else:
                msgs.append(f"think_time={think_s}s (stored; ROS not available)")
        if "time_per_player_min" in data and node is not None and ros_client.HAS_ROS:
            try:
                m = ros_client.Float32()
                m.data = float(data["time_per_player_min"])
                node.set_time_pub.publish(m)
                msgs.append(f"clock={data['time_per_player_min']}min")
            except Exception as e:
                return jsonify({"ok": False, "msg": str(e)}), 500
        return jsonify({"ok": True, "applied": msgs})

    # ── Node health ──────────────────────────────────────────────────────────
    @app.route("/api/nodes/health")
    def api_nodes_health():
        node = ros_client.ros_node
        if node is None or not ros_client.HAS_ROS:
            return jsonify({n: False for n in EXPECTED_NODES})
        try:
            alive = set(node.get_node_names())
            return jsonify({n: (n in alive) for n in EXPECTED_NODES})
        except Exception as e:
            return jsonify({"error": str(e)}), 500

    # ── Gantry jog routes ────────────────────────────────────────────────────
    @app.route("/api/gantry/jog/start", methods=["POST"])
    def api_gantry_jog_start():
        """Start (or heartbeat-refresh) continuous jog. Background thread
        publishes velocity at 20 Hz; the frontend re-calls this periodically
        while a direction is held, and jog_loop() auto-stops if this refresh
        stops arriving (closed tab, dropped connection, etc.)."""
        data  = request.get_json(silent=True) or {}
        dx    = float(data.get("dx", 0))
        dy    = float(data.get("dy", 0))
        speed = float(data.get("speed", 50))
        with state._jog_lock:
            state._jog["dx"]           = max(-1.0, min(1.0, dx))
            state._jog["dy"]           = max(-1.0, min(1.0, dy))
            state._jog["speed"]        = max(5.0, min(100.0, speed))
            state._jog["active"]       = True
            state._jog["last_refresh"] = time.time()
        return jsonify({"ok": True})

    @app.route("/api/gantry/jog/stop", methods=["POST"])
    def api_gantry_jog_stop():
        """Stop jogging and send a zero-velocity command immediately."""
        with state._jog_lock:
            state._jog["active"] = False
        node = ros_client.ros_node
        if node is not None and ros_client.HAS_ROS:
            try:
                node.vel_pub.publish(ros_client.Twist())
            except Exception:
                pass
        return jsonify({"ok": True})

    @app.route("/api/gantry/step", methods=["POST"])
    def api_gantry_step():
        """Send a direct point-to-point step command to the stepper driver."""
        data     = request.get_json(silent=True) or {}
        steps_a  = int(data.get("steps_a", 0))
        steps_b  = int(data.get("steps_b", 0))
        speed    = float(data.get("speed", 50))
        node = ros_client.ros_node
        if node is None:
            return jsonify({"ok": False, "msg": "ROS not connected"}), 503
        try:
            msg   = ros_client.RosPoint()
            msg.x = float(steps_a)
            msg.y = float(steps_b)
            msg.z = float(speed)
            node.cmd_pub.publish(msg)
            return jsonify({"ok": True})
        except Exception as e:
            return jsonify({"ok": False, "msg": str(e)}), 500

    @app.route("/api/gantry/goto", methods=["POST"])
    def api_gantry_goto():
        data = request.get_json(silent=True) or {}
        sq   = data.get("square", "")
        if sq:
            x, y = state.square_to_mm(sq)
            if x is None:
                return jsonify({"ok": False, "msg": "Invalid square"}), 400
        else:
            x = float(data.get("x", 0))
            y = float(data.get("y", 0))
        speed = float(data.get("speed", 40.0))
        # Attempt physical move via action server
        moved = False
        move_msg = "ROS not connected"
        node = ros_client.ros_node
        if node is not None:
            r = node.send_goto(x, y, speed)
            moved    = r["ok"]
            move_msg = r.get("msg", "")
        # Always update local display state
        with state._lock:
            state._state["gantry_x"] = x
            state._state["gantry_y"] = y
        return jsonify({"ok": True, "x": x, "y": y, "moved": moved, "msg": move_msg})

    # ── Board calibration routes ────────────────────────────────────────────
    @app.route("/api/gantry/calibration", methods=["GET"])
    def api_calib_get():
        with state._lock:
            return jsonify({
                "applied": state._state["calib_applied"],
                "a1_x":    state._state["calib_a1_x"],
                "a1_y":    state._state["calib_a1_y"],
                "h8_x":    state._state["calib_h8_x"],
                "h8_y":    state._state["calib_h8_y"],
                "sq_x":    state._state["calib_sq_x"],
                "sq_y":    state._state["calib_sq_y"],
            })

    @app.route("/api/gantry/calibration/save_a1", methods=["POST"])
    def api_calib_save_a1():
        with state._lock:
            x = state._state["gantry_x"]
            y = state._state["gantry_y"]
            state._state["calib_a1_x"] = x
            state._state["calib_a1_y"] = y
            state._state["calib_applied"] = False  # require re-apply
        return jsonify({"ok": True, "x": x, "y": y})

    @app.route("/api/gantry/calibration/save_h8", methods=["POST"])
    def api_calib_save_h8():
        with state._lock:
            x = state._state["gantry_x"]
            y = state._state["gantry_y"]
            state._state["calib_h8_x"] = x
            state._state["calib_h8_y"] = y
            state._state["calib_applied"] = False
        return jsonify({"ok": True, "x": x, "y": y})

    @app.route("/api/gantry/calibration/apply", methods=["POST"])
    def api_calib_apply():
        with state._lock:
            a1x = state._state["calib_a1_x"]
            a1y = state._state["calib_a1_y"]
            h8x = state._state["calib_h8_x"]
            h8y = state._state["calib_h8_y"]
        if None in (a1x, a1y, h8x, h8y):
            return jsonify({"ok": False, "msg": "Save both a1 and h8 first"}), 400
        sq_x = (h8x - a1x) / 7.0
        sq_y = (h8y - a1y) / 7.0
        if sq_x < 5.0 or sq_y < 5.0:
            return jsonify({"ok": False,
                            "msg": f"Square size too small ({sq_x:.1f}×{sq_y:.1f}mm). "
                                   "Check that a1 and h8 are the correct corners."}), 400
        with state._lock:
            state._state["calib_sq_x"]    = sq_x
            state._state["calib_sq_y"]    = sq_y
            state._state["calib_applied"] = True
        # Push to motion_planner_node — the node that actually drives the gantry
        # during real games — so calibrating here isn't just a local debug-tool value.
        pushed = False
        node = ros_client.ros_node
        if node is not None and ros_client.HAS_RCL_PARAMS:
            node.request_motion_planner_params(a1x, a1y, (sq_x + sq_y) / 2.0)
            pushed = True
        try:
            state._CALIB_FILE.write_text(state.json.dumps({
                "a1_x": a1x, "a1_y": a1y,
                "h8_x": h8x, "h8_y": h8y,
                "sq_x": sq_x, "sq_y": sq_y,
                "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            }, indent=2))
        except Exception as e:
            return jsonify({"ok": True, "sq_x": sq_x, "sq_y": sq_y,
                            "a1_x": a1x, "a1_y": a1y, "h8_x": h8x, "h8_y": h8y,
                            "pushed_to_motion_planner": pushed,
                            "warn": f"Saved in memory; file write failed: {e}"})
        return jsonify({"ok": True, "sq_x": sq_x, "sq_y": sq_y,
                        "a1_x": a1x, "a1_y": a1y, "h8_x": h8x, "h8_y": h8y,
                        "pushed_to_motion_planner": pushed})

    @app.route("/api/gantry/calibration/clear", methods=["POST"])
    def api_calib_clear():
        with state._lock:
            for k in ("calib_a1_x","calib_a1_y","calib_h8_x","calib_h8_y",
                      "calib_sq_x","calib_sq_y"):
                state._state[k] = None
            state._state["calib_applied"] = False
        try:
            state._CALIB_FILE.unlink(missing_ok=True)
        except Exception:
            pass
        return jsonify({"ok": True})

    @app.route("/api/gantry/position")
    def api_gantry_pos():
        with state._lock:
            return jsonify({"x": state._state["gantry_x"],
                            "y": state._state["gantry_y"],
                            "homed": state._state["gantry_homed"]})

    # ── Test routes ──────────────────────────────────────────────────────────
    @app.route("/api/tests/catalogue")
    def api_tests_catalogue():
        return jsonify(state.TEST_CATALOGUE)

    @app.route("/api/tests/run", methods=["POST"])
    def api_tests_run():
        data = request.get_json(silent=True) or {}
        cat  = data.get("category", "")
        sub  = data.get("subtest", "")
        if not cat or not sub:
            return jsonify({"ok": False, "msg": "Need category + subtest"}), 400
        node = ros_client.ros_node
        if node is None:
            return jsonify({"ok": False, "msg": "ROS not connected"}), 503
        ok = node.start_test(cat, sub)
        return jsonify({"ok": ok,
                        "msg": "started" if ok else
                               "already running, or test_runner_node unavailable"})

    @app.route("/api/tests/stop", methods=["POST"])
    def api_tests_stop():
        node = ros_client.ros_node
        if node is not None:
            node.stop_test()
        return jsonify({"ok": True})

    @app.route("/api/tests/stream")
    def api_tests_stream():
        node = ros_client.ros_node
        if node is None:
            return Response('data: {"type":"error","line":"ROS not connected"}\n\n',
                            mimetype="text/event-stream")
        return Response(node.stream_test_sse(),
                        mimetype="text/event-stream",
                        headers={"Cache-Control": "no-cache",
                                 "X-Accel-Buffering": "no"})

    @app.route("/api/tests/status")
    def api_tests_status():
        node = ros_client.ros_node
        if node is None:
            return jsonify({"running": False, "last_run": None})
        return jsonify({"running": node._test_running,
                        "last_run": node._test_last_run})
