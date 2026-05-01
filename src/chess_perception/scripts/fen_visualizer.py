#!/usr/bin/env python3
"""
FEN Live Board Visualizer — Smart Chess Board
==============================================
Subscribes to /perception/board_state and serves a live chess board at http://<pi-ip>:5000

Modes:
  Normal (with ROS): receives FEN from the running perception stack
  --no-ros          : inject FEN manually via --fen flag or POST /api/fen

Usage:
  # With full perception stack running:
  python3 src/chess_perception/scripts/fen_visualizer.py

  # Offline / no ROS (test rendering only):
  python3 src/chess_perception/scripts/fen_visualizer.py --no-ros
  python3 src/chess_perception/scripts/fen_visualizer.py --no-ros --fen "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"

  # Then open in browser on Mac:
  http://localhost:5000                    # if running locally on Mac
  http://<raspberry-pi-ip>:5000           # if running on Pi (find IP with: hostname -I)
"""

import argparse
import threading
import time
import sys
import os

# ── Optional ROS imports ──────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

# ── Flask ─────────────────────────────────────────────────────────────────────
try:
    from flask import Flask, jsonify, render_template_string, request as flask_request
except ImportError:
    print("ERROR: Flask not installed. Run:  pip3 install flask")
    sys.exit(1)

# ── python-chess (optional, for FEN validation) ───────────────────────────────
try:
    import chess
    CHESS_AVAILABLE = True
except ImportError:
    CHESS_AVAILABLE = False
    print("WARN: python-chess not installed. FEN validation disabled.")
    print("      Run:  pip3 install python-chess")


# ─────────────────────────────────────────────────────────────────────────────
# Shared state (written by ROS thread, read by Flask thread)
# ─────────────────────────────────────────────────────────────────────────────
_state = {
    "fen": "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
    "reference_fen": "",
    "last_updated": 0.0,
    "frame_count": 0,
    "board_detected": False,
    "error": "",
}
_lock = threading.Lock()


# ─────────────────────────────────────────────────────────────────────────────
# ROS Subscriber Node
# ─────────────────────────────────────────────────────────────────────────────
class FenSubscriberNode(Node):
    """
    Subscribes to /perception/board_state (BoardState custom msg).
    Also subscribes to /perception/board_debug to track if board is detected.

    If the custom message isn't built yet, falls back to /perception/board_state_fen
    as a plain String topic (easier to test with ros2 topic pub).
    """

    def __init__(self):
        super().__init__("fen_visualizer")

        # Try to import the custom BoardState message
        self._using_custom_msg = False
        try:
            # Try chess_perception package first
            try:
                from chess_perception.msg import BoardState
            except ImportError:
                from chess_interfaces.msg import BoardState

            self.create_subscription(
                BoardState,
                "/perception/board_state",
                self._on_board_state,
                10,
            )
            self._using_custom_msg = True
            self.get_logger().info("Subscribed to /perception/board_state (BoardState)")
        except ImportError:
            # Fallback: subscribe to a plain String topic
            self.create_subscription(
                String,
                "/perception/board_state_fen",
                self._on_fen_string,
                10,
            )
            self.get_logger().warn(
                "BoardState msg not found — falling back to /perception/board_state_fen (String). "
                "Publish FEN with: ros2 topic pub /perception/board_state_fen std_msgs/String '{data: \"<fen>\"}'"
            )

        self.get_logger().info(
            "FEN Visualizer ROS node ready. "
            "Open browser at http://<this-machine-ip>:5000"
        )

    def _on_board_state(self, msg):
        """Callback for custom BoardState message."""
        fen = getattr(msg, "fen", "")
        if not fen:
            return
        self._update_fen(fen, board_detected=True)

    def _on_fen_string(self, msg):
        """Callback for plain String FEN topic (fallback)."""
        self._update_fen(msg.data.strip(), board_detected=True)

    def _update_fen(self, fen: str, board_detected: bool):
        validated = self._validate_fen(fen)
        with _lock:
            _state["fen"] = validated or fen
            _state["board_detected"] = board_detected
            _state["last_updated"] = time.time()
            _state["frame_count"] += 1
            _state["error"] = "" if validated else f"Invalid FEN: {fen[:60]}"

    @staticmethod
    def _validate_fen(fen: str) -> str:
        """Return fen if valid (python-chess), else empty string."""
        if not CHESS_AVAILABLE:
            return fen
        try:
            chess.Board(fen)
            return fen
        except Exception:
            return ""


def _ros_spin(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# ─────────────────────────────────────────────────────────────────────────────
# HTML Page
# ─────────────────────────────────────────────────────────────────────────────
HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>♟ Chess Perception — Live Board</title>
  <link rel="stylesheet"
    href="https://unpkg.com/@chrisoakman/chessboardjs@1.0.0/dist/chessboard-1.0.0.min.css">
  <script src="https://code.jquery.com/jquery-3.5.1.min.js"></script>
  <script src="https://unpkg.com/@chrisoakman/chessboardjs@1.0.0/dist/chessboard-1.0.0.min.js"></script>
  <style>
    :root {
      --bg:       #0d1117;
      --panel:    #161b22;
      --border:   #30363d;
      --green:    #3fb950;
      --red:      #f85149;
      --yellow:   #d29922;
      --blue:     #58a6ff;
      --text:     #c9d1d9;
      --subtext:  #8b949e;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      background: var(--bg); color: var(--text);
      font-family: 'Segoe UI', system-ui, sans-serif;
      padding: 24px; display: flex; gap: 24px; flex-wrap: wrap;
    }
    h1 { color: var(--blue); font-size: 1.4em; margin-bottom: 16px; }
    .left  { flex: 0 0 auto; }
    .right { flex: 1 1 340px; min-width: 280px; }

    #board { width: 480px; }

    .card {
      background: var(--panel); border: 1px solid var(--border);
      border-radius: 8px; padding: 16px; margin-bottom: 14px;
    }
    .card h2 { font-size: 0.85em; text-transform: uppercase;
               letter-spacing: 0.05em; color: var(--subtext); margin-bottom: 10px; }

    .pill {
      display: inline-block; padding: 2px 10px; border-radius: 20px;
      font-size: 0.8em; font-weight: 600;
    }
    .ok   { background: #1a3326; color: var(--green); }
    .warn { background: #3d2a00; color: var(--yellow); }
    .err  { background: #3d1010; color: var(--red); }

    #fen-text {
      font-family: monospace; font-size: 0.78em;
      color: var(--green); word-break: break-all;
      background: #0d1117; border: 1px solid var(--border);
      border-radius: 4px; padding: 8px; margin-top: 8px;
    }

    .stat-row { display: flex; justify-content: space-between;
                padding: 4px 0; border-bottom: 1px solid var(--border);
                font-size: 0.85em; }
    .stat-row:last-child { border-bottom: none; }
    .stat-label { color: var(--subtext); }
    .stat-val   { font-family: monospace; color: var(--text); }

    #diff-list { font-family: monospace; font-size: 0.82em;
                 max-height: 180px; overflow-y: auto; }
    .diff-item  { padding: 3px 6px; border-radius: 4px;
                  margin: 2px 0; background: #2a1515; color: var(--red); }
    .no-diff    { color: var(--green); font-size: 0.85em; }

    button {
      background: var(--blue); color: #000; border: none;
      padding: 7px 16px; border-radius: 6px; cursor: pointer;
      font-weight: 600; font-size: 0.85em; margin-right: 6px;
    }
    button:hover { opacity: 0.85; }
    button.secondary { background: var(--border); color: var(--text); }

    #inject-fen {
      width: 100%; font-family: monospace; font-size: 0.8em;
      background: var(--bg); color: var(--text);
      border: 1px solid var(--border); border-radius: 4px;
      padding: 6px; margin-top: 6px;
    }

    #status-dot {
      display: inline-block; width: 10px; height: 10px;
      border-radius: 50%; margin-right: 6px;
      background: var(--yellow);
    }
    #status-dot.live { background: var(--green); animation: pulse 2s infinite; }
    #status-dot.stale { background: var(--red); }
    @keyframes pulse {
      0%,100% { opacity:1; } 50% { opacity:0.4; }
    }
  </style>
</head>
<body>
<div class="left">
  <h1>♟ Chess Perception — Live Board</h1>
  <div id="board"></div>
</div>

<div class="right">

  <!-- Status -->
  <div class="card">
    <h2>Status</h2>
    <div class="stat-row">
      <span class="stat-label">Connection</span>
      <span class="stat-val"><span id="status-dot"></span><span id="status-text">Connecting...</span></span>
    </div>
    <div class="stat-row">
      <span class="stat-label">Frames received</span>
      <span class="stat-val" id="frame-count">0</span>
    </div>
    <div class="stat-row">
      <span class="stat-label">Last update</span>
      <span class="stat-val"><span id="age">—</span>s ago</span>
    </div>
    <div class="stat-row">
      <span class="stat-label">FEN valid</span>
      <span class="stat-val" id="fen-valid">—</span>
    </div>
  </div>

  <!-- FEN -->
  <div class="card">
    <h2>Current FEN</h2>
    <div id="fen-text">Loading...</div>
  </div>

  <!-- Diff vs Reference -->
  <div class="card">
    <h2>Diff vs Reference</h2>
    <div style="margin-bottom:8px;">
      <button onclick="setReference()">📌 Set Current as Reference</button>
      <button class="secondary" onclick="setStartingPos()">Reset to Start</button>
    </div>
    <div id="ref-label" style="font-size:0.8em; color:var(--subtext); margin-bottom:8px;">
      Reference: starting position
    </div>
    <div id="diff-list"><span class="no-diff">Set a reference to compare</span></div>
  </div>

  <!-- Manual FEN Injection -->
  <div class="card">
    <h2>Inject FEN (offline test)</h2>
    <input id="inject-fen" type="text"
           placeholder="paste FEN here..."
           value="rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1">
    <div style="margin-top:8px;">
      <button onclick="injectFen()">Send FEN</button>
    </div>
  </div>

</div>

<script>
var board = Chessboard('board', {
  position: 'start',
  showNotation: true,
  pieceTheme: 'https://lichess1.org/assets/piece/cburnett/{piece}.svg'
});

var referenceFen  = 'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1';
var lastFrameCount = 0;

// ── FEN parsing helpers ──────────────────────────────────────────────────────
function fenToSquares(fen) {
  var placement = fen.split(' ')[0];
  var rows = placement.split('/');
  var squares = {};
  var files = 'abcdefgh';
  rows.forEach(function(row, rowIdx) {
    var fileIdx = 0;
    for (var i = 0; i < row.length; i++) {
      var ch = row[i];
      if (isNaN(ch)) {
        squares[files[fileIdx] + (8 - rowIdx)] = ch;
        fileIdx++;
      } else {
        fileIdx += parseInt(ch);
      }
    }
  });
  return squares;
}

function diffFens(refFen, curFen) {
  var ref = fenToSquares(refFen);
  var cur = fenToSquares(curFen);
  var allSq = new Set(Object.keys(ref).concat(Object.keys(cur)));
  var diffs = [];
  allSq.forEach(function(sq) {
    var a = ref[sq] || '(empty)';
    var b = cur[sq] || '(empty)';
    if (a !== b) diffs.push({ sq: sq, from: a, to: b });
  });
  return diffs;
}

// ── Update loop ──────────────────────────────────────────────────────────────
function update() {
  fetch('/api/fen')
    .then(function(r) { return r.json(); })
    .then(function(d) {
      // Board position
      try {
        board.position(d.fen.split(' ')[0], false);
      } catch(e) {}

      // FEN text
      document.getElementById('fen-text').textContent = d.fen;

      // Frame count / status
      document.getElementById('frame-count').textContent = d.frame_count;
      var dot  = document.getElementById('status-dot');
      var stxt = document.getElementById('status-text');
      var age  = d.last_updated > 0 ? ((Date.now() / 1000) - d.last_updated) : 9999;
      var ageEl = document.getElementById('age');
      ageEl.textContent = age < 9999 ? age.toFixed(1) : 'never';

      if (age < 3) {
        dot.className = 'live'; stxt.textContent = 'Live ✓';
      } else if (age < 10) {
        dot.className = 'stale'; stxt.textContent = 'Stale (' + age.toFixed(0) + 's)';
      } else {
        dot.className = 'stale'; stxt.textContent = 'No data';
      }

      // FEN validity
      var validEl = document.getElementById('fen-valid');
      if (d.error) {
        validEl.innerHTML = '<span class="pill err">Invalid</span>';
      } else {
        validEl.innerHTML = '<span class="pill ok">Valid</span>';
      }

      // Diff
      var diffs = diffFens(referenceFen, d.fen);
      var diffEl = document.getElementById('diff-list');
      if (d.frame_count === 0) {
        diffEl.innerHTML = '<span class="no-diff">Waiting for data...</span>';
      } else if (diffs.length === 0) {
        diffEl.innerHTML = '<span class="no-diff">✓ Matches reference exactly</span>';
      } else {
        var html = '';
        diffs.forEach(function(diff) {
          html += '<div class="diff-item">' +
                  diff.sq.toUpperCase() + ': ' +
                  pieceLabel(diff.from) + ' → ' + pieceLabel(diff.to) +
                  '</div>';
        });
        diffEl.innerHTML = html;
      }
    })
    .catch(function() {
      document.getElementById('status-dot').className = 'stale';
      document.getElementById('status-text').textContent = 'Server unreachable';
    });
}

function pieceLabel(p) {
  var map = {
    'K':'♔ K','Q':'♕ Q','R':'♖ R','B':'♗ B','N':'♘ N','P':'♙ P',
    'k':'♚ k','q':'♛ q','r':'♜ r','b':'♝ b','n':'♞ n','p':'♟ p',
    '(empty)':'□ empty'
  };
  return map[p] || p;
}

// ── Controls ─────────────────────────────────────────────────────────────────
function setReference() {
  fetch('/api/fen').then(function(r) { return r.json(); }).then(function(d) {
    referenceFen = d.fen;
    document.getElementById('ref-label').textContent =
      'Reference: ' + d.fen.split(' ')[0];
  });
}

function setStartingPos() {
  referenceFen = 'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1';
  document.getElementById('ref-label').textContent = 'Reference: starting position';
}

function injectFen() {
  var fen = document.getElementById('inject-fen').value.trim();
  fetch('/api/fen', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ fen: fen })
  });
}

setInterval(update, 1000);
update();
</script>
</body>
</html>
"""


# ─────────────────────────────────────────────────────────────────────────────
# Flask App
# ─────────────────────────────────────────────────────────────────────────────
app = Flask(__name__)
app.logger.setLevel("ERROR")  # suppress Flask request logs


@app.route("/")
def index():
    return render_template_string(HTML)


@app.route("/api/fen", methods=["GET"])
def api_fen_get():
    with _lock:
        return jsonify({
            "fen":          _state["fen"],
            "frame_count":  _state["frame_count"],
            "last_updated": _state["last_updated"],
            "board_detected": _state["board_detected"],
            "error":        _state["error"],
        })


@app.route("/api/fen", methods=["POST"])
def api_fen_post():
    """Inject a FEN string manually — useful for offline testing."""
    data = flask_request.get_json(silent=True) or {}
    fen = data.get("fen", "").strip()
    if not fen:
        return jsonify({"ok": False, "error": "No FEN provided"}), 400
    with _lock:
        _state["fen"]          = fen
        _state["last_updated"] = time.time()
        _state["frame_count"] += 1
        _state["error"]        = ""
    return jsonify({"ok": True, "fen": fen})


@app.route("/api/reset", methods=["POST"])
def api_reset():
    """Reset frame counter."""
    with _lock:
        _state["frame_count"]   = 0
        _state["last_updated"]  = 0.0
        _state["board_detected"] = False
    return jsonify({"ok": True})


@app.route("/healthz")
def healthz():
    return jsonify({"ok": True, "ros_available": ROS_AVAILABLE})


# ─────────────────────────────────────────────────────────────────────────────
# Entry Point
# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Live FEN board visualizer for chess perception debugging",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # With ROS perception stack running:
  python3 src/chess_perception/scripts/fen_visualizer.py

  # Offline — test board rendering without ROS:
  python3 src/chess_perception/scripts/fen_visualizer.py --no-ros

  # Offline — start with a specific FEN:
  python3 src/chess_perception/scripts/fen_visualizer.py --no-ros \\
      --fen "rnb2bnr/8/ppp2ppp/pppPPppp/2PpppP1/PPPpppPP/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        """
    )
    parser.add_argument("--port", type=int, default=5000,
                        help="HTTP port (default: 5000)")
    parser.add_argument("--host", default="0.0.0.0",
                        help="Bind address (default: 0.0.0.0 = all interfaces)")
    parser.add_argument("--no-ros", action="store_true",
                        help="Disable ROS subscriber (offline/test mode)")
    parser.add_argument("--fen", default=None,
                        help="Initial FEN string to display")
    args = parser.parse_args()

    # Pre-load a FEN if given
    if args.fen:
        with _lock:
            _state["fen"] = args.fen
            _state["last_updated"] = time.time()
            _state["frame_count"] = 1

    # Start ROS subscriber thread
    if not args.no_ros:
        if not ROS_AVAILABLE:
            print("ERROR: rclpy not found. Run with --no-ros for offline mode.")
            sys.exit(1)
        rclpy.init()
        ros_node = FenSubscriberNode()
        ros_thread = threading.Thread(target=_ros_spin, args=(ros_node,), daemon=True)
        ros_thread.start()
        print("✓ ROS subscriber started")
    else:
        print("⚠  Running in offline mode (--no-ros). Inject FEN via browser or:")
        print("   curl -X POST http://localhost:{}/api/fen \\".format(args.port))
        print('        -H "Content-Type: application/json" \\')
        print('        -d \'{"fen": "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"}\'')

    print()
    print("=" * 60)
    print("  Chess Perception Live Visualizer")
    print("=" * 60)
    print(f"  Local URL  : http://localhost:{args.port}")
    print(f"  Network URL: http://<this-machine-ip>:{args.port}")
    print(f"  Find Pi IP : hostname -I    (run on Pi)")
    print()
    print(f"  Topics subscribed:")
    print(f"    /perception/board_state   (BoardState)")
    print(f"    /perception/board_state_fen (String fallback)")
    print("=" * 60)
    print()

    # Start Flask (blocking)
    app.run(host=args.host, port=args.port, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
