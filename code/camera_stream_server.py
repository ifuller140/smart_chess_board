#!/usr/bin/env python3
"""
Pi Camera MJPEG Stream Server.

Serves a live MJPEG stream from the Pi Camera over HTTP so it can be
viewed in any browser on the same network.

Usage (on Pi):
  python3 camera_stream_server.py

Then on your Mac open:
  http://smart-chess-pi:8080/stream
  or
  http://<pi-ip>:8080/stream

Dependencies: picamera2 (or OpenCV fallback for USB cameras)
Install: sudo apt install python3-picamera2 libcamera-ipa
"""

import io
import time
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

# Try picamera2 first (Pi Camera CSI), fall back to OpenCV
try:
    from picamera2 import Picamera2
    USE_PICAM = True
except ImportError:
    USE_PICAM = False

import cv2

PORT = 8080
FRAME_RATE = 10

# Shared frame buffer
_frame_lock = threading.Lock()
_frame_jpeg = b''


def capture_loop_picamera2():
    """Capture frames from Pi Camera using picamera2."""
    global _frame_jpeg
    cam = Picamera2()
    config = cam.create_video_configuration(
        main={'size': (1280, 720), 'format': 'RGB888'})
    cam.configure(config)
    cam.start()
    time.sleep(1.0)  # AWB settle
    print(f'picamera2 started — streaming at http://0.0.0.0:{PORT}/stream')
    while True:
        frame_rgb = cam.capture_array('main')
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with _frame_lock:
                _frame_jpeg = buf.tobytes()
        time.sleep(1.0 / FRAME_RATE)


def capture_loop_opencv():
    """Capture frames from USB/V4L2 camera using OpenCV."""
    global _frame_jpeg
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    if not cap.isOpened():
        print('ERROR: Cannot open camera')
        return
    print(f'OpenCV camera started — streaming at http://0.0.0.0:{PORT}/stream')
    while True:
        ret, frame = cap.read()
        if ret:
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with _frame_lock:
                    _frame_jpeg = buf.tobytes()
        time.sleep(1.0 / FRAME_RATE)


class MJPEGHandler(BaseHTTPRequestHandler):

    def log_message(self, *args):
        pass  # Suppress request logs

    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            try:
                while True:
                    with _frame_lock:
                        jpeg = _frame_jpeg
                    if jpeg:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                        self.wfile.write(jpeg)
                        self.wfile.write(b'\r\n')
                    time.sleep(1.0 / FRAME_RATE)
            except (BrokenPipeError, ConnectionResetError):
                pass

        elif self.path == '/capture':
            # Capture single JPEG on demand
            with _frame_lock:
                jpeg = _frame_jpeg
            if jpeg:
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpeg)))
                self.end_headers()
                self.wfile.write(jpeg)
            else:
                self.send_response(503)
                self.end_headers()

        elif self.path == '/':
            # Mini HTML viewer page
            html = f'''<!DOCTYPE html>
<html>
<head>
  <title>Pi Chess Camera</title>
  <style>
    body {{ background: #111; color: #eee; font-family: sans-serif; text-align: center; margin: 0; }}
    h1   {{ margin: 16px 0 8px; font-size: 1.5em; letter-spacing: 2px; }}
    img  {{ max-width: 100%; border: 2px solid #444; display: block; margin: 0 auto; }}
    .info {{ font-size: 0.8em; color: #888; margin: 8px; }}
  </style>
</head>
<body>
  <h1>📷 Smart Chess Board — Camera</h1>
  <img src="/stream" alt="Live camera feed">
  <p class="info">Live MJPEG stream · Pi Camera Module v2 · {FRAME_RATE}fps</p>
  <p class="info"><a href="/capture" style="color:#aaf">Download still</a></p>
</body>
</html>'''
            body = html.encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        else:
            self.send_response(404)
            self.end_headers()


if __name__ == '__main__':
    # Start capture thread
    if USE_PICAM:
        t = threading.Thread(target=capture_loop_picamera2, daemon=True)
    else:
        t = threading.Thread(target=capture_loop_opencv, daemon=True)
    t.start()

    # Wait for first frame
    print('Waiting for first frame...')
    for _ in range(50):
        with _frame_lock:
            if _frame_jpeg:
                break
        time.sleep(0.2)

    print(f'Open in browser: http://smart-chess-pi:{PORT}/')
    print(f'Stream URL:      http://smart-chess-pi:{PORT}/stream')
    print(f'Still capture:   http://smart-chess-pi:{PORT}/capture')
    print(f'Ctrl+C to stop')

    server = HTTPServer(('0.0.0.0', PORT), MJPEGHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nServer stopped.')
