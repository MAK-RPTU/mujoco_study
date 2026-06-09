"""
web_viewer.py — Headless MuJoCo viewer that streams over an HTTP port.

Use this on the remote (headless) Linux box. It renders the sim offscreen
with EGL (GPU) and serves a live MJPEG stream you open in any browser —
including from your Windows machine. No client install needed: it's just
an <img> tag pointed at the stream.

    python viewer/web_viewer.py --model models/cube_contact.xml --port 8910

Then on Windows, either:
  (A) SSH tunnel (recommended, secure):
        ssh -L 8910:localhost:8910 <user>@<box-ip>
      and open  http://localhost:8910  in your browser.
  (B) Direct: open the box's firewall/security-group for the port and
      open  http://<box-ip>:8910  (see scripts/README_PORT.md).

It also accepts a tiny control: drag-less keyboard via URL — POST /reset
re-zeros the sim. Mostly you just watch.

Only stdlib + mujoco + numpy + pillow are required.
"""
import os
# EGL must be selected before importing mujoco on a headless box.
if "MUJOCO_GL" not in os.environ and not os.environ.get("DISPLAY"):
    os.environ["MUJOCO_GL"] = "egl"

import argparse
import io
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import mujoco
from PIL import Image


class SimState:
    """Runs physics AND rendering on one background thread, publishing the
    latest JPEG frame for HTTP handlers to read.

    Why one thread: MuJoCo's GL/EGL render context can only be 'current' on
    the thread that created it (eglMakeCurrent is per-thread). So we create
    the renderer and call render() only here, and hand finished JPEG bytes
    to the (multi-threaded) HTTP handlers via a lock. The handlers never
    touch GL.
    """

    def __init__(self, model_path, width, height, fps, realtime=True):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.width = width
        self.height = height
        self.fps = fps
        self.realtime = realtime
        self.lock = threading.Lock()
        self._latest_jpeg = None
        self._stop = False
        self._reset_requested = False

    def reset(self):
        self._reset_requested = True

    def _physics_loop(self):
        # Create the renderer HERE so its EGL context belongs to this thread.
        renderer = mujoco.Renderer(self.model, self.height, self.width)
        dt = self.model.opt.timestep
        # Render every `render_every` physics steps to target ~fps.
        render_every = max(1, int(round((1.0 / self.fps) / dt)))
        step = 0
        try:
            while not self._stop:
                if self._reset_requested:
                    mujoco.mj_resetData(self.model, self.data)
                    self._reset_requested = False
                mujoco.mj_step(self.model, self.data)
                if step % render_every == 0:
                    renderer.update_scene(self.data)
                    img = renderer.render()
                    buf = io.BytesIO()
                    Image.fromarray(img).save(buf, format="JPEG", quality=80)
                    with self.lock:
                        self._latest_jpeg = buf.getvalue()
                step += 1
                if self.realtime:
                    time.sleep(dt)
        finally:
            renderer.close()

    def start(self):
        self._thread = threading.Thread(target=self._physics_loop, daemon=True)
        self._thread.start()
        # Wait for the first frame so the stream never serves None.
        for _ in range(200):
            if self._latest_jpeg is not None:
                break
            time.sleep(0.02)

    def jpeg(self):
        with self.lock:
            return self._latest_jpeg


INDEX_HTML = b"""<!doctype html><html><head><title>MuJoCo remote viewer</title>
<style>body{background:#111;color:#ddd;font-family:monospace;text-align:center}
img{max-width:95vw;border:1px solid #444;margin-top:1em}
button{font-family:monospace;padding:.5em 1em;margin:.5em}</style></head>
<body><h3>MuJoCo headless stream</h3>
<img src="/stream"/><br>
<button onclick="fetch('/reset',{method:'POST'})">reset sim</button>
<p>Streaming offscreen render over MJPEG. Close the tab to stop watching;
the sim keeps running on the server.</p></body></html>"""


def make_handler(state, fps):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):  # quiet
            pass

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(INDEX_HTML)
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                period = 1.0 / fps
                try:
                    while True:
                        frame = state.jpeg()
                        if frame is None:
                            time.sleep(period)
                            continue
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            f"Content-Length: {len(frame)}\r\n\r\n".encode())
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                        time.sleep(period)
                except (BrokenPipeError, ConnectionResetError):
                    pass  # browser tab closed
            else:
                self.send_error(404)

        def do_POST(self):
            if self.path == "/reset":
                state.reset()
                self.send_response(204)
                self.end_headers()
            else:
                self.send_error(404)

    return Handler


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="path to an MJCF .xml")
    ap.add_argument("--port", type=int, default=8910)
    ap.add_argument("--host", default="0.0.0.0",
                    help="0.0.0.0 to allow remote/tunnel; 127.0.0.1 local only")
    ap.add_argument("--width", type=int, default=720)
    ap.add_argument("--height", type=int, default=540)
    ap.add_argument("--fps", type=float, default=30.0)
    args = ap.parse_args()

    state = SimState(args.model, args.width, args.height, args.fps)
    state.start()

    httpd = ThreadingHTTPServer((args.host, args.port),
                                make_handler(state, args.fps))
    print(f"[web_viewer] serving {args.model}")
    print(f"[web_viewer] open  http://localhost:{args.port}  "
          f"(SSH-tunnel) or http://<box-ip>:{args.port} (direct)")
    print("[web_viewer] Ctrl-C to stop.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[web_viewer] stopping.")


if __name__ == "__main__":
    main()
