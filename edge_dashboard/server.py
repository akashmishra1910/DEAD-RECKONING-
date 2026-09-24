"""
edge_dashboard/server.py
========================
Lightweight, zero-dependency local HTTP server for Edge Engine Test Dashboard.
"""

import os
import json
import socket
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

from .controller import EdgeTestController


class EdgeDashboardHandler(BaseHTTPRequestHandler):
    """Handles REST API and static dashboard UI requests."""

    controller: Optional[EdgeTestController] = None
    static_dir: str = os.path.join(os.path.dirname(__file__), "static")

    def _send_json(self, status_code: int, data: dict):
        body = json.dumps(data).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            index_path = os.path.join(self.static_dir, "index.html")
            if os.path.exists(index_path):
                with open(index_path, "rb") as f:
                    content = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
            else:
                self.send_error(404, "index.html not found")
        elif self.path == "/api/state":
            if self.controller:
                snapshot = self.controller.get_snapshot()
                self._send_json(200, snapshot)
            else:
                self._send_json(500, {"error": "Controller not initialized"})
        else:
            self.send_error(404, "Not Found")

    def do_POST(self):
        content_len = int(self.headers.get("Content-Length", 0))
        body_bytes = self.rfile.read(content_len) if content_len > 0 else b"{}"
        try:
            payload = json.loads(body_bytes.decode("utf-8")) if body_bytes else {}
        except Exception:
            payload = {}

        if not self.controller:
            self._send_json(500, {"error": "Controller not available"})
            return

        if self.path == "/api/control":
            action = payload.get("action", "")
            val = payload.get("value")

            if action == "start":
                self.controller.start_replay()
            elif action == "pause":
                self.controller.pause_replay()
            elif action == "stop":
                self.controller.stop_replay()
            elif action == "restart":
                self.controller.restart_replay()
            elif action == "reset":
                self.controller.reset_engine()
            elif action == "gnss_denied":
                self.controller.set_gnss_denied(True)
            elif action == "gnss_restore":
                self.controller.set_gnss_denied(False)
            elif action == "rate":
                mult = float(val) if val is not None else 1.0
                self.controller.set_replay_speed(mult)
            else:
                self._send_json(400, {"error": f"Unknown control action: {action}"})
                return

            self._send_json(200, {"status": "ok", "action": action})

        elif self.path == "/api/test":
            scenario_id = int(payload.get("scenario", 1))
            res = self.controller.run_predefined_test(scenario_id)
            self._send_json(200, res)

        elif self.path == "/api/replay":
            action = payload.get("action", "")
            source = payload.get("source", "synthetic")
            if action == "load":
                ok = self.controller.load_replay(source)
                self._send_json(200, {"status": "ok", "loaded": ok})
            else:
                self._send_json(400, {"error": f"Unknown replay action: {action}"})

        else:
            self.send_error(404, "Not Found")

    def log_message(self, format, *args):
        # Suppress routine GET /api/state polling logs from cluttering stdout
        return


def find_available_port(start_port: int = 8080, max_port: int = 8100) -> int:
    """Finds an available TCP port."""
    for p in range(start_port, max_port):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            if s.connect_ex(("127.0.0.1", p)) != 0:
                return p
    return start_port


def run_server(port: Optional[int] = None, open_browser: bool = False, controller: Optional[EdgeTestController] = None):
    """Runs the Edge Engine Test Dashboard HTTP server."""
    actual_port = port if port is not None else find_available_port(8080)
    ctrl = controller if controller is not None else EdgeTestController()

    EdgeDashboardHandler.controller = ctrl

    server = ThreadingHTTPServer(("127.0.0.1", actual_port), EdgeDashboardHandler)
    url = f"http://localhost:{actual_port}"

    print("=" * 65)
    print("ISRO PS 26168 — EDGE ENGINE TEST DASHBOARD")
    print("Platform: SOFTWARE EDGE TEST (Laptop Execution)")
    print("Target Profile: EDGE_FOG (~200 Hz Core Engine)")
    print(f"URL: {url}")
    print("=" * 65)
    print("Press Ctrl+C to shut down.")

    if open_browser:
        try:
            import webbrowser
            webbrowser.open(url)
        except Exception:
            pass

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down Edge Engine Test Dashboard...")
    finally:
        ctrl.shutdown_engine()
        server.server_close()


if __name__ == "__main__":
    run_server()
