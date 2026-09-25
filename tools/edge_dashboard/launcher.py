"""
edge_dashboard/launcher.py
==========================
Thin desktop launcher and orchestrator for the ISRO IDR Edge Engine Dashboard.
Handles:
1. Port detection and dynamic conflict resolution.
2. Backend process spawning and lifecycle tracking.
3. Health check polling (/api/state) with timeout.
4. Clean shutdown and child process termination (no zombies/orphans).
5. Lightweight aerospace-themed startup splash window (Tkinter or headless fallback).
6. Automatic browser launching.
7. Development and debug mode (--dev).
"""

import os
import sys
import time
import json
import socket
import signal
import atexit
import logging
import urllib.request
import urllib.error
import subprocess
import threading
import webbrowser
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, Tuple, Callable

# Ensure repo root is in python path
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

LOG_DIR = os.path.join(REPO_ROOT, "logs")
DEFAULT_LOG_FILE = os.path.join(LOG_DIR, "idr_launcher.log")


@dataclass
class LauncherConfig:
    """Configuration parameters for the desktop launcher."""
    preferred_port: int = 8080
    max_port: int = 8100
    host: str = "127.0.0.1"
    dev_mode: bool = False
    open_browser: bool = True
    reuse_server: bool = True
    timeout_s: float = 12.0
    headless: bool = False
    log_file: str = DEFAULT_LOG_FILE
    browser_opener: Optional[Callable[[str], bool]] = None


def setup_launcher_logger(log_file: str, dev_mode: bool = False) -> logging.Logger:
    """Sets up a dedicated file and console logger for the orchestrator."""
    os.makedirs(os.path.dirname(log_file), exist_ok=True)
    logger = logging.getLogger("IDRLauncher")
    logger.setLevel(logging.DEBUG if dev_mode else logging.INFO)

    # Avoid duplicate handlers on re-entry
    if not logger.handlers:
        fmt = logging.Formatter("[%(asctime)s] [%(levelname)s] [Launcher] %(message)s", datefmt="%H:%M:%S")

        # File handler
        fh = logging.FileHandler(log_file, mode="a", encoding="utf-8")
        fh.setFormatter(fmt)
        fh.setLevel(logging.DEBUG)
        logger.addHandler(fh)

        # Console handler (stdout)
        ch = logging.StreamHandler(sys.stdout)
        ch.setFormatter(fmt)
        ch.setLevel(logging.DEBUG if dev_mode else logging.INFO)
        logger.addHandler(ch)

    return logger


def is_port_bindable(port: int, host: str = "127.0.0.1") -> bool:
    """Checks if a TCP port is currently available to bind."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        if sys.platform != "win32":
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((host, port))
            return True
        except Exception:
            return False


def is_idr_backend_running(port: int, host: str = "127.0.0.1", timeout: float = 0.5) -> bool:
    """Checks if an authentic IDR dashboard backend is already responding on the port."""
    url = f"http://{host}:{port}/api/state"
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "IDRLauncher/1.0"})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            if resp.status == 200:
                data = json.loads(resp.read().decode("utf-8"))
                # Verify key IDR signatures
                return "engine_status" in data and ("EDGE" in data.get("profile", "") or "MOBILE" in data.get("profile", ""))
    except Exception:
        pass
    return False


def select_best_port(config: LauncherConfig, logger: logging.Logger) -> Tuple[int, bool]:
    """
    Selects the best port:
    Returns (port, is_reusing_existing_server).
    """
    # 1. If reuse_server is True, check if preferred port is already running IDR
    if config.reuse_server and is_idr_backend_running(config.preferred_port, config.host):
        logger.info(f"Detected existing healthy IDR dashboard backend on port {config.preferred_port}. Reusing server.")
        return config.preferred_port, True

    # 2. Check if preferred port is available to bind
    if is_port_bindable(config.preferred_port, config.host):
        return config.preferred_port, False

    logger.warning(f"Preferred port {config.preferred_port} is occupied. Scanning for next available port...")

    # 3. Scan up to max_port
    for p in range(config.preferred_port + 1, config.max_port + 1):
        if config.reuse_server and is_idr_backend_running(p, config.host):
            logger.info(f"Found existing healthy IDR dashboard backend on port {p}. Reusing server.")
            return p, True
        if is_port_bindable(p, config.host):
            logger.info(f"Selected open port {p} for IDR dashboard.")
            return p, False

    # Fallback to preferred port if all scan fails
    logger.warning(f"No free port found in range {config.preferred_port}-{config.max_port}. Attempting {config.preferred_port}.")
    return config.preferred_port, False


class IDRLauncher:
    """
    Main desktop orchestrator managing process startup, health polling,
    UI presentation, and graceful termination.
    """

    def __init__(self, config: Optional[LauncherConfig] = None):
        self.config = config or LauncherConfig()
        self.logger = setup_launcher_logger(self.config.log_file, self.config.dev_mode)
        self.backend_process: Optional[subprocess.Popen] = None
        self.actual_port: int = self.config.preferred_port
        self.is_reusing_existing: bool = False
        self.is_running: bool = False
        self._shutdown_lock = threading.Lock()

        # Register cleanup on exit
        atexit.register(self.shutdown)
        self._register_signals()

    def _register_signals(self):
        """Registers OS signal handlers for clean shutdown."""
        try:
            signal.signal(signal.SIGINT, self._signal_handler)
            signal.signal(signal.SIGTERM, self._signal_handler)
        except (ValueError, AttributeError):
            # Not in main thread or signal unsupported
            pass

    def _signal_handler(self, signum, frame):
        self.logger.info(f"Received OS signal {signum}. Initiating graceful shutdown...")
        self.shutdown()
        sys.exit(0)

    def start_backend_process(self, port: int) -> subprocess.Popen:
        """Starts the backend server as a tracked child process."""
        cmd = [
            sys.executable,
            os.path.join(REPO_ROOT, "edge_dashboard.py"),
            "--port", str(port),
            "--no-browser",
        ]
        self.logger.info(f"Starting backend process: {' '.join(cmd)}")

        # Configure environment
        env = os.environ.copy()
        env["PYTHONPATH"] = REPO_ROOT + (os.pathsep + env["PYTHONPATH"] if "PYTHONPATH" in env else "")

        # Launch child process
        proc = subprocess.Popen(
            cmd,
            cwd=REPO_ROOT,
            env=env,
            stdout=subprocess.DEVNULL if not self.config.dev_mode else None,
            stderr=subprocess.DEVNULL if not self.config.dev_mode else None,
            close_fds=True,
        )
        self.backend_process = proc
        self.logger.info(f"Backend process spawned (PID: {proc.pid})")
        return proc

    def poll_health(self, port: int, timeout_s: float, on_progress: Optional[Callable[[str, str], None]] = None) -> Tuple[bool, Optional[Dict[str, Any]]]:
        """
        Polls the IDR backend /api/state endpoint until engine reports RUNNING.
        on_progress callback can report ('step_name', 'status') to UI.
        """
        url = f"http://{self.config.host}:{port}/api/state"
        start_t = time.time()
        self.logger.info(f"Polling backend health at {url} (timeout: {timeout_s}s)...")

        backend_ready = False
        engine_ready = False
        api_healthy = False

        while time.time() - start_t < timeout_s:
            # Check if spawned process died prematurely
            if self.backend_process is not None and self.backend_process.poll() is not None:
                rc = self.backend_process.returncode
                self.logger.error(f"Backend process terminated unexpectedly with exit code {rc}")
                if on_progress:
                    on_progress("backend", "FAIL")
                return False, None

            try:
                req = urllib.request.Request(url, headers={"User-Agent": "IDRLauncher/1.0"})
                with urllib.request.urlopen(req, timeout=0.8) as resp:
                    if resp.status == 200:
                        data = json.loads(resp.read().decode("utf-8"))
                        if not backend_ready:
                            backend_ready = True
                            self.logger.info("Backend HTTP server is responding.")
                            if on_progress:
                                on_progress("backend", "PASS")

                        st = data.get("engine_status", "")
                        if st == "RUNNING":
                            if not engine_ready:
                                engine_ready = True
                                self.logger.info(f"Edge Engine status: RUNNING (Profile: {data.get('profile', 'UNKNOWN')})")
                                if on_progress:
                                    on_progress("engine", "PASS")

                        # Verify API health checks
                        api_h = data.get("api_health", {})
                        if api_h.get("initialize") == "PASS":
                            api_healthy = True
                            if on_progress:
                                on_progress("api", "PASS")

                        if backend_ready and engine_ready:
                            self.logger.info("Health verification passed successfully.")
                            return True, data
            except Exception:
                pass

            time.sleep(0.2)

        self.logger.error(f"Health check timed out after {timeout_s:.1f}s.")
        return False, None

    def open_dashboard_in_browser(self, port: int) -> bool:
        """Opens the existing dashboard in the system default web browser."""
        url = f"http://{self.config.host}:{port}"
        self.logger.info(f"Opening dashboard UI at {url}")

        if self.config.browser_opener:
            return self.config.browser_opener(url)

        if self.config.open_browser:
            try:
                return webbrowser.open(url)
            except Exception as e:
                self.logger.error(f"Failed to open browser: {e}")
                return False
        return True

    def _safe_log(self, level: str, msg: str):
        """Safely logs messages even if streams are closing during atexit."""
        try:
            for h in list(self.logger.handlers):
                stream = getattr(h, "stream", None)
                if stream is not None and getattr(stream, "closed", False):
                    self.logger.removeHandler(h)
            if not self.logger.handlers:
                return
            log_fn = getattr(self.logger, level, self.logger.info)
            log_fn(msg)
        except Exception:
            pass

    def shutdown(self):
        """Terminates spawned backend processes gracefully and releases resources."""
        with self._shutdown_lock:
            if not self.is_running and self.backend_process is None:
                return

            self._safe_log("info", "Shutdown initiated. Cleaning up child processes...")

            if self.backend_process is not None:
                proc = self.backend_process
                self.backend_process = None

                if proc.poll() is None:
                    self._safe_log("info", f"Sending SIGTERM to child process PID {proc.pid}...")
                    try:
                        proc.terminate()
                        proc.wait(timeout=2.0)
                        self._safe_log("info", f"Child process PID {proc.pid} terminated cleanly.")
                    except Exception:
                        self._safe_log("warning", f"Process PID {proc.pid} did not exit in 2s. Forcing kill...")
                        try:
                            proc.kill()
                            proc.wait(timeout=1.0)
                        except Exception:
                            pass

            self.is_running = False
            self._safe_log("info", "Launcher shutdown complete.")

    def run(self) -> int:
        """
        Executes the complete launch sequence:
        1. Port selection
        2. Startup splash presentation (if GUI available)
        3. Spawning/reusing backend
        4. Health check polling
        5. Browser opening
        6. Process monitoring until interrupted
        """
        self.is_running = True
        self.logger.info("=" * 60)
        self.logger.info("ISRO IDR DESKTOP LAUNCHER STARTUP")
        self.logger.info("=" * 60)

        # 1. Port selection
        self.actual_port, self.is_reusing_existing = select_best_port(self.config, self.logger)
        dashboard_url = f"http://{self.config.host}:{self.actual_port}"

        # 2. Check if GUI splash is usable
        gui_available = not self.config.headless
        if gui_available:
            try:
                import tkinter
            except ImportError:
                gui_available = False

        if gui_available:
            return self._run_with_splash_gui(dashboard_url)
        else:
            return self._run_headless(dashboard_url)

    def _run_headless(self, dashboard_url: str) -> int:
        """Headless or CLI launch mode."""
        self.logger.info(f"Starting in headless/CLI mode on {dashboard_url}...")

        if not self.is_reusing_existing:
            self.start_backend_process(self.actual_port)

        ok, state = self.poll_health(self.actual_port, self.config.timeout_s)
        if not ok:
            self.logger.error("Failed to verify backend health.")
            self.shutdown()
            return 1

        self.open_dashboard_in_browser(self.actual_port)

        # If we started a child process, monitor it until interrupted
        if self.backend_process is not None:
            self.logger.info("Backend running. Press Ctrl+C in terminal to stop.")
            try:
                while self.is_running and self.backend_process.poll() is None:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                self.logger.info("KeyboardInterrupt received.")
            finally:
                self.shutdown()
        return 0

    def _run_with_splash_gui(self, dashboard_url: str) -> int:
        """Launches the elegant Tkinter aerospace splash window."""
        try:
            import tkinter as tk
            from tkinter import ttk, messagebox
        except Exception:
            return self._run_headless(dashboard_url)

        # Check if a display server is actually available (e.g. macOS WindowServer or X11)
        try:
            root = tk.Tk()
        except Exception as e:
            self.logger.warning(f"Tkinter display initialization failed ({e}). Falling back to headless.")
            return self._run_headless(dashboard_url)

        # Setup splash window properties
        root.title("ISRO IDR — Edge Navigation Engine")
        root.geometry("420x290")
        root.resizable(False, False)
        root.configure(bg="#FFFFFF")

        # Center on screen
        root.update_idletasks()
        screen_w = root.winfo_screenwidth()
        screen_h = root.winfo_screenheight()
        x = (screen_w - 420) // 2
        y = (screen_h - 290) // 2
        root.geometry(f"420x290+{x}+{y}")

        # Header Frame
        header_frame = tk.Frame(root, bg="#FFFFFF")
        header_frame.pack(fill="x", padx=24, pady=(24, 12))

        lbl_org = tk.Label(header_frame, text="ISRO IDR", font=("SF Pro Display", 18, "bold"), fg="#172033", bg="#FFFFFF")
        lbl_org.pack(anchor="w")

        lbl_sub = tk.Label(header_frame, text="Edge Navigation Engine", font=("SF Pro Text", 12), fg="#64748B", bg="#FFFFFF")
        lbl_sub.pack(anchor="w")

        # Divider
        sep = tk.Frame(root, height=1, bg="#E2E8F0")
        sep.pack(fill="x", padx=24, pady=8)

        # Status text
        status_var = tk.StringVar(value="● Initializing application...")
        lbl_status = tk.Label(root, textvariable=status_var, font=("SF Pro Text", 11, "bold"), fg="#0284C7", bg="#FFFFFF")
        lbl_status.pack(anchor="w", padx=24, pady=(4, 12))

        # Checklist frame
        chk_frame = tk.Frame(root, bg="#FFFFFF")
        chk_frame.pack(fill="x", padx=24, pady=0)

        chk_backend_var = tk.StringVar(value="○ Dashboard backend")
        chk_engine_var = tk.StringVar(value="○ Edge Engine")
        chk_api_var = tk.StringVar(value="○ API health")

        lbl_chk1 = tk.Label(chk_frame, textvariable=chk_backend_var, font=("SF Pro Text", 11), fg="#64748B", bg="#FFFFFF")
        lbl_chk1.pack(anchor="w", pady=2)

        lbl_chk2 = tk.Label(chk_frame, textvariable=chk_engine_var, font=("SF Pro Text", 11), fg="#64748B", bg="#FFFFFF")
        lbl_chk2.pack(anchor="w", pady=2)

        lbl_chk3 = tk.Label(chk_frame, textvariable=chk_api_var, font=("SF Pro Text", 11), fg="#64748B", bg="#FFFFFF")
        lbl_chk3.pack(anchor="w", pady=2)

        # Bottom info
        lbl_bottom = tk.Label(root, text="Starting local navigation engine...", font=("SF Pro Text", 10), fg="#94A3B8", bg="#FFFFFF")
        lbl_bottom.pack(side="bottom", pady=16)

        # Threading for background launch & health check
        launch_success = [False]

        def on_progress_update(step: str, status: str):
            if step == "backend" and status == "PASS":
                chk_backend_var.set("✓ Dashboard backend")
                lbl_chk1.configure(fg="#16A34A")
            elif step == "engine" and status == "PASS":
                chk_engine_var.set("✓ Edge Engine")
                lbl_chk2.configure(fg="#16A34A")
            elif step == "api" and status == "PASS":
                chk_api_var.set("✓ API health")
                lbl_chk3.configure(fg="#16A34A")

        def worker():
            try:
                if not self.is_reusing_existing:
                    self.start_backend_process(self.actual_port)

                ok, _ = self.poll_health(self.actual_port, self.config.timeout_s, on_progress=on_progress_update)
                launch_success[0] = ok

                if ok:
                    status_var.set("● Opening dashboard...")
                    lbl_status.configure(fg="#16A34A")
                    time.sleep(0.4)
                    self.open_dashboard_in_browser(self.actual_port)
                    # Close splash window
                    root.after(300, root.destroy)
                else:
                    status_var.set("✕ Startup Failed")
                    lbl_status.configure(fg="#DC2626")
                    # Display error options
                    root.after(100, show_error_dialog)
            except Exception as e:
                self.logger.error(f"Exception in splash worker: {e}")
                status_var.set("✕ Startup Error")
                root.after(100, show_error_dialog)

        def show_error_dialog():
            btn_frame = tk.Frame(root, bg="#FFFFFF")
            btn_frame.pack(side="bottom", pady=10)

            def on_retry():
                root.destroy()
                IDRLauncher(self.config).run()

            def on_open_logs():
                if sys.platform == "darwin":
                    subprocess.run(["open", self.config.log_file])
                elif sys.platform == "win32":
                    os.startfile(self.config.log_file)
                else:
                    subprocess.run(["xdg-open", self.config.log_file])

            def on_exit():
                self.shutdown()
                root.destroy()

            tk.Button(btn_frame, text="Retry", command=on_retry, padx=8).pack(side="left", padx=4)
            tk.Button(btn_frame, text="Open Logs", command=on_open_logs, padx=8).pack(side="left", padx=4)
            tk.Button(btn_frame, text="Exit", command=on_exit, padx=8).pack(side="left", padx=4)

        t = threading.Thread(target=worker, daemon=True)
        t.start()

        # Handle window close
        root.protocol("WM_DELETE_WINDOW", lambda: (self.shutdown(), root.destroy()))

        root.mainloop()

        # Keep parent process alive while child runs in background if not detached
        if launch_success[0] and self.backend_process is not None:
            try:
                while self.is_running and self.backend_process.poll() is None:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                pass
            finally:
                self.shutdown()

        return 0 if launch_success[0] else 1
