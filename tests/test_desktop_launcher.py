"""
tests/test_desktop_launcher.py
==============================
Automated test suite verifying the thin desktop launcher / orchestrator:
1. Configuration and logger setup.
2. Port availability detection and conflict resolution.
3. Backend health check polling and timeout handling.
4. Browser opening callback.
5. Process tracking, termination, and clean shutdown.
6. Reuse of existing running backend.
7. Headless execution flow.
"""

import os
import sys
import time
import json
import socket
import signal
import subprocess
from unittest.mock import MagicMock, patch
import pytest

from edge_dashboard.launcher import (
    LauncherConfig,
    IDRLauncher,
    is_port_bindable,
    is_idr_backend_running,
    select_best_port,
    setup_launcher_logger,
)


def test_launcher_config_defaults():
    """Verifies default launcher configuration parameters."""
    cfg = LauncherConfig()
    assert cfg.preferred_port == 8080
    assert cfg.max_port == 8100
    assert cfg.host == "127.0.0.1"
    assert cfg.dev_mode is False
    assert cfg.open_browser is True
    assert cfg.reuse_server is True
    assert cfg.timeout_s == 12.0
    assert "idr_launcher.log" in cfg.log_file


def test_port_bindable_check():
    """Verifies is_port_bindable correctly detects open vs occupied ports."""
    # Find an ephemeral open port
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        free_port = s.getsockname()[1]

    # Free port should be bindable
    assert is_port_bindable(free_port) is True

    # Occupied port should not be bindable
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(("127.0.0.1", free_port))
    server_sock.listen(1)

    assert is_port_bindable(free_port) is False
    server_sock.close()


def test_select_best_port_conflict_resolution():
    """Verifies dynamic port scanning when preferred port is occupied."""
    # Occupy port 8080 temporarily
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("127.0.0.1", 8080))
        s.listen(1)
        port_8080_occupied = True
    except Exception:
        port_8080_occupied = True  # Already occupied by daemon

    logger = setup_launcher_logger("logs/test_launcher.log", dev_mode=False)
    cfg = LauncherConfig(preferred_port=8080, max_port=8090, reuse_server=False)

    chosen_port, is_reused = select_best_port(cfg, logger)
    # Port 8080 is occupied, so launcher must pick a port > 8080
    assert chosen_port > 8080
    assert is_reused is False
    assert chosen_port <= 8090

    try:
        s.close()
    except Exception:
        pass


def test_poll_health_success():
    """Verifies successful polling of /api/state and progress callback invocations."""
    cfg = LauncherConfig(headless=True, timeout_s=3.0)
    launcher = IDRLauncher(cfg)

    mock_state = {
        "engine_status": "RUNNING",
        "profile": "EDGE_FOG",
        "api_health": {"initialize": "PASS"}
    }

    mock_response = MagicMock()
    mock_response.status = 200
    mock_response.read.return_value = json.dumps(mock_state).encode("utf-8")
    mock_response.__enter__.return_value = mock_response

    progress_events = []
    def on_prog(step, status):
        progress_events.append((step, status))

    with patch("urllib.request.urlopen", return_value=mock_response):
        ok, data = launcher.poll_health(port=8080, timeout_s=2.0, on_progress=on_prog)
        assert ok is True
        assert data["engine_status"] == "RUNNING"
        assert ("backend", "PASS") in progress_events
        assert ("engine", "PASS") in progress_events
        assert ("api", "PASS") in progress_events


def test_poll_health_timeout():
    """Verifies timeout handling when backend fails to respond."""
    cfg = LauncherConfig(headless=True, timeout_s=0.6)
    launcher = IDRLauncher(cfg)

    with patch("urllib.request.urlopen", side_effect=Exception("Connection refused")):
        ok, data = launcher.poll_health(port=8099, timeout_s=0.5)
        assert ok is False
        assert data is None


def test_browser_opener_callback():
    """Verifies browser launcher invokes browser_opener callback with correct URL."""
    opened_urls = []
    def mock_opener(url):
        opened_urls.append(url)
        return True

    cfg = LauncherConfig(preferred_port=8085, host="127.0.0.1", browser_opener=mock_opener)
    launcher = IDRLauncher(cfg)

    res = launcher.open_dashboard_in_browser(8085)
    assert res is True
    assert len(opened_urls) == 1
    assert opened_urls[0] == "http://127.0.0.1:8085"


def test_child_process_lifecycle_and_shutdown():
    """Verifies that child process is tracked, terminated on shutdown, and leaves no zombies."""
    cfg = LauncherConfig(headless=True)
    launcher = IDRLauncher(cfg)

    # Spawn dummy child process
    proc = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"])
    launcher.backend_process = proc
    launcher.is_running = True

    assert proc.poll() is None  # Process is running

    # Execute shutdown
    launcher.shutdown()

    # Process must be terminated
    assert proc.poll() is not None
    assert launcher.backend_process is None
    assert launcher.is_running is False


def test_reusing_existing_idr_backend():
    """Verifies that launcher detects an existing running IDR backend and marks reuse."""
    logger = setup_launcher_logger("logs/test_launcher.log", dev_mode=False)
    cfg = LauncherConfig(preferred_port=8080, reuse_server=True)

    with patch("edge_dashboard.launcher.is_idr_backend_running", return_value=True):
        chosen_port, is_reused = select_best_port(cfg, logger)
        assert chosen_port == 8080
        assert is_reused is True


def test_headless_run_flow_mocked():
    """Verifies complete run() sequence in headless mode."""
    opened_urls = []
    cfg = LauncherConfig(
        preferred_port=8080,
        headless=True,
        reuse_server=True,
        timeout_s=2.0,
        browser_opener=lambda url: opened_urls.append(url) or True
    )
    launcher = IDRLauncher(cfg)

    with patch("edge_dashboard.launcher.is_idr_backend_running", return_value=True), \
         patch.object(launcher, "poll_health", return_value=(True, {"engine_status": "RUNNING"})):
        rc = launcher.run()
        assert rc == 0
        assert len(opened_urls) == 1
        assert "8080" in opened_urls[0]
