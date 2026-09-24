#!/usr/bin/env python3
"""
launch_idr_app.py
=================
Desktop launcher entrypoint for the ISRO IDR Edge Engine Dashboard.
Supports standard double-click desktop launching as well as CLI developer mode.

Usage:
    python3 launch_idr_app.py
    python3 launch_idr_app.py --dev
    python3 launch_idr_app.py --port 8080 --no-browser
    python3 launch_idr_app.py --headless
"""

import sys
import os
import argparse

# Ensure repo root is on sys.path
REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from edge_dashboard.launcher import IDRLauncher, LauncherConfig


def main():
    parser = argparse.ArgumentParser(description="ISRO IDR — Desktop Engine Launcher")
    parser.add_argument("--port", type=int, default=8080, help="Preferred TCP port (default: 8080)")
    parser.add_argument("--dev", action="store_true", help="Enable developer mode (verbose console output)")
    parser.add_argument("--no-browser", action="store_true", help="Do not open browser automatically")
    parser.add_argument("--no-reuse", action="store_true", help="Do not reuse existing running server")
    parser.add_argument("--timeout", type=float, default=12.0, help="Startup health check timeout in seconds (default: 12.0)")
    parser.add_argument("--headless", action="store_true", help="Run in headless CLI mode without Tkinter splash window")
    args = parser.parse_args()

    config = LauncherConfig(
        preferred_port=args.port,
        dev_mode=args.dev,
        open_browser=not args.no_browser,
        reuse_server=not args.no_reuse,
        timeout_s=args.timeout,
        headless=args.headless,
    )

    launcher = IDRLauncher(config)
    exit_code = launcher.run()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
