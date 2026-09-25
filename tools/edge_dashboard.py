#!/usr/bin/env python3
"""
edge_dashboard.py
=================
Launch script for the ISRO PS 26168 Edge Engine Test Dashboard.
Usage:
    ./edge_dashboard.py
    python edge_dashboard.py [--port 8080] [--no-browser]
"""

import sys
import os
import argparse

# Ensure repo root is on python path
repo_root = os.path.dirname(os.path.abspath(__file__))
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

from edge_dashboard.server import run_server

def main():
    parser = argparse.ArgumentParser(description="ISRO PS 26168 Edge Engine Test Dashboard")
    parser.add_argument("--port", type=int, default=None, help="TCP port to bind (default: auto 8080+)")
    parser.add_argument("--no-browser", action="store_true", help="Do not open browser automatically")
    args = parser.parse_args()

    run_server(port=args.port, open_browser=not args.no_browser)

if __name__ == "__main__":
    main()
