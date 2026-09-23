#!/usr/bin/env python3
"""
scripts/idr_map_tool.py
=======================
Command-line entry point for the ISRO IDR Offline OpenStreetMap extraction & map tool.
"""

import sys
import os

# Add repo root to path
repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

from idr_map.cli import main

if __name__ == "__main__":
    sys.exit(main())
