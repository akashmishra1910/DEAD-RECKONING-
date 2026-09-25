"""
edge_dashboard package
======================
Local Laptop Test Dashboard for PS 26168 EdgeNavigationEngine.
"""
from .controller import EdgeTestController
from .server import run_server

__all__ = ["EdgeTestController", "run_server"]
