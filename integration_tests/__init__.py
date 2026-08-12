"""Cross-package integration tests (state_machine + flight + vision), kept
outside every package they exercise -- state_machine/__init__.py itself
does `from state_machine.drone import Drone`, which needs dronekit
installed, so a shim-installer living INSIDE state_machine/ would trigger
that real import (and fail) before it ever got a chance to install its
fake dronekit/picamera2 first. See _hardware_shims.py.
"""
