"""
Fake `dronekit` and `picamera2` modules, installed into sys.modules so
state_machine's own code (and vision.Cameras.RPICamera.RPICamera, imported
unconditionally by takeoff_impl.py) can be imported in a dev environment
that has neither package installed -- this repo's actual dev/CI machine,
confirmed via ModuleNotFoundError on both.

install() MUST run before anything imports state_machine.* or vision.* --
those modules do `import dronekit` / `from picamera2 import ...` at their
own top level, which Python resolves via sys.modules at import time. This
module itself, and its caller, deliberately live OUTSIDE the state_machine
package (see this directory's own __init__.py) -- state_machine/__init__.py
does `from state_machine.drone import Drone` (needs real dronekit) the
moment ANY state_machine.* submodule is first touched, including one meant
to install a fake dronekit -- so the installer can't live inside the
package it's patching for.

Only the symbols state_machine/RPICamera actually reference by name are
provided (see one_drone_sim_test.py's own audit: dronekit.Vehicle/connect/
VehicleMode/LocationGlobalRelative; picamera2.Picamera2, picamera2.devices.
IMX500, picamera2.devices.imx500.NetworkIntrinsics) -- this is a shim for
import-time survival, not a dronekit/picamera2 reimplementation. The actual
simulated vehicle/camera behavior lives in one_drone_sim_test.py's
MockVehicle/MockCamera, which never touch these classes at runtime (the
testbench sets Drone._vehicle directly and monkeypatches takeoff_impl's
RPICamera name, bypassing dronekit.connect()/real RPICamera construction
entirely).
"""

import sys
import types


def install() -> None:
    _install_dronekit()
    _install_picamera2()


def _install_dronekit() -> None:
    if "dronekit" in sys.modules:
        return
    dronekit = types.ModuleType("dronekit")

    class Vehicle:
        """Referenced only as a type hint (state_machine/drone.py's
        `self._vehicle: dronekit.Vehicle | None`), evaluated eagerly at
        class-body execution time since that file has no
        `from __future__ import annotations`. Nothing does an isinstance
        check against this, so MockVehicle doesn't need to subclass it."""

    class VehicleMode:
        def __init__(self, name: str):
            self.name = name

        def __repr__(self) -> str:
            return f"VehicleMode({self.name!r})"

    class LocationGlobalRelative:
        def __init__(self, lat: float, lon: float, alt: float | None = None):
            self.lat = lat
            self.lon = lon
            self.alt = alt

    class LocationGlobal:
        def __init__(self, lat: float, lon: float, alt: float | None = None):
            self.lat = lat
            self.lon = lon
            self.alt = alt

    def connect(*_args, **_kwargs):
        raise RuntimeError(
            "dronekit shim: connect() should never be called in a simulated"
            " run -- the testbench sets Drone._vehicle directly instead of"
            " calling Drone.connect_drone()"
        )

    dronekit.Vehicle = Vehicle
    dronekit.VehicleMode = VehicleMode
    dronekit.LocationGlobalRelative = LocationGlobalRelative
    dronekit.LocationGlobal = LocationGlobal
    dronekit.connect = connect
    sys.modules["dronekit"] = dronekit


def _install_picamera2() -> None:
    if "picamera2" in sys.modules:
        return
    picamera2 = types.ModuleType("picamera2")

    class Picamera2:
        """Never actually constructed in the testbench -- MockCamera
        replaces RPICamera wholesale (see one_drone_sim_test.py), so this
        only has to exist for RPICamera.py's own module-level import to
        succeed."""

    picamera2.Picamera2 = Picamera2

    devices = types.ModuleType("picamera2.devices")

    class IMX500:
        def __init__(self, *_args, **_kwargs):
            pass

    devices.IMX500 = IMX500

    imx500 = types.ModuleType("picamera2.devices.imx500")

    class NetworkIntrinsics:
        def __init__(self, *_args, **_kwargs):
            self.task = None
            self.inference_rate = 30

        def update_with_defaults(self) -> None:
            pass

    imx500.NetworkIntrinsics = NetworkIntrinsics
    devices.imx500 = imx500
    picamera2.devices = devices

    sys.modules["picamera2"] = picamera2
    sys.modules["picamera2.devices"] = devices
    sys.modules["picamera2.devices.imx500"] = imx500
