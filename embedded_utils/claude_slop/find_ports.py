#!/usr/bin/env python3
"""
find_ports.py - list serial ports on this Pi so you can figure out which
/dev/ttyACM* (or /dev/serial/by-id/...) belongs to the DWM3001CDK's UCI
interface.

The DWM3001CDK has TWO USB connectors:
  - the LOWER port (often silkscreened J9) goes to the on-board J-Link -
    debug/flash only, not UCI.
  - the UPPER port (often silkscreened J20) is the nRF52833's own USB,
    which the stock UCI firmware exposes as a USB-CDC ACM serial port -
    THIS is the one you want for range_demo.py.

If you're unsure which /dev/ttyACM* is which, unplug everything, plug in
just the UPPER port of one board, run this script, note the device, then
repeat for the LOWER port and confirm it's a different (J-Link) device.
Prefer wiring by /dev/serial/by-id/... paths in your own scripts since
ttyACM numbering can shuffle across reboots when you have several boards.
"""
import pprint
from uci import find_ports

if __name__ == "__main__":
    pprint.pprint(find_ports())
