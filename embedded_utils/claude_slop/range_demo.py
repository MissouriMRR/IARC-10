#!/usr/bin/env python3
"""
range_demo.py - Bring up a basic two-board FiRa ranging session over UCI
between two Qorvo DWM3001CDKs (each plugged into its own Raspberry Pi
Zero 2 W, or both plugged into one Pi for a quick bench test).

Run this on BOTH Pis at (roughly) the same time - one as --role controller,
the other as --role controlee. They just need to agree on the session id,
channel, and each other's short MAC address.

Example (Pi #1, plugged into the "controller" board):
    python3 range_demo.py --port /dev/ttyACM0 --role controller \\
        --session-id 42 --my-addr 0x0001 --peer-addr 0x0002

Example (Pi #2, plugged into the "controlee" board):
    python3 range_demo.py --port /dev/ttyACM0 --role controlee \\
        --session-id 42 --my-addr 0x0002 --peer-addr 0x0001

If you don't know the port yet, run:
    python3 -c "from uci import find_ports; import pprint; pprint.pprint(find_ports())"
and look for the DWM3001CDK's *second* USB interface (the nRF52833's own
USB-CDC port used for UCI - not the J-Link debug port).
"""

import argparse
import logging
import sys
import time

from uci import (
    UciTransport, UciDevice, UciTimeout,
    DEVICE_TYPE_CONTROLLER, DEVICE_TYPE_CONTROLEE,
    DEVICE_ROLE_INITIATOR, DEVICE_ROLE_RESPONDER,
    build_fira_app_config_tlvs, status_name,
)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="serial device, e.g. /dev/ttyACM0")
    ap.add_argument("--role", required=True, choices=["controller", "controlee"],
                     help="controller=Initiator side, controlee=Responder side")
    ap.add_argument("--session-id", type=lambda x: int(x, 0), default=42)
    ap.add_argument("--my-addr", type=lambda x: int(x, 0), required=True,
                     help="this board's short MAC address, e.g. 0x0001")
    ap.add_argument("--peer-addr", type=lambda x: int(x, 0), required=True,
                     help="the other board's short MAC address, e.g. 0x0002")
    ap.add_argument("--channel", type=int, default=9, choices=[5, 9])
    ap.add_argument("--interval-ms", type=int, default=200)
    ap.add_argument("--count", type=int, default=0, help="stop after N ranging reports (0 = run forever)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                         format="%(asctime)s %(levelname)-7s %(message)s")
    log = logging.getLogger("range_demo")

    with UciTransport(args.port) as t:
        dev = UciDevice(t)

        log.info("Resetting device...")
        status = dev.device_reset()
        log.info("  device_reset -> %s", status_name(status))
        time.sleep(0.3)  # give the firmware a moment after reset

        info = dev.get_device_info()
        log.info("Device info: uci=%s mac=%s phy=%s vendor=%s",
                  info["uci_version"], info["mac_version"], info["phy_version"],
                  info["vendor_info"].hex())

        log.info("Initializing session %d...", args.session_id)
        status = dev.session_init(args.session_id)
        log.info("  session_init -> %s", status_name(status))
        if status != 0x00:
            log.error("session_init failed, aborting")
            sys.exit(1)

        if args.role == "controller":
            device_type = DEVICE_TYPE_CONTROLLER
            device_role = DEVICE_ROLE_INITIATOR
        else:
            device_type = DEVICE_TYPE_CONTROLEE
            device_role = DEVICE_ROLE_RESPONDER

        tlvs = build_fira_app_config_tlvs(
            device_type=device_type,
            device_role=device_role,
            device_short_addr=args.my_addr,
            dst_short_addr=args.peer_addr,
            channel_number=args.channel,
            ranging_interval_ms=args.interval_ms,
        )
        log.info("Setting app config (%s / %s)...",
                  "CONTROLLER" if device_type == DEVICE_TYPE_CONTROLLER else "CONTROLEE",
                  "INITIATOR" if device_role == DEVICE_ROLE_INITIATOR else "RESPONDER")
        rsp = dev.session_set_app_config(args.session_id, tlvs)
        status = rsp.payload[0]
        log.info("  session_set_app_config -> %s", status_name(status))
        if status != 0x00:
            log.error("app config rejected, raw rsp payload: %s", rsp.payload.hex())
            sys.exit(1)

        # Session should now be in SESSION_STATE_IDLE. Wait for the
        # notification just to log the transition (optional).
        try:
            st = dev.wait_session_status(timeout=1.0)
            log.info("Session status NTF: %s", st)
        except UciTimeout:
            pass

        log.info("Starting ranging...")
        status = dev.range_start(args.session_id)
        log.info("  range_start -> %s", status_name(status))
        if status != 0x00:
            log.error("range_start failed, aborting")
            sys.exit(1)

        n = 0
        try:
            while args.count == 0 or n < args.count:
                try:
                    data = dev.wait_range_data(timeout=5.0)
                except UciTimeout:
                    log.warning("no ranging data in 5s - check the other board is running too")
                    continue
                n += 1
                if not data.measurements:
                    log.info("[%d] seq=%d no measurements this round", n, data.sequence_number)
                    continue
                for m in data.measurements:
                    if m.ok:
                        log.info("[%d] seq=%d peer=0x%04X distance=%.2f m  aoa_az=%.1f deg  aoa_el=%.1f deg",
                                  n, data.sequence_number, m.mac_address, m.distance_cm / 100.0,
                                  m.aoa_azimuth_deg, m.aoa_elevation_deg)
                    else:
                        log.info("[%d] seq=%d peer=0x%04X measurement status=0x%02X (not OK)",
                                  n, data.sequence_number, m.mac_address, m.status)
        except KeyboardInterrupt:
            log.info("Interrupted, stopping ranging...")
        finally:
            dev.range_stop(args.session_id)
            dev.session_deinit(args.session_id)
            log.info("Stopped and cleaned up.")


if __name__ == "__main__":
    main()
