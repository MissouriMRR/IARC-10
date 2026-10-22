"""
Two-board ranging demo. Run on both Pis (from the repo root) at about the same time:

    python3 -m embedded_utils.uwb.demo --port /dev/ttyACM0 --controller --my-addr 0x0001 --peer 0x0002
    python3 -m embedded_utils.uwb.demo --port /dev/ttyACM0              --my-addr 0x0002 --peer 0x0001

A controller can range with several controlees: repeat --peer.
`--info` just prints the board's device info and exits.
"""

import argparse
import logging

from . import RangingSession, UciClient, UciTimeout, list_ports, name_of, Status


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial device; omit to list candidate ports")
    ap.add_argument("--info", action="store_true", help="print device info and exit")
    ap.add_argument("--controller", action="store_true", help="this board initiates (default: controlee)")
    ap.add_argument("--my-addr", type=lambda x: int(x, 0))
    ap.add_argument("--peer", type=lambda x: int(x, 0), action="append", default=[])
    ap.add_argument("--session-id", type=lambda x: int(x, 0), default=42)
    ap.add_argument("--channel", type=int, default=9, choices=[5, 9])
    ap.add_argument("--interval-ms", type=int, default=200)
    ap.add_argument("-v", "--verbose", action="store_true", help="log every packet")
    args = ap.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(message)s")
    log = logging.getLogger("uwb.demo")

    if not args.port:
        for kind, ports in list_ports().items():
            print(kind)
            for p in ports:
                print("  ", p)
        return

    with UciClient(args.port) as uwb:
        info = uwb.get_device_info()
        log.info("UCI %s, MAC %s, PHY %s, vendor %s", info.uci_version, info.mac_version,
                 info.phy_version, info.vendor_info.hex())
        if args.info:
            return
        if args.my_addr is None or not args.peer:
            ap.error("--my-addr and at least one --peer are required to range")

        session = RangingSession(uwb, session_id=args.session_id, controller=args.controller,
                                 my_address=args.my_addr, peers=args.peer,
                                 channel=args.channel, ranging_interval_ms=args.interval_ms)
        with session:
            log.info("ranging as %s (handle 0x%08X)",
                     "controller" if args.controller else "controlee", session.handle)
            try:
                for rnd in session.rounds(timeout=5.0):
                    for m in rnd.measurements:
                        if m.ok:
                            log.info("seq=%d peer=0x%04X %.2f m nlos=%d",
                                     rnd.sequence_number, m.mac_address, m.distance_m, m.nlos)
                        else:
                            log.info("seq=%d peer=0x%04X %s", rnd.sequence_number,
                                     m.mac_address, name_of(Status, m.status))
            except UciTimeout:
                log.error("no ranging rounds for 5 s - is the other board running?")
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
