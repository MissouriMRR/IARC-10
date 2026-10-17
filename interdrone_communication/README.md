# Team 5 Interdrone Communication

Missouri S&T Multirotor Drone Design Team's code for inter-drone communication for the 2026 IARC competition.

## Environment Setup:

The steps below document how to set up your environment to work with this repo and perform local testing

### Required Software:

- **uv** 0.9.5
- Git

### How to set up environment with git and uv:

1. Clone the repo <code>git clone https://github.com/MissouriMRR/IARC-10.git</code>
2. cd into, <code>cd Interdrone-Communication</code>
3. Run <code>uv python install 3.12</code>
4. Run <code>uv sync</code>

## How to perform tests with the code:

1. Edit **config.json** to match your desired settings. If you wish to perform the test on your local network use 127.0.0.\* network and use the provided ports. If you wish to perform it over an external network, update the IP in config to match your computers ip.
2. After the config file is set up, simply run <code>uv run main.py #</code> (with number corresponding to which drone you are). This will start the server and client.

Tip:
Use the split terminal button in VScode to run multiple servers and clients at the same time.

## Telemetry to Mission Planner via the BATMAN VM

`drone-flight.service` runs MAVProxy and the flight code together. MAVProxy is the only
process that opens the FC serial port. It feeds the flight code over loopback and sends a
second copy over the mesh to the BATMAN VM. The VM passes each drone to its own UDP port on
the Windows host. Traffic goes both ways, so Mission Planner can also load params and send
commands.

```
FC --serial0--> Pi MAVProxy --127.0.0.1:14551--> run.py
                    \--bat0--> VM 169.254.97.201:1455N --> Windows :1455N --> Mission Planner
```

| Drone | Mesh / Windows UDP port |
|-------|-------------------------|
| 1     | 14551                   |
| 2     | 14552                   |
| 3     | 14553                   |
| 4     | 14554                   |

### Pi install (each drone)

```bash
sudo cp interdrone_communication/drone_flight_start.sh /usr/local/bin/
sudo cp interdrone_communication/drone-flight.service /etc/systemd/system/
sudo chmod +x /usr/local/bin/drone_flight_start.sh
# The old forwarder must be *stopped*, not just disabled -- a running instance keeps the port.
sudo systemctl disable --now mavproxy-forward.service 2>/dev/null
sudo systemctl daemon-reload
sudo systemctl enable --now drone-flight.service
journalctl -u drone-flight -f
```

The drone id comes from the hostname (`mrrdt-#`). Overrides go in `/etc/default/drone-flight`,
for example `MAVLINK_FORWARD=0` to skip MAVProxy and let the flight code open `/dev/serial0`
directly, or `FC_MASTER=tcp:<sim-host>:5762` to bench-test against a SITL.

### VM install

```bash
sudo cp interdrone_communication/vm_mavlink_relay.sh /usr/local/bin/
sudo cp interdrone_communication/mavlink-relay@.service /etc/systemd/system/
sudo chmod +x /usr/local/bin/vm_mavlink_relay.sh
sudo systemctl daemon-reload
sudo systemctl enable --now mavlink-relay@{1..4}
```

The relay sends to the VM's default gateway. That gateway is the Windows host under Hyper-V's
Default Switch and under VirtualBox NAT. With a bridged adapter or VMware NAT, set
`WINDOWS_HOST_IP=<windows ip>` in `/etc/default/mavlink-relay`.

### Mission Planner

Connect with **UDP** and enter the port for the drone you want (for example 14551 for drone 1).
Open one connection per drone. Windows Firewall must allow inbound UDP 14551-14554.

> Commands sent from Mission Planner go straight to the FC. Don't change mode, arm or
> disarm from Mission Planner during autonomous flight, because the flight code will fight
> it. Viewing telemetry and reading params is safe.
