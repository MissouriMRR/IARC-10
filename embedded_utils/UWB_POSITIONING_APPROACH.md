# UWB Positioning Approach — Concept Overview

This document explains **why** we're building the UWB positioning system the way we are, before anyone writes code. Read this first so we're all solving the same problem the same way.

## The Goal

Give each drone an estimate of its own position **relative to the other drones**, using UWB, so our collision-avoidance logic has something to act on. IARC arenas are GPS-denied by design, so UWB is our primary positioning source — not a backup.

## Key Terms (read before the rest — everything below assumes these)

- **Ranging** — measuring the straight-line distance between two UWB radios. This is all a single UWB measurement gives you: a distance, no direction.
- **ToF (Time of Flight)** — the underlying physics: distance = (time a radio signal took to travel between two devices) × speed of light. UWB is precise enough to measure this down to sub-nanosecond timing.
- **TWR (Two-Way Ranging)** — the *method* used to measure ToF without needing perfectly synced clocks: device A and device B exchange a request and a reply, timestamp each step, and the round-trip timing math yields a distance. This is what our boards do, and their onboard firmware handles the math for us — our code just triggers a session and reads the result.
- **TDoA (Time Difference of Arrival)** — a *different* ranging architecture: one device broadcasts once, and multiple listening devices (with tightly synced clocks to each other) compare arrival times. Requires multiple synced anchors — doesn't apply to our one-anchor setup. Mentioned here only so nobody confuses it with TWR.
- **AoA / PDoA (Angle of Arrival / Phase Difference of Arrival)** — a technique for measuring the *direction* a signal came from, using multiple antennas at a known spacing. This is what our boards **cannot** do (see below).
- **Anchor** — a fixed, stationary UWB reference point at a known location. Not a drone.
- **Odometry** — a drone's own estimate of its short-term movement (distance traveled, heading), from IMU/flight-controller data — not from UWB.
- **Law of Cosines** — basic trigonometry for solving a triangle when you know all three side lengths (or two sides and an angle). This is the math we use to turn two distance measurements into an angle.
- **Bearing** — a direction expressed as a compass angle (e.g., "42° from drone's heading").

## The Hardware Constraint, in Short

Our boards (DWM3001CDK) are built around a **non-PDoA** chip — they can only measure distance, never angle, and that's a hardware limit, not something firmware can fix. True AoA-capable Qorvo boards exist but cost enough per unit that outfitting 4 drones isn't in budget right now. **Our approach gets us direction anyway, without needing AoA hardware at all** — see below.

## The Core Idea: Motion as a Substitute for an Antenna Array

AoA hardware measures angle by comparing signal phase across two antennas at a fixed, known spacing. We don't have that — but a drone in flight can create its own varying spacing just by moving, and we already know that movement (distance, heading) from odometry. **We turn the drone's flight path into the "antenna baseline" instead of using physical antennas.**

## The Algorithm

**Summary:** each drone measures its distance to the shared anchor, moves a short, known distance, and measures the distance again. Those two distances plus the known movement form a triangle, which we solve (Law of Cosines) for the angle to the anchor. Combining that angle with the drone's own compass heading gives a real-world bearing, and bearing + distance gives an (x, y) position relative to the anchor. Every drone repeats this independently against the *same* anchor, so all positions end up in one shared coordinate frame — which is what makes comparing drones for collision avoidance simple vector math instead of a harder multi-node localization problem.

This is the actual logic every drone runs, on a loop, continuously during flight:

```python
import math

def bearing_and_position_cycle(anchor, drone):
    # 1. First ranging measurement (side 1 of the triangle)
    d1 = anchor.twr_range(drone)

    # 2. Deliberate, tracked movement (this is our synthetic antenna baseline)
    drone.start_odometry_tracking()
    drone.fly_forward(distance=SHORT_BASELINE_DISTANCE)
    x, heading = drone.stop_odometry_tracking()  # baseline length + heading during the move

    # 3. Second ranging measurement (side 2 of the triangle)
    d2 = anchor.twr_range(drone)

    # 4. Law of cosines: solve for the angle between travel direction and anchor direction
    cos_theta = (d1**2 + x**2 - d2**2) / (2 * d1 * x)
    theta = math.acos(cos_theta)

    # 5. Resolve left/right ambiguity (+theta vs -theta) -- strategy TBD, see Known Limitations
    theta = disambiguate(theta, d1, d2, x)

    # 6. Convert relative angle to an absolute bearing using drone's own heading
    bearing = heading + theta  # or heading - theta, depending on disambiguation result

    # 7. Convert distance + bearing into (x, y) relative to the anchor (anchor = origin)
    local_x = d2 * math.cos(bearing)
    local_y = d2 * math.sin(bearing)

    # 8. Smooth the noisy per-cycle estimate
    smoothed_x, smoothed_y = position_filter.update(local_x, local_y)

    # 9. Share this drone's position with the others
    comms.broadcast(drone.id, smoothed_x, smoothed_y)

    return smoothed_x, smoothed_y
```

Walking through why each step exists:

1. **`d1 = TWR_range(anchor)`** — first side of our triangle. Standard two-way ranging call; the firmware returns a clean distance.
2. **Track odometry across a deliberate move** — this movement *is* our synthetic antenna baseline. `x` (distance moved) and `hdg` (heading during the move) both come from the flight controller, not UWB.
3. **`d2 = TWR_range(anchor)`** — second side of the triangle, measured from the new position.
4. **Law of Cosines** — with `d1`, `d2`, and `x` known, this solves for `theta`: the angle between the direction the drone just flew and the direction to the anchor. This is the step that produces "angle" out of nothing but distances.
5. **Disambiguate** — `cos(theta)` alone can't distinguish the anchor being on the left vs. right of the flight path; something else (a slight curve in the path, or a location prior) has to break the tie. This needs a concrete strategy before implementation — flagged in Known Limitations.
6. **Bearing** — combining `theta` with the drone's own known heading converts a *relative* angle into an *absolute* direction to the anchor.
7. **Polar → Cartesian** — `d2` + `bearing` gives an (x, y) position with the anchor as the origin.
8. **Filter** — raw per-cycle solves will be noisy; smoothing (even a basic moving average to start) keeps position estimates from jumping around.
9. **Broadcast** — since every drone runs this same loop against the *same* anchor, all positions land in one shared coordinate frame. Comparing two drones' positions for collision avoidance becomes plain vector subtraction — no drone-to-drone ranging required.

## Known Limitations to Design Around

- **Left/right ambiguity** — needs a concrete disambiguation strategy before this is usable (see step 5 above).
- **Baseline length vs. accuracy** — too short an `x` makes `theta` hypersensitive to ranging noise; needs empirical tuning.
- **Anchor must stay still**, and odometry accuracy over the short P1→P2 window needs validating, not assumed.
- **Rules check pending** — confirm a fixed external anchor is allowed once 2027 rules drop.

## Implementation Breakdown

- **Ranging module** — triggers TWR sessions via Qorvo's UCI/QM33 SDK, returns distances.
- **Odometry module** — captures `x` and heading between ranging points.
- **Position solver** — Law of Cosines, disambiguation, bearing, Cartesian conversion.
- **Filter** — smooths noisy per-cycle position estimates.
- **Comms** — shares each drone's computed position with the others.

If your task touches any of the above, this doc should tell you *why* it's built this way — ping the team if something here doesn't match what you're seeing in practice.
