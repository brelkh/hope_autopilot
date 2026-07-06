# hope_autopilot

A demo autopilot for **X-Plane 11** that flies a full mission — takeoff, waypoint
navigation, and an automatic ILS-style approach and landing — driven from a
browser-based map UI. Written as a single Python script that talks to X-Plane
over its UDP dataref interface.

> ⚠️ **Demo / educational project.** It flies a simulator, not a real aircraft.
> The control loops are simple proportional/PD gains tuned by hand for a
> **Cessna 172**; expect to retune for other aircraft.

## How it works

The script does two things at once:

1. **A Flask web server** (port `5000`) serves a Leaflet map. You click to drop
   waypoints, set a per-leg altitude, and hit **Send Path**. Buttons trigger
   **Start Takeoff** and an always-available **🛑 Kill Switch**.
2. **An autopilot thread** reads state from X-Plane over UDP and sends control
   inputs back (via joystick override + control datarefs), stepping through a
   flight-phase state machine.

### Flight phases

| Phase          | What it does                                                             |
| -------------- | ------------------------------------------------------------------------ |
| `IDLE`         | Waits for **Start Takeoff**. Sends nothing.                              |
| `TAKEOFF_ROLL` | Full throttle, holds runway heading with rudder, rotates at `Vr`.        |
| `CLIMB`        | Fixed climb attitude, wings level, retracts gear, climbs to cruise cap.  |
| `ENROUTE`      | Waypoint navigation at the 300 ft cruise cap. Ends by flying to the FAF. |
| `APPROACH`     | Localizer-style centerline tracking + 3° glideslope down to the runway. |
| `FLARE`        | Idle power, nose-up attitude, tracks centerline to touchdown.            |
| `LANDED`       | Rollout: brakes, holds runway heading until stopped.                    |
| `KILLED`       | Kill switch fired — all control released back to the pilot. Restart to reuse. |

After the last map waypoint, the autopilot finds the nearest suitable runway end
from X-Plane's `apt.dat`, places a **final approach fix (FAF)** on the extended
centerline, flies to it, then descends. If it crosses the threshold too high it
**goes around** and re-attempts (naturally picking the opposite runway end).

### Approach control model

The approach and flare use the standard **pitch-for-speed / power-for-path**
model rather than "pitch down to descend":

- **Elevator holds airspeed** (`APPROACH_SPEED_MS`) — nose up if fast, down if slow.
- **Throttle flies the glidepath** — above the 3° slope pull power off, below it add power.

These are two decoupled loops. Trying to descend by shoving the nose down instead
just trades height for speed and balloons; the earlier version compensated with
full flaps + full speedbrake + reverse throttle and ended up mushing the wing —
descending at a crawl and overshooting the runway.

## Requirements

- **X-Plane 11** with UDP data output enabled (Settings → Data Output / Network;
  the script sends/receives on the default ports below).
- **Python 3.8+**
- `pip install flask`

## Setup

1. **Point the script at your `apt.dat`** so it can find runways. Edit
   `APT_DAT_PATH` near the top of [`hope.py`](hope.py):
   ```
   <X-Plane install>/Custom Scenery/Global Airports/Earth nav data/apt.dat
   ```
   (or `.../Global Scenery/Global Airports/Earth nav data/apt.dat`). If it isn't
   found, auto-landing is disabled but takeoff/enroute still work.

2. **Check the network settings** if you're not running on the same machine as
   X-Plane. In [`hope.py`](hope.py):
   ```python
   XPLANE_IP = "localhost"   # X-Plane's IP
   SEND_PORT = 49000         # X-Plane's receive port
   RECV_PORT = 49001         # this script's receive port
   ```

## Running

1. Start X-Plane, position the aircraft (a **Cessna 172**) on a runway, engine running.
2. Launch the script:
   ```bash
   pip install flask
   python hope.py
   ```
3. Open <http://localhost:5000> in a browser.
4. Click the map to add waypoints, set the altitude, and press **Send Path**.
5. Press **Start Takeoff** (confirm the aircraft is lined up and ready).
6. Watch it fly the route and land. The terminal prints live per-phase telemetry.
7. **🛑 Kill Switch** at any time instantly returns control to the pilot.

## Tuning

Key constants live near the top of [`hope.py`](hope.py):

- `CRUISE_ALTITUDE_FT` — cruise cap held for the whole route (default 300 ft AGL).
- `ROTATE_SPEED_MS`, `CLIMB_PITCH_DEG` — takeoff/climb behavior.
- `APPROACH_SPEED_MS`, `APPROACH_FLAPS`, `APPROACH_THROTTLE_TRIM` — final-approach
  targets (tuned for the 172).
- `GLIDESLOPE_DEG`, `FINAL_APPROACH_DISTANCE_M`, `FLARE_ALT_AGL_M` — approach geometry.

The various control gains (e.g. `0.04 * pitch_error`, `CROSS_TRACK_GAIN`) are
proportional/PD constants — adjust in small steps and watch the telemetry.

## Safety notes

- This is a **simulator toy**. Do not connect it to anything real.
- The joystick override is engaged on takeoff; the **Kill Switch** disables it and
  zeroes control inputs so manual/joystick control works again immediately.
- Control datarefs (brakes, gear, flaps) persist across script restarts — the
  script releases brakes on takeoff to avoid a previous run leaving them locked.
