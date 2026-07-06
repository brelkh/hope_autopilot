import socket
import struct
import time
import threading
import math
from flask import Flask, request, jsonify

# ----------------------------
# X-PLANE CONNECTION
# ----------------------------
XPLANE_IP = "localhost"
SEND_PORT = 49000
RECV_PORT = 49001

# Path to X-Plane's airport/runway database on this machine.
# Typically: <X-Plane install>/Global Scenery/Global Airports/Earth nav data/apt.dat
# Update this to match your actual install location.
APT_DAT_PATH = r"C:\X-Plane 11\Custom Scenery\Global Airports\Earth nav data\apt.dat"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", RECV_PORT))

if hasattr(socket, "SIO_UDP_CONNRESET"):
    sock.ioctl(socket.SIO_UDP_CONNRESET, False)

# ----------------------------
# GLOBAL STATE
# ----------------------------
current_lat = 0.0
current_lon = 0.0
current_heading = 0.0
current_roll = 0.0
current_pitch = 0.0
current_altitude = 0.0
current_groundspeed = 0.0
current_airspeed = 0.0  # m/s true airspeed -- what the wing actually flies on (see request_data)
current_onground = 1.0  # 1 = on ground, 0 = airborne
current_pitch_rate = 0.0  # deg/s, positive = pitching up (assumed; verify if unsure)
current_roll_rate = 0.0   # deg/s

path = []
path_lock = threading.Lock()
current_wp_index = 0

WAYPOINT_RADIUS_M = 500.0
DEFAULT_TARGET_ALTITUDE = 1500.0  # meters MSL (unused now that cruise is capped -- see CRUISE_ALTITUDE_M)

# Cruise altitude cap: 300 feet AGL, held constant for the entire flight
# regardless of what altitude was set per-waypoint on the map.
CRUISE_ALTITUDE_FT = 300.0
FEET_TO_METERS = 0.3048
CRUISE_ALTITUDE_M = CRUISE_ALTITUDE_FT * FEET_TO_METERS  # ~91.4m

# Enroute cruise, same pitch-for-speed / power-for-path model as the approach:
# the elevator holds CRUISE_SPEED_MS, and the throttle trims around a baseline
# to hold the cruise altitude cap (add power to hold/climb, pull it to descend).
CRUISE_SPEED_MS = 46.0       # ~90 kts -- comfortable 172 low-level cruise
CRUISE_THROTTLE_TRIM = 0.55  # baseline power that roughly holds the cruise cap

# ----------------------------
# FLIGHT PHASE STATE MACHINE
# ----------------------------
# IDLE          -> autopilot does nothing, waiting for "Start Takeoff"
# TAKEOFF_ROLL  -> full throttle, hold runway heading with rudder, rotate at speed
# CLIMB         -> fixed climb pitch, wings level, climb to a safe altitude
# ENROUTE       -> normal waypoint navigation (your existing logic)
flight_phase = "IDLE"
phase_lock = threading.Lock()

runway_heading = None      # captured at takeoff start
takeoff_elevation = None   # captured at liftoff

ROTATE_SPEED_MS = 30.0     # ~58 kts groundspeed -- TUNE to your aircraft's Vr
CLIMB_PITCH_DEG = 8.0
CLIMB_ALTITUDE_AGL = CRUISE_ALTITUDE_M  # climb to the 300ft cruise cap, then level off
GEAR_RETRACT_AGL_M = 30.0   # retract gear once this far above takeoff elevation
gear_is_down = True         # tracks commanded gear state (avoids resending every tick)

# ----------------------------
# LANDING CONFIG
# ----------------------------
FINAL_APPROACH_DISTANCE_M = 15000.0  # length of straight-in final approach leg (lengthened -- more room to line up before descending)
GLIDESLOPE_DEG = 3.0                # standard 3-degree glideslope
FLARE_ALT_AGL_M = 15.0              # height above runway to begin flare
FLARE_PITCH_DEG = 4.0               # nose-up attitude during flare
LANDED_GROUNDSPEED_MS = 2.0         # below this, consider the rollout complete

# Cessna 172 final-approach targets (pitch-for-speed / power-for-path model).
# Elevator holds APPROACH_SPEED_MS; throttle holds the glidepath around a
# baseline trim setting. Retune these numbers for a different aircraft.
APPROACH_SPEED_MS = 33.0           # ~64 kts -- 172 short-final speed
APPROACH_FLAPS = 0.66              # ~20 deg -- partial flaps, NOT full; a 172 has no speedbrake
APPROACH_THROTTLE_TRIM = 0.30      # baseline power that roughly holds a 3-deg glide at approach speed

landing_runway = None   # dict: {icao, lat, lon, heading, elevation_m} once selected
faf_point = None         # dict: {lat, lon} -- final approach fix, ahead of APPROACH entry
faf_inserted = False     # tracks whether the FAF has been added as a real waypoint yet

# ----------------------------
# HELPERS
# ----------------------------
def send_dref(name, value):
    msg = struct.pack("<5sf500s", b"DREF\x00", float(value), name + b"\x00")
    sock.sendto(msg, (XPLANE_IP, SEND_PORT))

def send_elevator(v):
    send_dref(b"sim/joystick/yoke_pitch_ratio", v)

def send_aileron(v):
    send_dref(b"sim/joystick/yoke_roll_ratio", v)

def send_rudder(v):
    send_dref(b"sim/joystick/yoke_heading_ratio", v)

def send_throttle(v):
    send_dref(b"sim/joystick/throttle_ratio", v)

def release_parking_brake():
    send_dref(b"sim/flightmodel/controls/parkbrake", 0.0)

def release_wheel_brakes():
    send_dref(b"sim/flightmodel/controls/l_brake_add", 0.0)
    send_dref(b"sim/flightmodel/controls/r_brake_add", 0.0)

def set_wheel_brakes(v):
    send_dref(b"sim/flightmodel/controls/l_brake_add", v)
    send_dref(b"sim/flightmodel/controls/r_brake_add", v)

def set_gear_down(down):
    send_dref(b"sim/cockpit2/controls/gear_handle_down", 1.0 if down else 0.0)

def set_flaps(ratio):
    # ratio: 0.0 (up) to 1.0 (full flaps)
    send_dref(b"sim/flightmodel2/controls/flap_handle_deploy_ratio", ratio)
    # Fallback for aircraft that don't respond to the handle-deploy dataref --
    # some (mostly older/simpler) aircraft models only listen to this one.
    send_dref(b"sim/flightmodel/controls/flaprqst", ratio)

def set_speedbrake(ratio):
    # ratio: 0.0 (stowed) to 1.0 (fully extended). Speedbrakes add drag
    # without changing pitch attitude -- much more effective for a steep,
    # controlled descent than pitching the nose down further, and doesn't
    # risk overspeeding the way an aggressive dive does.
    send_dref(b"sim/flightmodel/controls/speedbrake_ratio", ratio)

def enable_override():
    send_dref(b"sim/operation/override/override_joystick", 1)

def disable_override():
    send_dref(b"sim/operation/override/override_joystick", 0)

def kill_manual_control():
    """Emergency stop: immediately hand the aircraft back to the human pilot.
    Zeroes any control input the script was sending and releases the
    joystick override, so normal manual/joystick input works again right
    away regardless of what the autopilot was doing."""
    send_elevator(0.0)
    send_aileron(0.0)
    send_rudder(0.0)
    disable_override()

def zero_elevator_trim():
    send_dref(b"sim/flightmodel/controls/elv_trim", 0.0)

def disengage_xplane_autopilot():
    # Best-effort: not every dataref here is writable on every aircraft, but
    # this covers the common ones. If the plane still won't pitch down after
    # this, manually switch off the AP and re-center trim in the X-Plane
    # cockpit before hitting "Start Takeoff" -- that's the more reliable fix.
    send_dref(b"sim/cockpit/autopilot/autopilot_mode", 0.0)
    send_dref(b"sim/cockpit2/autopilot/servos_on", 0.0)

def bearing_offset_point(lat, lon, bearing_deg, distance_m):
    """Flat-earth approximation of a point at `distance_m` along `bearing_deg`
    from (lat, lon). Fine for short distances like a final approach leg."""
    R = 6371000.0
    brg = math.radians(bearing_deg)
    dlat = (distance_m * math.cos(brg)) / R
    dlon = (distance_m * math.sin(brg)) / (R * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)

def load_airports(path):
    """Parse an X-Plane apt.dat file into a flat list of landable runway ends.
    Each entry represents ONE end of a runway -- the point/heading an aircraft
    would use to land on that end and roll out toward the opposite end."""
    runway_ends = []
    current_elev_m = 0.0

    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                parts = line.split()
                if not parts:
                    continue

                if parts[0] in ("1", "16", "17"):  # airport header (land/sea/heliport)
                    try:
                        current_elev_m = float(parts[1]) * 0.3048
                    except (ValueError, IndexError):
                        current_elev_m = 0.0

                elif parts[0] == "100" and len(parts) >= 19:
                    # Runway row: two ends, each with its own lat/lon
                    try:
                        lat1, lon1 = float(parts[9]), float(parts[10])
                        lat2, lon2 = float(parts[18]), float(parts[19])
                    except (ValueError, IndexError):
                        continue

                    hdg_1to2 = get_heading_to_target(lat1, lon1, lat2, lon2)
                    hdg_2to1 = (hdg_1to2 + 180) % 360

                    runway_ends.append({
                        "lat": lat1, "lon": lon1, "heading": hdg_1to2,
                        "opp_lat": lat2, "opp_lon": lon2,
                        "elevation_m": current_elev_m,
                    })
                    runway_ends.append({
                        "lat": lat2, "lon": lon2, "heading": hdg_2to1,
                        "opp_lat": lat1, "opp_lon": lon1,
                        "elevation_m": current_elev_m,
                    })
    except FileNotFoundError:
        print(f"WARNING: apt.dat not found at {path} -- auto-landing disabled.")
        return []

    print(f"Loaded {len(runway_ends)} runway ends from apt.dat")
    return runway_ends

def find_nearest_runway_end(lat, lon, runway_ends):
    if not runway_ends:
        return None

    # Prefer ends where the aircraft is actually positioned in the correct
    # approach corridor -- i.e. behind the threshold, opposite the landing
    # direction (along_track negative). Picking by raw distance alone can
    # select an end whose landing heading points AWAY from the aircraft,
    # which sends it flying away from the field in a straight line while
    # the glideslope math chases an ever-growing target altitude.
    candidates = []
    for r in runway_ends:
        cross_track, along_track = cross_track_and_along_track(
            lat, lon, r["lat"], r["lon"], r["heading"]
        )
        candidates.append((r, cross_track, along_track))

    in_corridor = [(r, ct, at) for (r, ct, at) in candidates if at < 0]

    if in_corridor:
        # Among valid-corridor ends, pick the closest one overall
        best = min(in_corridor, key=lambda c: math.hypot(c[1], c[2]))
        return best[0]
    else:
        # Nothing is in a clean corridor position (e.g. directly overhead) --
        # fall back to simple nearest-threshold distance.
        best = min(runway_ends, key=lambda r: get_distance_m(lat, lon, r["lat"], r["lon"]))
        return best

def cross_track_and_along_track(lat, lon, ref_lat, ref_lon, course_deg):
    """Given an aircraft position and a reference point + course (e.g. runway
    threshold + runway heading), return (cross_track_m, along_track_m):
    cross_track: signed perpendicular distance from the extended course line
                 (positive = aircraft is to the right of course, when flying
                 the course direction; sign is a convention, flip if the
                 aircraft banks the wrong way when correcting)
    along_track: distance along the course line from ref point to aircraft's
                 projected position (roughly how far out on final it is)
    """
    dist = get_distance_m(ref_lat, ref_lon, lat, lon)
    bearing_to_ac = get_heading_to_target(ref_lat, ref_lon, lat, lon)
    angle = math.radians(normalize_error(bearing_to_ac - course_deg))
    cross_track = dist * math.sin(angle)
    along_track = dist * math.cos(angle)
    return cross_track, along_track

def get_heading_to_target(lat1, lon1, lat2, lon2):
    dlon = math.radians(lon2 - lon1)
    lat1r = math.radians(lat1)
    lat2r = math.radians(lat2)
    x = math.sin(dlon) * math.cos(lat2r)
    y = math.cos(lat1r) * math.sin(lat2r) - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlon)
    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360

def get_distance_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def normalize_error(err):
    if err > 180:
        err -= 360
    if err < -180:
        err += 360
    return err

# ----------------------------
# REQUEST DATA
# ----------------------------
def request_data():
    refs = [
        (101, b"sim/flightmodel/position/latitude"),
        (102, b"sim/flightmodel/position/longitude"),
        (103, b"sim/flightmodel/position/psi"),
        (104, b"sim/flightmodel/position/phi"),
        (105, b"sim/flightmodel/position/theta"),
        (106, b"sim/flightmodel/position/elevation"),
        (107, b"sim/flightmodel/position/groundspeed"),
        (108, b"sim/flightmodel/failures/onground_any"),
        (109, b"sim/flightmodel/position/Q"),  # pitch rotation rate, deg/s
        (110, b"sim/flightmodel/position/P"),  # roll rotation rate, deg/s
        (111, b"sim/flightmodel/position/true_airspeed"),  # m/s -- speed loop flies this, not groundspeed
    ]
    for idx, ref in refs:
        msg = struct.pack("<5sii400s", b"RREF\x00", 5, idx, ref + b"\x00")
        sock.sendto(msg, (XPLANE_IP, SEND_PORT))

# ----------------------------
# AUTOPILOT LOOP
# ----------------------------
def autopilot():
    global current_lat, current_lon, current_heading
    global current_roll, current_pitch, current_altitude
    global current_groundspeed, current_airspeed, current_onground, current_pitch_rate, current_roll_rate
    global current_wp_index, flight_phase, runway_heading, takeoff_elevation
    global gear_is_down, landing_runway, faf_point, faf_inserted

    dt = 0.05

    while True:
        try:
            data, _ = sock.recvfrom(4096)
        except OSError as e:
            print("recv error (ignored):", e)
            continue

        if not data.startswith(b"RREF,"):
            continue

        offset = 5
        while offset + 8 <= len(data):
            idx, value = struct.unpack_from("<if", data, offset)
            if idx == 101:
                current_lat = value
            elif idx == 102:
                current_lon = value
            elif idx == 103:
                current_heading = value
            elif idx == 104:
                current_roll = value
            elif idx == 105:
                current_pitch = value
            elif idx == 106:
                current_altitude = value
            elif idx == 107:
                current_groundspeed = value
            elif idx == 108:
                current_onground = value
            elif idx == 109:
                current_pitch_rate = value
            elif idx == 110:
                current_roll_rate = value
            elif idx == 111:
                current_airspeed = value
            offset += 8

        with phase_lock:
            phase = flight_phase

        # ============================================================
        # PHASE: KILLED -- emergency stop triggered. Do nothing, forever,
        # until the script is restarted. Control has been handed back.
        # ============================================================
        if phase == "KILLED":
            time.sleep(dt)
            continue

        # ============================================================
        # PHASE: IDLE -- do nothing, let the pilot/sim hold state
        # ============================================================
        if phase == "IDLE":
            print(f"[IDLE] Lat:{current_lat:.4f} Lon:{current_lon:.4f} "
                  f"GS:{current_groundspeed:.1f} OnGround:{current_onground:.0f}")
            time.sleep(dt)
            continue

        # ============================================================
        # PHASE: TAKEOFF_ROLL -- full power, hold runway heading, rotate
        # ============================================================
        if phase == "TAKEOFF_ROLL":
            send_throttle(1.0)

            heading_error = normalize_error(runway_heading - current_heading)
            rudder = max(-1.0, min(1.0, 0.03 * heading_error))
            send_rudder(rudder)
            send_aileron(0.0)

            if current_groundspeed >= ROTATE_SPEED_MS:
                elevator = 0.1  # gentle pull to rotate (softened from 0.2 -- was too aggressive)
            else:
                elevator = 0.0
            send_elevator(elevator)

            print(f"[TAKEOFF_ROLL] GS:{current_groundspeed:.1f} m/s "
                  f"HdgErr:{heading_error:.1f} OnGround:{current_onground:.0f}")

            if current_onground < 0.5:
                print("Liftoff detected -> CLIMB phase")
                takeoff_elevation = current_altitude
                with phase_lock:
                    flight_phase = "CLIMB"

            time.sleep(dt)
            continue

        # ============================================================
        # PHASE: CLIMB -- fixed climb attitude, wings level, gear up alt
        # ============================================================
        if phase == "CLIMB":
            send_throttle(0.75)

            pitch_error = CLIMB_PITCH_DEG - current_pitch
            elevator = 0.04 * pitch_error - 0.015 * current_pitch_rate  # PD: position + rate damping
            elevator = max(-0.6, min(0.3, elevator))  # widened -- needs to level off quickly at a much lower cruise target now
            send_elevator(elevator)

            roll_error = 0.0 - current_roll  # wings level
            aileron = 0.03 * roll_error
            aileron = max(-0.4, min(0.4, aileron))
            send_aileron(aileron)
            send_rudder(0.0)

            agl = current_altitude - (takeoff_elevation or current_altitude)
            print(f"[CLIMB] Alt:{current_altitude:.0f} AGL:{agl:.0f} Pitch:{current_pitch:.1f} "
                  f"PitchRate:{current_pitch_rate:.1f} Elevator:{elevator:.2f}")

            if gear_is_down and agl >= GEAR_RETRACT_AGL_M:
                print("Retracting gear")
                set_gear_down(False)
                gear_is_down = False

            if agl >= CLIMB_ALTITUDE_AGL:
                print("Climb altitude reached -> ENROUTE phase")
                with phase_lock:
                    flight_phase = "ENROUTE"

            time.sleep(dt)
            continue

        # ============================================================
        # PHASE: ENROUTE -- normal waypoint navigation
        # ============================================================
        if phase == "ENROUTE":
            active_wp = None
            with path_lock:
                if path and current_wp_index < len(path):
                    wp = path[current_wp_index]
                    dist = get_distance_m(current_lat, current_lon, wp["lat"], wp["lon"])
                    if dist < WAYPOINT_RADIUS_M:
                        print(f"Reached waypoint {current_wp_index} (dist {dist:.0f}m)")
                        current_wp_index += 1
                    if current_wp_index < len(path):
                        active_wp = path[current_wp_index]
                # Recomputed fresh every tick (not just on the crossing tick) so
                # that a go-around -- which resets faf_inserted and drops us
                # back into ENROUTE with the path already exhausted -- reliably
                # re-triggers the FAF-insertion logic below instead of getting
                # stuck with active_wp = None forever.
                path_complete = bool(path) and current_wp_index >= len(path)

            if path_complete:
                if not faf_inserted:
                    nearest = find_nearest_runway_end(current_lat, current_lon, airport_runways)
                    if nearest is not None:
                        dist_km = get_distance_m(current_lat, current_lon, nearest["lat"], nearest["lon"]) / 1000
                        print(f"Path complete. Nearest runway found {dist_km:.1f}km away, "
                              f"heading {nearest['heading']:.0f} -- flying to FAF to establish approach")
                        landing_runway = nearest
                        faf_lat, faf_lon = bearing_offset_point(
                            nearest["lat"], nearest["lon"],
                            (nearest["heading"] + 180) % 360,  # back along extended centerline
                            FINAL_APPROACH_DISTANCE_M
                        )
                        # Altitude that matches a proper glideslope AT that distance --
                        # so arriving here means we're already correctly positioned
                        # for a normal-length final descent, not stuck overhead.
                        faf_alt = nearest["elevation_m"] + FINAL_APPROACH_DISTANCE_M * math.tan(
                            math.radians(GLIDESLOPE_DEG)
                        )
                        faf_point = {"lat": faf_lat, "lon": faf_lon}
                        with path_lock:
                            path.append({"lat": faf_lat, "lon": faf_lon, "alt": faf_alt})
                            current_wp_index = len(path) - 1
                        faf_inserted = True
                        print(f"FAF waypoint added at alt {faf_alt:.0f}m -- continuing ENROUTE toward it")
                        time.sleep(dt)
                        continue
                    else:
                        print("Path complete but no airport data loaded -- holding level indefinitely. "
                              "Check APT_DAT_PATH.")
                else:
                    # We just arrived at the FAF waypoint itself (at the correct
                    # altitude for this distance) -- now start the precision glide.
                    print("FAF reached -- entering APPROACH")
                    with phase_lock:
                        flight_phase = "APPROACH"
                    time.sleep(dt)
                    continue

            # Always hold the 300ft cruise cap, regardless of what altitude
            # was set per-waypoint on the map -- level off once and stay
            # there for the whole route.
            target_altitude = takeoff_elevation + CRUISE_ALTITUDE_M if takeoff_elevation else CRUISE_ALTITUDE_M

            wp_status = f"WP {current_wp_index}/{len(path)}" if path else "no path"
            print(f"[ENROUTE] Lat:{current_lat:.4f} Lon:{current_lon:.4f} "
                  f"Head:{current_heading:.1f} Alt:{current_altitude:.0f} Spd:{current_airspeed:.0f} [{wp_status}]")

            # Same model as APPROACH: elevator holds airspeed, throttle holds
            # altitude. Descending by cutting throttle and pushing the nose
            # down just trades height for speed and won't settle -- on this
            # airframe it wouldn't come down at all. Let power do the vertical
            # work instead.

            # THROTTLE -> altitude. Above the cruise cap: pull power off to
            # sink; below it: add power to hold or climb back up.
            alt_error = current_altitude - target_altitude   # +ve = above target
            throttle = CRUISE_THROTTLE_TRIM - 0.01 * alt_error
            throttle = max(0.0, min(1.0, throttle))
            send_throttle(throttle)

            # ELEVATOR -> airspeed. Holds a steady cruise speed so the wing
            # keeps flying while the throttle moves us up or down.
            speed_error = current_airspeed - CRUISE_SPEED_MS  # +ve = too fast
            pitch_target = max(-8, min(8, 0.5 * speed_error))
            pitch_error = pitch_target - current_pitch
            elevator = 0.04 * pitch_error - 0.015 * current_pitch_rate
            elevator = max(-0.4, min(0.4, elevator))

            if active_wp is not None:
                desired_heading = get_heading_to_target(
                    current_lat, current_lon, active_wp["lat"], active_wp["lon"]
                )
                heading_error = normalize_error(desired_heading - current_heading)
                roll_target = 0.2 * heading_error
                roll_target = max(-20, min(20, roll_target))
            else:
                roll_target = 0

            roll_error = roll_target - current_roll
            aileron = 0.03 * roll_error
            aileron = max(-0.4, min(0.4, aileron))

            send_elevator(elevator)
            send_aileron(aileron)
            send_rudder(0.0)

            time.sleep(dt)
            continue

        # ============================================================
        # PHASE: APPROACH -- straight-in descent toward runway threshold
        # ============================================================
        if phase == "APPROACH":
            if gear_is_down is False:
                print("Extending gear, setting approach flaps")
                set_gear_down(True)
                gear_is_down = True
                set_flaps(APPROACH_FLAPS)   # partial flaps (0.0-1.0). Full flaps + full
                set_speedbrake(0.0)          # speedbrake (the old 2.0/5.0) just mushed the wing.

            dist_to_threshold = get_distance_m(
                current_lat, current_lon, landing_runway["lat"], landing_runway["lon"]
            )

            # Localizer-style guidance: instead of aiming at a point (which
            # converges slowly / can orbit), measure how far sideways we are
            # from the extended runway centerline (cross-track error) and bank
            # proportionally to correct it, always relative to the FIXED
            # runway heading. Big offset -> big intercept angle (bounded);
            # small offset -> gentle correction. This is what real ILS/
            # localizer autopilots do, and it avoids chasing any point at all.
            cross_track, along_track = cross_track_and_along_track(
                current_lat, current_lon,
                landing_runway["lat"], landing_runway["lon"],
                landing_runway["heading"]
            )

            # Old gain (0.08) was too weak -- your last log showed CrossTrack
            # drifting from -25m to -35m over the ENTIRE approach (getting
            # worse, not converging). That offset was invisible from far away
            # and only became obvious as a sudden "veer" once close to the
            # ground. Raised substantially so it actually closes the gap.
            CROSS_TRACK_GAIN = 0.3      # deg of intercept angle per meter of offset
            MAX_INTERCEPT_DEG = 35.0    # cap how aggressively it cuts back toward centerline

            # NOTE: if this makes it turn AWAY from the runway when offset,
            # flip the sign here (change - to +) -- cross-track sign convention
            # can go either way depending on which side "positive" lands on.
            intercept = max(-MAX_INTERCEPT_DEG, min(MAX_INTERCEPT_DEG, -CROSS_TRACK_GAIN * cross_track))
            desired_heading = (landing_runway["heading"] + intercept) % 360

            heading_error = normalize_error(desired_heading - current_heading)
            roll_target = max(-20, min(20, 0.2 * heading_error))
            roll_error = roll_target - current_roll
            aileron = max(-0.4, min(0.4, 0.03 * roll_error))
            send_aileron(aileron)

            rudder = max(-1.0, min(1.0, 0.02 * heading_error))
            send_rudder(rudder)

            # Glideslope: use along-track distance (position along the runway
            # centerline) rather than straight-line distance to threshold --
            # more accurate once cross-track offset is nonzero.
            # along_track is negative while properly positioned behind the
            # threshold (approaching); distance remaining to the threshold
            # along the corridor is therefore -along_track, clamped at 0
            # once past it.
            distance_remaining = max(0, -along_track)
            glide_alt_agl = distance_remaining * math.tan(math.radians(GLIDESLOPE_DEG))
            # Cap at cruise altitude -- with a much longer final leg, the raw
            # glideslope math would otherwise compute a target well above the
            # 300ft cruise cap far out, asking the plane to climb before it
            # can descend. Instead: hold cruise altitude until close enough
            # that the glideslope naturally dips below it, then follow it down.
            cruise_alt_msl = (takeoff_elevation or 0) + CRUISE_ALTITUDE_M
            target_altitude = min(landing_runway["elevation_m"] + glide_alt_agl, cruise_alt_msl)

            # Vertical control uses the pitch-for-SPEED / power-for-PATH model.
            # On a stabilised approach the elevator flies AIRSPEED and the
            # throttle flies the GLIDEPATH -- the opposite of the intuitive
            # "pitch down to descend", which just trades height for speed and
            # balloons. Two decoupled loops instead of one control war.

            # ELEVATOR -> airspeed. Too fast: raise the nose to bleed speed.
            # Too slow: lower it. This keeps the wing flying instead of
            # mushing, wherever we are on the glidepath.
            speed_error = current_airspeed - APPROACH_SPEED_MS   # +ve = too fast
            pitch_target = max(-8, min(8, 0.5 * speed_error))
            pitch_error = pitch_target - current_pitch
            elevator = 0.04 * pitch_error - 0.015 * current_pitch_rate
            elevator = max(-0.4, min(0.4, elevator))
            send_elevator(elevator)

            # THROTTLE -> glidepath. Above the glideslope: pull power off and
            # sink toward it; below it: add power. A baseline trim setting
            # roughly holds the 3-degree slope at approach speed, so the plane
            # settles onto the path instead of diving at it. Idle here also
            # recovers a too-high approach without ever mushing.
            alt_error = current_altitude - target_altitude       # +ve = above path
            throttle = APPROACH_THROTTLE_TRIM - 0.01 * alt_error
            throttle = max(0.0, min(0.6, throttle))
            send_throttle(throttle)

            agl = current_altitude - landing_runway["elevation_m"]
            print(f"[APPROACH] DistRemaining:{distance_remaining:.0f}m AlongTrack:{along_track:.0f}m "
                  f"CrossTrack:{cross_track:.0f}m AGL:{agl:.0f} TargetAlt:{target_altitude:.0f} "
                  f"Spd:{current_airspeed:.0f} Pitch:{current_pitch:.1f} Roll:{current_roll:.1f} Elevator:{elevator:.2f} "
                  f"Throttle:{throttle:.2f} Hdg:{current_heading:.0f} HdgTarget:{desired_heading:.0f}")

            # GO-AROUND: if we've flown past the threshold (along_track turns
            # positive) while still well above flare altitude, we can't safely
            # land on this pass -- descending further this late just chases an
            # impossible target. Instead, climb away and let the existing
            # runway-selection logic re-attempt. Since find_nearest_runway_end
            # only picks ends where the aircraft is in that end's proper
            # approach corridor, and we're now on the far side of this runway,
            # it will naturally select the OPPOSITE end on the retry --
            # i.e. it comes back around and lands from the other direction.
            OVERSHOOT_MARGIN_M = 100.0
            GO_AROUND_SAFETY_AGL_M = FLARE_ALT_AGL_M + 50.0
            if along_track > OVERSHOOT_MARGIN_M and agl > GO_AROUND_SAFETY_AGL_M:
                print(f"Overshot runway at AGL:{agl:.0f} -- going around to re-attempt")
                faf_inserted = False
                landing_runway = None
                faf_point = None
                set_flaps(0.0)
                with phase_lock:
                    flight_phase = "ENROUTE"
                time.sleep(dt)
                continue

            if agl <= FLARE_ALT_AGL_M:
                print("Entering FLARE")
                with phase_lock:
                    flight_phase = "FLARE"

            time.sleep(dt)
            continue

        # ============================================================
        # PHASE: FLARE -- cut power, hold nose up, track runway heading
        # ============================================================
        if phase == "FLARE":
            send_throttle(0.0)  # throttle to idle for the flare -- not reverse

            pitch_error = FLARE_PITCH_DEG - current_pitch
            elevator1 = 0.04 * pitch_error - 0.015 * current_pitch_rate
            elevator2 = max(-0.2, min(0.2, elevator1))
            send_elevator(elevator2)

            # Keep correcting any remaining cross-track offset (gently) rather
            # than abruptly switching to pure fixed-heading -- avoids landing
            # meaningfully off centerline if APPROACH hadn't fully closed it.
            cross_track, _ = cross_track_and_along_track(
                current_lat, current_lon,
                landing_runway["lat"], landing_runway["lon"],
                landing_runway["heading"]
            )
            intercept = max(-10, min(10, -0.15 * cross_track))
            desired_heading = (landing_runway["heading"] + intercept) % 360
            heading_error = normalize_error(desired_heading - current_heading)
            roll_target = max(-10, min(10, 0.15 * heading_error))
            roll_error = roll_target - current_roll
            aileron1 = 0.03 * roll_error
            aileron2 = max(-0.3, min(0.3, aileron1))
            send_aileron(aileron2)

            rudder = max(-1.0, min(1.0, 0.02 * heading_error))
            send_rudder(rudder)

            agl = current_altitude - landing_runway["elevation_m"]
            print(f"[FLARE] AGL:{agl:.1f} GS:{current_groundspeed:.1f} CrossTrack:{cross_track:.0f}m "
                  f"OnGround:{current_onground:.0f}")

            if current_onground > 0.5:
                print("Touchdown -> LANDED phase")
                with phase_lock:
                    flight_phase = "LANDED"

            time.sleep(dt)
            continue

        # ============================================================
        # PHASE: LANDED -- rollout, braking, track runway heading
        # ============================================================
        if phase == "LANDED":
            send_throttle(-0.6)
            set_wheel_brakes(1.0)
            send_elevator(0.0)
            send_aileron(0.0)

            heading_error = normalize_error(landing_runway["heading"] - current_heading)
            rudder = max(-1.0, min(1.0, 0.02 * heading_error))
            send_rudder(rudder)

            print(f"[LANDED] GS:{current_groundspeed:.1f} m/s -- braking")

            if current_groundspeed <= LANDED_GROUNDSPEED_MS:
                print("Aircraft stopped. Autopilot task complete.")

            time.sleep(dt)
            continue

# ----------------------------
# WEB UI
# ----------------------------
app = Flask(__name__)

@app.route("/")
def index():
    return """
    <html>
    <head>
        <link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css"/>
        <style>
            body { margin:0; font-family: sans-serif; }
            #map { height: 100vh; }
            #panel {
                position: absolute; top: 10px; right: 10px; z-index: 1000;
                background: white; padding: 10px; border-radius: 6px;
                box-shadow: 0 1px 6px rgba(0,0,0,0.3); width: 220px;
            }
            #panel button { margin: 4px 4px 0 0; }
            #status { margin-top: 8px; font-size: 13px; white-space: pre-wrap; }
            #takeoffBtn { background:#c0392b; color:white; border:none; padding:8px; border-radius:4px; width:100%; }
            #killBtn { background:#000; color:#fff; border:2px solid #f00; padding:10px; border-radius:4px; width:100%; font-weight:bold; }
        </style>
    </head>
    <body>
        <div id="map"></div>
        <div id="panel">
            <button id="killBtn" onclick="killSwitch()">🛑 KILL SWITCH</button>
            <hr>
            <button id="takeoffBtn" onclick="startTakeoff()">Start Takeoff</button>
            <hr>
            <div>Click map to add waypoints (in order)</div>
            <label>Altitude (m): <input id="altInput" type="number" value="1500" style="width:80px"></label>
            <div style="margin-top:8px;">
                <button onclick="sendPath()">Send Path</button>
                <button onclick="clearPath()">Clear</button>
            </div>
            <div id="status"></div>
        </div>
        <script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>
        <script>
            var map = L.map('map').setView([51, 0], 5);
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);

            var waypoints = [];
            var markers = [];
            var line = null;

            function redrawLine() {
                if (line) map.removeLayer(line);
                var latlngs = waypoints.map(w => [w.lat, w.lon]);
                line = L.polyline(latlngs, {color: 'blue'}).addTo(map);
            }

            map.on('click', function(e) {
                var alt = parseFloat(document.getElementById('altInput').value) || 1500;
                var wp = {lat: e.latlng.lat, lon: e.latlng.lng, alt: alt};
                waypoints.push(wp);
                var marker = L.marker([wp.lat, wp.lon]).addTo(map).bindTooltip(String(waypoints.length));
                markers.push(marker);
                redrawLine();
            });

            function clearPath() {
                waypoints = [];
                markers.forEach(m => map.removeLayer(m));
                markers = [];
                if (line) map.removeLayer(line);
                fetch('/set_path', {
                    method: 'POST', headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({waypoints: []})
                });
                document.getElementById('status').innerText = "Path cleared";
            }

            function sendPath() {
                fetch('/set_path', {
                    method: 'POST', headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({waypoints: waypoints})
                }).then(r => r.json()).then(d => {
                    document.getElementById('status').innerText = "Path sent: " + waypoints.length + " waypoints";
                });
            }

            function startTakeoff() {
                if (!confirm("Confirm: engine running, aircraft lined up on runway, ready to go?")) return;
                fetch('/start_takeoff', {method: 'POST'})
                    .then(r => r.json())
                    .then(d => { document.getElementById('status').innerText = JSON.stringify(d); });
            }

            function killSwitch() {
                // No confirm dialog on purpose -- a kill switch needs to be instant.
                fetch('/kill_switch', {method: 'POST'})
                    .then(r => r.json())
                    .then(d => { document.getElementById('status').innerText = "KILLED: " + JSON.stringify(d); });
            }
        </script>
    </body>
    </html>
    """

@app.route("/kill_switch", methods=["POST"])
def kill_switch():
    global flight_phase
    with phase_lock:
        flight_phase = "KILLED"
    kill_manual_control()
    print("!!! KILL SWITCH TRIGGERED -- control released to pilot !!!")
    return jsonify({"status": "ok", "message": "Autopilot killed, manual control restored"})

@app.route("/start_takeoff", methods=["POST"])
def start_takeoff():
    global flight_phase, runway_heading
    with phase_lock:
        if flight_phase != "IDLE":
            return jsonify({"status": "error", "message": f"Already in phase {flight_phase}"})
        runway_heading = current_heading
        flight_phase = "TAKEOFF_ROLL"
    release_parking_brake()
    release_wheel_brakes()  # dataref values persist across script restarts --
                            # if a previous run reached LANDED, wheel brakes
                            # could still be locked at full, silently
                            # preventing any takeoff acceleration.
    enable_override()
    zero_elevator_trim()
    disengage_xplane_autopilot()
    print(f"Takeoff started. Runway heading locked at {runway_heading:.1f}")
    return jsonify({"status": "ok", "runway_heading": runway_heading})

@app.route("/set_path", methods=["POST"])
def set_path():
    global path, current_wp_index
    data = request.json
    waypoints = data.get("waypoints", [])
    with path_lock:
        path = [{"lat": w["lat"], "lon": w["lon"], "alt": w.get("alt", DEFAULT_TARGET_ALTITUDE)}
                for w in waypoints]
        current_wp_index = 0
    print(f"New path set: {len(path)} waypoints")
    return jsonify({"status": "ok", "count": len(path)})

@app.route("/status")
def status():
    with path_lock, phase_lock:
        return jsonify({
            "phase": flight_phase,
            "lat": current_lat, "lon": current_lon,
            "heading": current_heading, "altitude": current_altitude,
            "groundspeed": current_groundspeed, "onground": current_onground,
            "roll": current_roll, "pitch": current_pitch,
            "wp_index": current_wp_index, "path_length": len(path),
        })

# ----------------------------
# START
# ----------------------------
if __name__ == "__main__":
    airport_runways = load_airports(APT_DAT_PATH)
    request_data()
    threading.Thread(target=autopilot, daemon=True).start()
    app.run(port=5000)