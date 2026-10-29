"""
Autonomous No-GPS Mission Script - v2
=======================================
Sequence: ARM -> WAIT 10s -> TAKEOFF to 2m -> hold -> FORWARD 5m -> hold
          -> RIGHT 5m -> hold -> RTL

New in v2 (based on real-world design requirements):
  - Optional RC-channel trigger to start the mission (simulates a
    transmitter toggle switch - test with MAVProxy's `rc` command)
  - Refuses to start if the vehicle is already armed (safety)
  - Holds/waits at each waypoint after confirming arrival, before
    moving to the next command
  - Uses RTL (Return To Launch) instead of LAND at the end

NOTE: No mode-change / manual-override watchdog is included by design.
The abort safety path is the RC-to-Pixhawk hardware switch (RTL/LAND
assigned via RCx_OPTION on an AUX channel), which acts independently
of this script and the Raspberry Pi entirely - see the setup guide
for configuring that in Mission Planner.
"""

import time
import math
from pymavlink import mavutil

# ----------------------------------------------------------------------
# CONFIGURATION
# ----------------------------------------------------------------------

CONNECTION_STRING = "udpin:0.0.0.0:14553"

TARGET_ALTITUDE_M = 2.0
FORWARD_DISTANCE_M = 5.0
RIGHT_DISTANCE_M = 5.0
ARM_WAIT_SECONDS = 3               # kept short - ArduCopter auto-disarms
                                    # after a period of zero throttle
                                    # input following arming
WAYPOINT_HOLD_SECONDS = 5

ALTITUDE_TOLERANCE_M = 0.4         # widened - was too tight, causing
                                    # unnecessary timeouts
POSITION_TOLERANCE_M = 0.5
COMMAND_TIMEOUT_S = 30

# RC trigger settings (set USE_RC_TRIGGER = False to skip and start
# immediately instead - useful while you don't have a transmitter
# hooked into the test yet)
USE_RC_TRIGGER = False
TRIGGER_CHANNEL = 7                # e.g. channel 7 on your transmitter
TRIGGER_PWM_THRESHOLD = 1800       # switch counts as "ON" above this


# ----------------------------------------------------------------------
# CONNECTION HELPERS
# ----------------------------------------------------------------------

def connect_vehicle(connection_string):
    print(f"[CONNECT] Connecting to {connection_string} ...")
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print(f"[CONNECT] Heartbeat received. System={master.target_system}, "
          f"Component={master.target_component}")

    print("[CONNECT] Requesting LOCAL_POSITION_NED stream at 4Hz ...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        250000,
        0, 0, 0, 0, 0
    )
    time.sleep(1)

    return master


def is_armed(master):
    hb = master.messages.get('HEARTBEAT')
    if hb is None:
        return False
    return (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0


# ----------------------------------------------------------------------
# EKF ORIGIN
# ----------------------------------------------------------------------

def set_ekf_origin(master, lat, lon, alt_m):
    print("[EKF] Setting EKF origin ...")
    master.mav.set_gps_global_origin_send(
        master.target_system,
        int(lat * 1e7),
        int(lon * 1e7),
        int(alt_m * 1000)
    )
    time.sleep(1)


# ----------------------------------------------------------------------
# RC TRIGGER (simulates a transmitter toggle switch starting the mission)
# ----------------------------------------------------------------------

def wait_for_rc_trigger(master, channel, pwm_threshold):
    """
    Blocks until the given RC channel's value crosses the threshold.
    In SITL, simulate this from the MAVProxy console with:
        rc 7 2000      (turn ON)
        rc 7 1000      (turn OFF)
    """
    print(f"[TRIGGER] Waiting for RC channel {channel} > {pwm_threshold} "
          f"to start mission ...")
    while True:
        msg = master.recv_match(type='RC_CHANNELS', blocking=True, timeout=5)
        if msg is None:
            continue
        # channel N's value is in the field 'chanN_raw'
        value = getattr(msg, f'chan{channel}_raw', None)
        if value is not None and value > pwm_threshold:
            print(f"[TRIGGER] Channel {channel} = {value}. Starting mission!")
            return
        time.sleep(0.2)


# ----------------------------------------------------------------------
# MODE + ARMING
# ----------------------------------------------------------------------

def set_mode(master, mode_name):
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"[MODE] Requested mode: {mode_name}")


def refuse_if_already_armed(master):
    """Safety check: never start a mission on a vehicle that's already flying."""
    master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    if is_armed(master):
        raise RuntimeError(
            "ABORT: Vehicle is already armed. This script will only start "
            "a mission from a disarmed state. Land/disarm manually first."
        )
    print("[SAFETY] Confirmed vehicle is disarmed. Safe to proceed.")


def arm_and_confirm(master):
    print("[ARM] Sending arm command ...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    start = time.time()
    while time.time() - start < COMMAND_TIMEOUT_S:
        master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if is_armed(master):
            print("[ARM] Confirmed: vehicle is armed.")
            return True
        time.sleep(0.5)
    raise RuntimeError("Arming timed out - check pre-arm checks.")


# ----------------------------------------------------------------------
# POSITION READING
# ----------------------------------------------------------------------

def get_local_position(master):
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=2)
    if msg is None:
        return None
    return (msg.x, msg.y, msg.z)


# ----------------------------------------------------------------------
# TAKEOFF
# ----------------------------------------------------------------------

def takeoff_and_confirm(master, target_alt_m):
    print(f"[TAKEOFF] Commanding takeoff to {target_alt_m} m ...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_alt_m
    )

    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack is not None:
        print(f"[TAKEOFF] Command ACK result: {ack.result} "
              f"(0 = ACCEPTED, anything else = check mavlink COMMAND_ACK docs)")
        if ack.result != 0:
            raise RuntimeError(
                f"Takeoff command was REJECTED by the flight controller "
                f"(result={ack.result}). Check pre-arm status / mode / EKF."
            )
    else:
        print("[TAKEOFF] WARNING: no COMMAND_ACK received - proceeding anyway.")

    start = time.time()
    while time.time() - start < COMMAND_TIMEOUT_S:
        pos = get_local_position(master)
        if pos is not None:
            current_alt = -pos[2]
            print(f"[TAKEOFF] Current altitude: {current_alt:.2f} m")
            if abs(current_alt - target_alt_m) <= ALTITUDE_TOLERANCE_M:
                print("[TAKEOFF] Target altitude reached.")
                return True
        time.sleep(0.5)
    raise RuntimeError("Takeoff timed out before reaching target altitude.")


# ----------------------------------------------------------------------
# MOVEMENT (with hold-at-waypoint and abort detection)
# ----------------------------------------------------------------------

def get_yaw(master):
    """Current heading in radians (0 = North), from the ATTITUDE message."""
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    if msg is None:
        return 0.0
    return msg.yaw


def move_body_relative(master, forward_m, right_m, hold_alt_m):
    """
    Moves the drone forward_m/right_m relative to its CURRENT heading,
    by converting that into a FIXED North/East/Down target computed once
    up front - then repeatedly sending that same fixed target in
    MAV_FRAME_LOCAL_NED (a world-anchored frame).

    IMPORTANT: we deliberately do NOT use MAV_FRAME_BODY_OFFSET_NED with
    repeated sends here. That frame redefines "target" relative to the
    drone's CURRENT position on every single message - since we must
    stream setpoints continuously, that turns into a "moving carrot"
    the drone can never catch, causing runaway distance/altitude and
    curved flight paths. Computing the target once, in a fixed frame,
    avoids this entirely. (This is especially important for no-GPS
    flight, where the EKF position estimate is already noisier - we
    don't want to compound that with a moving target too.)
    """
    start_pos = get_local_position(master)
    if start_pos is None:
        raise RuntimeError("Could not read starting position.")
    start_n, start_e, _ = start_pos

    yaw = get_yaw(master)
    print(f"[MOVE] Current heading: {math.degrees(yaw):.1f} deg")

    target_n = start_n + forward_m * math.cos(yaw) - right_m * math.sin(yaw)
    target_e = start_e + forward_m * math.sin(yaw) + right_m * math.cos(yaw)
    target_d = -hold_alt_m

    target_distance = math.sqrt(forward_m**2 + right_m**2)
    print(f"[MOVE] Moving forward={forward_m}m, right={right_m}m "
          f"-> fixed target N={target_n:.2f}, E={target_e:.2f} "
          f"(target distance {target_distance:.2f}m) ...")

    start_time = time.time()
    last_send = 0

    while time.time() - start_time < COMMAND_TIMEOUT_S:
        if time.time() - last_send > 0.25:
            master.mav.set_position_target_local_ned_send(
                0,
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,
                target_n, target_e, target_d,
                0, 0, 0,
                0, 0, 0,
                0, 0
            )
            last_send = time.time()

        pos = get_local_position(master)
        if pos is not None:
            n, e, _ = pos
            remaining = math.sqrt((target_n - n)**2 + (target_e - e)**2)
            print(f"[MOVE] Distance remaining to target: {remaining:.2f} m")
            if remaining <= POSITION_TOLERANCE_M:
                print("[MOVE] Target position reached.")
                print(f"[HOLD] Waiting {WAYPOINT_HOLD_SECONDS}s at waypoint ...")
                time.sleep(WAYPOINT_HOLD_SECONDS)
                return True

        time.sleep(0.2)

    raise RuntimeError("Movement timed out before reaching target displacement.")


# ----------------------------------------------------------------------
# RETURN TO LAUNCH (instead of plain land)
# ----------------------------------------------------------------------

def rtl_and_confirm(master):
    print("[RTL] Commanding Return To Launch ...")
    set_mode(master, "RTL")
    start = time.time()
    while time.time() - start < COMMAND_TIMEOUT_S * 2:  # RTL can take longer
        master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if not is_armed(master):
            print("[RTL] Confirmed: vehicle has landed and disarmed.")
            return True
        time.sleep(0.5)
    raise RuntimeError("RTL timed out - vehicle may still be armed/airborne.")


# ----------------------------------------------------------------------
# MAIN MISSION SEQUENCE
# ----------------------------------------------------------------------

def run_mission():
    master = connect_vehicle(CONNECTION_STRING)

    # SAFETY: refuse to run if already armed
    refuse_if_already_armed(master)

    set_ekf_origin(master, lat=-35.363261, lon=149.165230, alt_m=0)

    set_mode(master, "GUIDED")
    time.sleep(1)

    if USE_RC_TRIGGER:
        wait_for_rc_trigger(master, TRIGGER_CHANNEL, TRIGGER_PWM_THRESHOLD)

    arm_and_confirm(master)

    print(f"[WAIT] Holding for {ARM_WAIT_SECONDS} seconds before takeoff ...")
    time.sleep(ARM_WAIT_SECONDS)

    takeoff_and_confirm(master, TARGET_ALTITUDE_M)

    move_body_relative(master, forward_m=FORWARD_DISTANCE_M, right_m=0,
                        hold_alt_m=TARGET_ALTITUDE_M)

    move_body_relative(master, forward_m=0, right_m=RIGHT_DISTANCE_M,
                        hold_alt_m=TARGET_ALTITUDE_M)

    rtl_and_confirm(master)

    print("[MISSION] Complete.")


if __name__ == "__main__":
    run_mission()
