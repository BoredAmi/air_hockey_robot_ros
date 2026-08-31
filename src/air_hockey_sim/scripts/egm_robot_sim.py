#!/usr/bin/env python3
"""Stand-in for the real ABB robot controller's EGM (Externally Guided
Motion) side, so pc_node's movement_node/MovementController can be run
and tested against the simulator without the real robot.

Protocol background (see src/pc_node/egm/egm.proto and
src/pc_node/src/movement.cpp): MovementController is the *sensor* side -
it binds UDP port 6511 and waits. The *robot controller* is the one that
initiates: it periodically sends EgmRobot (feedback) packets to that
port, and MovementController replies, to whichever address each packet
came from, with an EgmSensor (target/correction) packet. This script
plays the robot controller's role: it sends EgmRobot at the same 4ms
cadence the real robot's EGM.mod configures (\\SampleRate:=4), and moves
an internal position estimate toward whatever target the last EgmSensor
reply asked for, rate-limited to 3000mm/s (matching EGM.mod's
\\maxspeeddeviation:=3000) rather than teleporting - so a real
movement_node's rate-limiting/timing-sensitive logic (attack sequencing,
strike lead time, etc.) sees something reasonably close to how the real
robot behaves.

That internal position is also periodically pushed into Gazebo (as the
pose of the "paddle" placeholder model - see models/paddle/) so you can
watch it move. That part is a visual approximation, not physically
simulated robot dynamics: the paddle model gets *teleported* via the
world's set_pose service at a modest rate (these calls have real,
sometimes-slow latency in this environment - see launch_puck.sh's notes
on `ign` CLI call latency - so this can't run anywhere near the 250Hz of
the EGM loop itself). It'll shove the puck if it overlaps it, but this
isn't a substitute for a properly joint-controlled robot arm - update
this once the real robot model/control is added.

Coordinates: EGM speaks millimeters in the robot's own frame (see
MovementController::TableToRobotCoordinates/RobotToTableCoordinates for
the table-frame<->robot-frame mapping this mirrors). The Gazebo world
speaks meters in the CAD frame the table model is built in (see
air_hockey_table.sdf's notes: cad_x = (table_frame_x - WIDTH)/1000,
cad_y = (table_frame_y - HEIGHT)/1000). This script bridges all three.

Table dimensions, ROBOT_ORIGIN_CORNER, and PADDLE speed all default to
matching pc_node's Config defaults (see src/pc_node/include/config.hpp) -
there's no config.json in this repo, so those defaults are what
movement_node actually runs with today. Update the constants below if
that changes.

Usage:
    ros2 run air_hockey_sim egm_robot_sim.py
    # or directly:
    python3 egm_robot_sim.py
"""
import os
import socket
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "egm"))
import egm_pb2  # noqa: E402

# --- pc_node/Config defaults (config.json doesn't exist in this repo) ---
PHYSICAL_TABLE_WIDTH_MM = 1980.0
PHYSICAL_TABLE_HEIGHT_MM = 1065.0
ROBOT_ORIGIN_CORNER = 0

# --- EGM wire parameters (must match robot/EGM.mod) ---
PC_HOST = "127.0.0.1"
PC_PORT = 6511          # MovementController::startEgmServer()'s bind port
SAMPLE_RATE_S = 0.004   # EGMActPose \SampleRate:=4 (ms)
MAX_SPEED_MM_S = 3000.0  # EGMRunPose \maxspeeddeviation:=3000

# --- Gazebo visualization sync ---
WORLD = "air_hockey_world"
PADDLE_MODEL = "paddle"
GAZEBO_SYNC_PERIOD_S = 0.15
GAZEBO_CALL_TIMEOUT_S = 1


def table_to_robot(table_x, table_y):
    """Mirrors MovementController::TableToRobotCoordinates (movement.cpp)."""
    if ROBOT_ORIGIN_CORNER == 0:
        return table_y, table_x
    if ROBOT_ORIGIN_CORNER == 1:
        return PHYSICAL_TABLE_WIDTH_MM - table_x, table_y
    if ROBOT_ORIGIN_CORNER == 2:
        return table_x, PHYSICAL_TABLE_HEIGHT_MM - table_y
    if ROBOT_ORIGIN_CORNER == 3:
        return table_y, PHYSICAL_TABLE_WIDTH_MM - table_x
    return table_y, table_x


def robot_to_table(robot_x, robot_y):
    """Mirrors MovementController::RobotToTableCoordinates (movement.cpp)."""
    if ROBOT_ORIGIN_CORNER == 0:
        return robot_y, robot_x
    if ROBOT_ORIGIN_CORNER == 1:
        return PHYSICAL_TABLE_WIDTH_MM - robot_x, robot_y
    if ROBOT_ORIGIN_CORNER == 2:
        return robot_x, PHYSICAL_TABLE_HEIGHT_MM - robot_y
    if ROBOT_ORIGIN_CORNER == 3:
        return PHYSICAL_TABLE_WIDTH_MM - robot_y, PHYSICAL_TABLE_HEIGHT_MM - robot_x
    return robot_y, robot_x


def table_mm_to_cad_m(table_x, table_y):
    """See air_hockey_table.sdf's marker-position notes for this same mapping."""
    return (table_x - PHYSICAL_TABLE_WIDTH_MM) / 1000.0, (table_y - PHYSICAL_TABLE_HEIGHT_MM) / 1000.0


class SharedState:
    def __init__(self, initial_robot_pos):
        self.lock = threading.Lock()
        self.robot_pos = initial_robot_pos  # (x, y) mm, robot frame


def gazebo_sync_loop(state, stop_event):
    """Periodically teleports the paddle model to the current internal
    position - see module docstring for why this is a visual
    approximation, not real robot dynamics."""
    while not stop_event.is_set():
        with state.lock:
            rx, ry = state.robot_pos
        table_x, table_y = robot_to_table(rx, ry)
        cad_x, cad_y = table_mm_to_cad_m(table_x, table_y)
        try:
            subprocess.run(
                ["timeout", str(GAZEBO_CALL_TIMEOUT_S), "ign", "service",
                 "-s", f"/world/{WORLD}/set_pose",
                 "--reqtype", "ignition.msgs.Pose",
                 "--reptype", "ignition.msgs.Boolean",
                 "--timeout", "2000",
                 "--req", f"name: '{PADDLE_MODEL}', "
                          f"position: {{x: {cad_x}, y: {cad_y}, z: 0.04}}, "
                          f"orientation: {{w: 1}}"],
                capture_output=True, timeout=GAZEBO_CALL_TIMEOUT_S + 0.5,
            )
        except Exception as exc:  # noqa: BLE001 - never let this kill the EGM loop
            print(f"[egm_robot_sim] paddle sync failed: {exc}", file=sys.stderr)
        stop_event.wait(GAZEBO_SYNC_PERIOD_S)


def rate_limit(current, target, max_step):
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    dist = (dx * dx + dy * dy) ** 0.5
    if dist <= max_step or dist == 0.0:
        return target
    scale = max_step / dist
    return (current[0] + dx * scale, current[1] + dy * scale)


def main():
    start_table = (PHYSICAL_TABLE_WIDTH_MM, PHYSICAL_TABLE_HEIGHT_MM / 2.0)  # matches idleTablePosition()
    start_robot = table_to_robot(*start_table)
    state = SharedState(start_robot)
    target = start_robot

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(SAMPLE_RATE_S * 0.75)

    stop_event = threading.Event()
    sync_thread = threading.Thread(target=gazebo_sync_loop, args=(state, stop_event), daemon=True)
    sync_thread.start()

    print(f"[egm_robot_sim] Sending EgmRobot to {PC_HOST}:{PC_PORT} every {SAMPLE_RATE_S*1000:.0f}ms, "
          f"starting at robot=({start_robot[0]:.1f}, {start_robot[1]:.1f})mm "
          f"table=({start_table[0]:.1f}, {start_table[1]:.1f})mm")

    seqno = 1
    last_log = time.monotonic()
    try:
        while True:
            tick_start = time.monotonic()

            with state.lock:
                current = state.robot_pos

            robot_msg = egm_pb2.EgmRobot()
            robot_msg.header.seqno = seqno
            robot_msg.header.tm = int(tick_start * 1000) & 0xFFFFFFFF
            robot_msg.header.mtype = egm_pb2.EgmHeader.MSGTYPE_DATA
            robot_msg.feedBack.cartesian.pos.x = current[0]
            robot_msg.feedBack.cartesian.pos.y = current[1]
            robot_msg.feedBack.cartesian.pos.z = 0.0
            robot_msg.feedBack.cartesian.orient.u0 = 1.0
            robot_msg.feedBack.cartesian.orient.u1 = 0.0
            robot_msg.feedBack.cartesian.orient.u2 = 0.0
            robot_msg.feedBack.cartesian.orient.u3 = 0.0
            robot_msg.motorState.state = egm_pb2.EgmMotorState.MOTORS_ON
            robot_msg.mciState.state = egm_pb2.EgmMCIState.MCI_RUNNING
            robot_msg.rapidExecState.state = egm_pb2.EgmRapidCtrlExecState.RAPID_RUNNING
            seqno += 1

            sock.sendto(robot_msg.SerializeToString(), (PC_HOST, PC_PORT))

            try:
                data, _addr = sock.recvfrom(4096)
                sensor_msg = egm_pb2.EgmSensor()
                sensor_msg.ParseFromString(data)
                if sensor_msg.HasField("planned") and sensor_msg.planned.HasField("cartesian") \
                        and sensor_msg.planned.cartesian.HasField("pos"):
                    p = sensor_msg.planned.cartesian.pos
                    target = (p.x, p.y)
            except socket.timeout:
                pass

            max_step = MAX_SPEED_MM_S * SAMPLE_RATE_S
            new_pos = rate_limit(current, target, max_step)
            with state.lock:
                state.robot_pos = new_pos

            if tick_start - last_log >= 0.5:
                last_log = tick_start
                tx, ty = robot_to_table(*new_pos)
                print(f"[egm_robot_sim] robot=({new_pos[0]:.1f}, {new_pos[1]:.1f})mm "
                      f"table=({tx:.1f}, {ty:.1f})mm target=({target[0]:.1f}, {target[1]:.1f})mm")

            elapsed = time.monotonic() - tick_start
            remaining = SAMPLE_RATE_S - elapsed
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        sync_thread.join(timeout=2)
        sock.close()


if __name__ == "__main__":
    main()
