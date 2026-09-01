#!/usr/bin/env python3

import argparse
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import warnings

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "egm"))
import egm_pb2  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
with warnings.catch_warnings():
    # ikpy warns about the two fixed (non-actuated) tool frames being
    # "active" per active_links_mask - harmless, they're explicitly masked
    # False below and never actuated.
    warnings.simplefilter("ignore")
    import ikpy.chain  # noqa: E402

# --- pc_node/Config defaults, used only if config.json can't be read ---
DEFAULT_PHYSICAL_TABLE_WIDTH_MM = 1980.0
DEFAULT_PHYSICAL_TABLE_HEIGHT_MM = 1065.0
DEFAULT_ROBOT_ORIGIN_CORNER = 0

PHYSICAL_TABLE_WIDTH_MM = DEFAULT_PHYSICAL_TABLE_WIDTH_MM
PHYSICAL_TABLE_HEIGHT_MM = DEFAULT_PHYSICAL_TABLE_HEIGHT_MM
ROBOT_ORIGIN_CORNER = DEFAULT_ROBOT_ORIGIN_CORNER

# Not in config.json (a compile-time C++ constant, not a runtime setting) -
# must be kept in sync with movement.hpp's own BASE_TO_EDGE_OFFSET_MM by
# hand. Used (with PHYSICAL_TABLE_HEIGHT_MM/2) to reproduce
# MovementController::reachCircleCenter() below, so the Gazebo base can be
# spawned there (centered behind the band, physically sensible) while IK
# targets still land on the correct absolute table position - see the
# REACH_CENTER_ROBOT comment where it's used.
BASE_TO_EDGE_OFFSET_MM = 400.0


def load_config(path):
    global PHYSICAL_TABLE_WIDTH_MM, PHYSICAL_TABLE_HEIGHT_MM, ROBOT_ORIGIN_CORNER
    try:
        with open(path) as f:
            cfg = json.load(f)
    except OSError as exc:
        print(f"[egm_robot_sim] Could not read {path} ({exc}) - using pc_node's hardcoded "
              f"Config defaults instead.", file=sys.stderr)
        return
    PHYSICAL_TABLE_WIDTH_MM = cfg.get("PHYSICAL_TABLE_WIDTH", DEFAULT_PHYSICAL_TABLE_WIDTH_MM)
    PHYSICAL_TABLE_HEIGHT_MM = cfg.get("PHYSICAL_TABLE_HEIGHT", DEFAULT_PHYSICAL_TABLE_HEIGHT_MM)
    ROBOT_ORIGIN_CORNER = cfg.get("robot_origin_corner", DEFAULT_ROBOT_ORIGIN_CORNER)
    print(f"[egm_robot_sim] Loaded {path}: PHYSICAL_TABLE_WIDTH={PHYSICAL_TABLE_WIDTH_MM} "
          f"PHYSICAL_TABLE_HEIGHT={PHYSICAL_TABLE_HEIGHT_MM} robot_origin_corner={ROBOT_ORIGIN_CORNER}")


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

# The countertop mesh (air_hockey_table's model.sdf, "10mm thick" comment) has
# its origin at its own BOTTOM face, so the actual play surface sits at world
# Z=0.01, not Z=0 - confirmed against the puck's own observed rest height
# (~0.0123, matching table-top 0.01 + the puck's own 2.5mm half-thickness,
# see puck/model.sdf). Feeding the IK chain a flat Z=0 target was therefore
# commanding the paddle 10mm INTO the solid countertop every cycle - a
# constant, forced penetration the stiff JointPositionController PID (see
# inject_gz_joint_control.py) fights continuously, which is what showed up as
# jitter/"pressure" against the table. +2mm above the true surface keeps the
# paddle just clear of it while still deeply overlapping the puck's own
# vertical extent (puck top ~0.0148) for real strike contact.
PADDLE_TARGET_Z_M = 0.012

# --- IRB1200 arm animation (see worlds/air_hockey_table.sdf's table_camera
# comment and the launch file's spawn_irb1200 arg for the spawn side) ---
IRB1200_MODEL_NAME = "irb1200"
IRB1200_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
# Indices into the ikpy chain: base_link (fixed), joint_1..6 (active), then
# two more fixed tool frames (joint_6-flange, link_6-tool0) - see
# irb1200_5_90_macro.xacro's link/joint list.
IK_ACTIVE_LINKS_MASK = [False, True, True, True, True, True, True, False, False]
# Tool pointing straight down (world/base -Z) - the paddle mounted on the
# flange is assumed to hang straight down onto the table, same as the flat
# "paddle" placeholder model always was.
IK_TARGET_ORIENTATION = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]


def build_ik_chain():
    """Processes the same xacro the launch file spawns (ignoring the
    JointPositionController plugins inject_gz_joint_control.py adds - ikpy
    only needs the kinematic structure) into a temp URDF file and loads it
    as an ikpy chain, rooted at base_link so the chain's local frame matches
    the robot frame this script already tracks position in (meters instead
    of mm)."""
    xacro_path = os.path.join(
        get_package_share_directory("abb_irb1200_support"), "urdf", "irb1200_5_90.xacro")
    urdf_xml = subprocess.run(
        ["xacro", xacro_path, "use_fake_hardware:=true"],
        capture_output=True, text=True, check=True, timeout=15,
    ).stdout
    with tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False) as f:
        f.write(urdf_xml)
        urdf_path = f.name
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path, base_elements=["base_link"], active_links_mask=IK_ACTIVE_LINKS_MASK)
    os.unlink(urdf_path)
    return chain


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


def gazebo_sync_loop(state, stop_event, ik_chain=None, joint_publishers=None):
    """When ik_chain/joint_publishers are given, solves IK for the current
    internal position and publishes the result to drive the spawned
    IRB1200's joints (which carries its own real paddle - see
    inject_gz_joint_control.py's ee_paddle). Otherwise falls back to
    teleporting the standalone placeholder paddle model, so there's still
    some visible paddle even if IK setup failed (see build_ik_chain's
    caller)."""
    ik_seed = None
    while not stop_event.is_set():
        with state.lock:
            rx, ry = state.robot_pos
        table_x, table_y = robot_to_table(rx, ry)
        cad_x, cad_y = table_mm_to_cad_m(table_x, table_y)
        if ik_chain is None:
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

        if ik_chain is not None:
            try:
                # Robot-frame mm -> ikpy chain-local meters, relative to
                # MovementController::reachCircleCenter() (movement.cpp:
                # (-BASE_TO_EDGE_OFFSET_MM, PHYSICAL_TABLE_HEIGHT/2)) rather
                # than robot-frame (0,0) directly, since the launch file
                # spawns the base AT reachCircleCenter's own CAD position
                # (centered behind the band - see its comment) instead of at
                # robot-frame (0,0)'s position (a table corner, geometrically
                # correct but not where a real robot would actually be
                # mounted). Subtracting it here keeps the base spawn free to
                # sit wherever's physically sensible while still landing
                # exactly on the real robot's own commanded position.
                #
                # Y is negated: TableToRobotCoordinates (movement.cpp) for
                # robot_origin_corner=1 (this project's real config.json
                # value) is robotX = WIDTH - table_x, robotY = table_y - an
                # X-axis flip with NO Y flip relative to the table frame.
                # That's a reflection, not a rotation, so no single spawn yaw
                # can reproduce it (a yaw rotates X and Y together); negating
                # ry here supplies the missing Y flip so the composed
                # transform matches TableToRobotCoordinates exactly.
                # Confirmed algebraically against table_mm_to_cad_m (matches
                # the standalone placeholder paddle's teleport target above,
                # which goes through the real robot_to_table/
                # table_mm_to_cad_m round trip rather than this shortcut).
                reach_center_x_mm = -BASE_TO_EDGE_OFFSET_MM
                reach_center_y_mm = PHYSICAL_TABLE_HEIGHT_MM / 2.0
                target = [(rx - reach_center_x_mm) / 1000.0,
                          -(ry - reach_center_y_mm) / 1000.0,
                          PADDLE_TARGET_Z_M]
                kwargs = {"initial_position": ik_seed} if ik_seed is not None else {}
                solution = ik_chain.inverse_kinematics(
                    target, IK_TARGET_ORIENTATION, orientation_mode="all", **kwargs)
                ik_seed = solution
                for i, joint_name in enumerate(IRB1200_JOINT_NAMES):
                    link_index = i + 1  # index 0 is the fixed base_link
                    pub = joint_publishers.get(joint_name)
                    if pub is not None:
                        msg = Float64()
                        msg.data = float(solution[link_index])
                        pub.publish(msg)
            except Exception as exc:  # noqa: BLE001 - never let this kill the EGM loop
                print(f"[egm_robot_sim] IK/joint publish failed: {exc}", file=sys.stderr)

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
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="config.json",
                         help="path to pc_node's config.json (default: config.json, resolved "
                              "relative to the current working directory)")
    parser.add_argument("--animate-arm", action="store_true", default=True,
                         help="solve IK and drive the spawned IRB1200's joints (default on)")
    parser.add_argument("--no-animate-arm", action="store_false", dest="animate_arm",
                         help="skip IK/joint animation (paddle placeholder teleport still happens)")
    args = parser.parse_args()
    load_config(args.config)

    start_table = (PHYSICAL_TABLE_WIDTH_MM, PHYSICAL_TABLE_HEIGHT_MM / 2.0)  # matches idleTablePosition()
    start_robot = table_to_robot(*start_table)
    state = SharedState(start_robot)
    target = start_robot

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(SAMPLE_RATE_S * 0.75)

    ik_chain = None
    joint_publishers = None
    ros_node = None
    if args.animate_arm:
        try:
            rclpy.init()
            ros_node = rclpy.create_node("egm_robot_sim_ik")
            joint_publishers = {
                name: ros_node.create_publisher(
                    # gz-sim's JointPositionController actually listens on a
                    # topic with a literal "/0/" axis-index segment before
                    # cmd_pos (confirmed via the plugin's own debug log), but
                    # ROS2 topic names reject a bare numeric token - so this
                    # ROS-side name (without /0/) is remapped to the real
                    # gz-side topic by config/irb1200_joint_bridge.yaml.
                    Float64, f"/model/{IRB1200_MODEL_NAME}/joint/{name}/cmd_pos", 10)
                for name in IRB1200_JOINT_NAMES
            }
            ik_chain = build_ik_chain()
            print("[egm_robot_sim] IK chain loaded, driving the spawned IRB1200's joints.")
        except Exception as exc:  # noqa: BLE001 - arm animation is optional, EGM must still run
            print(f"[egm_robot_sim] Could not set up IK/arm animation ({exc}) - continuing "
                  f"without it (paddle placeholder teleport still works).", file=sys.stderr)
            ik_chain = None

    stop_event = threading.Event()
    sync_thread = threading.Thread(
        target=gazebo_sync_loop, args=(state, stop_event, ik_chain, joint_publishers), daemon=True)
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
        if ros_node is not None:
            ros_node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
