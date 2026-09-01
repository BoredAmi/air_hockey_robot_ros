#!/usr/bin/env python3
"""Reads a URDF from stdin, bolts base_link to a fixed world frame, attaches
a real paddle collision body to tool0, and injects one gz-sim
JointPositionController <gazebo> plugin per revolute joint, writing the
result to stdout.

Replaces the old approach of marking the spawned robot <static>true</static>
(a fine fix while it was just a placeholder - see egm_robot_sim.py's history)
now that it's actually being driven: JointPositionController applies real
PID torque to reach/hold a commanded position against gravity, instead of
freezing the whole model rigid.

The world-fixing joint matters independently of the per-joint PID gains
below: nothing in this URDF ties base_link to anything static (a real
IRB1200 is bolted to a mounting plate; this one, on its own, is just a free
rigid body). Per-joint PID only controls angles BETWEEN links - it does
nothing to stop the whole assembly (base_link and everything attached to
it) from free-falling as a unit under gravity, since nothing anchors
base_link's own position/orientation in the world. This was the real cause
of it "falling over" even after the per-joint controllers were added and
tuned - a <initial_position> fix (see below) was necessary but insufficient
without this.

Gains are a first-pass estimate, not measured against the real robot: joint_2
and joint_3 carry the most gravitational torque (heaviest links, longest
moment arms - see irb1200_5_90_macro.xacro's link masses), so if the arm
still visibly droops or oscillates once actually driven, these are the first
thing to retune.

<initial_position> is set explicitly (0, matching the URDF's own zero-pose,
which stands fine unsupported - confirmed when this model was still spawned
<static>true</static>, before any active control existed) rather than left
to whatever gz-sim's undocumented default is: egm_robot_sim.py's first real
IK command doesn't arrive until a few seconds after spawn (xacro processing
+ ikpy chain loading + rclpy/ROS discovery all happen first), so if the
undocumented default were "apply zero effort until the first command"
rather than "hold position 0", gravity would get a multi-second head start
on an uncontrolled joint before any PID ever engages.

Usage:
    xacro irb1200_5_90.xacro use_fake_hardware:=true | inject_gz_joint_control.py
"""
import sys

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

PLUGIN_TEMPLATE = """<gazebo>
  <plugin filename="ignition-gazebo-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
    <joint_name>{joint}</joint_name>
    <initial_position>0</initial_position>
    <p_gain>3000</p_gain>
    <i_gain>200</i_gain>
    <d_gain>100</d_gain>
    <i_max>500</i_max>
    <i_min>-500</i_min>
    <cmd_max>3000</cmd_max>
    <cmd_min>-3000</cmd_min>
  </plugin>
</gazebo>
"""


WORLD_FIXING_JOINT = """<link name="world"/>
<joint name="world_to_base" type="fixed">
  <parent link="world"/>
  <child link="base_link"/>
</joint>
"""

# Radius/length/mass/friction/restitution match models/paddle/model.sdf
# exactly (the old teleported placeholder, now retired - see
# egm_robot_sim.py's history) - PADDLE_RADIUS_MM from pc_node/include/config.hpp.
# A primitive cylinder, not a mesh: dartsim only gives reliable collision +
# restitution for primitive-vs-primitive contact pairs (matches why the
# puck/table cushion collisions are primitives too - see air_hockey_table
# and puck model.sdf's notes). tool0's local +Z points toward world -Z (the
# IK target orientation makes the tool point straight down at the table -
# see egm_robot_sim.py's IK_TARGET_ORIENTATION), so offsetting the paddle
# along tool0's local -Z by half its length puts the paddle's flat bottom
# face at tool0's origin (i.e. right at the IK target height) with the rest
# of the disc extending back up and out of the way, matching how the old
# placeholder was positioned relative to its own teleported target point.
EE_PADDLE = """<link name="ee_paddle">
  <inertial>
    <mass value="0.3"/>
    <inertia ixx="0.00027" ixy="0" ixz="0" iyy="0.00027" iyz="0" izz="0.00036"/>
  </inertial>
  <collision name="ee_paddle_collision">
    <origin xyz="0 0 -0.03" rpy="0 0 0"/>
    <geometry><cylinder radius="0.049" length="0.06"/></geometry>
    <surface>
      <friction><ode><mu>0.01</mu><mu2>0.01</mu2></ode></friction>
      <bounce><restitution_coefficient>0.6</restitution_coefficient><threshold>0.01</threshold></bounce>
    </surface>
  </collision>
  <visual name="ee_paddle_visual">
    <origin xyz="0 0 -0.03" rpy="0 0 0"/>
    <geometry><cylinder radius="0.049" length="0.06"/></geometry>
    <material name=""><color rgba="0.1 0.3 0.8 1"/></material>
  </visual>
</link>
<joint name="tool0_to_paddle" type="fixed">
  <parent link="tool0"/>
  <child link="ee_paddle"/>
</joint>
"""


def main():
    urdf = sys.stdin.read()
    plugins = "".join(PLUGIN_TEMPLATE.format(joint=j) for j in JOINT_NAMES)
    urdf = urdf.replace("</robot>", WORLD_FIXING_JOINT + EE_PADDLE + plugins + "</robot>")
    sys.stdout.write(urdf)


if __name__ == "__main__":
    main()
