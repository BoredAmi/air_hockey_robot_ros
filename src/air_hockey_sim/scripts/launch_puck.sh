#!/usr/bin/env bash
# Give the puck an instant shove so you can test how it bounces off the
# cushion.
#
# Usage: launch_puck.sh VX VY
#   VX, VY   target velocity in m/s, in the table's CAD frame (same x/y
#            the table and puck models are placed in - see
#            air_hockey_table.sdf: table origin at one play-area corner,
#            x/y both negative across the play area).
#
# Requires the sim to already be running (ros2 launch air_hockey_sim
# air_hockey_table.launch.py) with the ApplyLinkWrench plugin loaded
# (declared in air_hockey_table.sdf).
#
# History, because the previous approach looked reasonable and wasn't:
# this used to apply a constant force via the *persistent*-wrench topic
# for a DURATION_S you passed in, then clear it (force = mass*v/duration).
# That's a real push spread over real time, and DURATION_S couldn't be
# made small: shrinking it below roughly 0.1-0.2s made the sequential
# "set" then "clear" `ign topic` calls unreliable in this environment
# (each call's own latency swamps a short sleep), and the clear would
# sometimes just never land - watched the puck accelerate clean off the
# table because of exactly that. And even when it worked, a real DURATION
# meant the force was often still pushing *after* the puck had already
# hit the cushion, fighting the bounce - visibly wrong.
#
# What actually gets a clean instant hit: the ApplyLinkWrench system's
# *instantaneous* wrench topic (not persistent) applies for a single
# physics step and needs no clear. Force is calibrated for exactly one
# step: F = mass * v / dt, with dt = the world's max_step_size (0.001s -
# see air_hockey_table.sdf's <physics>; if that ever changes, DT below
# needs to match it or the delivered speed will be off). One `ign topic`
# call, no sleep, nothing left over to interfere with the bounce -
# verified reliable across repeated trials and against the puck's actual
# logged trajectory (smooth constant-velocity coast into the cushion,
# clean rebound, no residual push).
set -euo pipefail

VX="${1:?usage: launch_puck.sh VX VY}"
VY="${2:?usage: launch_puck.sh VX VY}"
CALL_TIMEOUT=2
DT=0.001

WORLD="air_hockey_world"
MODEL="puck"
MASS="0.025"

FX=$(python3 -c "print(${MASS}*${VX}/${DT})")
FY=$(python3 -c "print(${MASS}*${VY}/${DT})")

echo "Hitting '${MODEL}' with a single-step impulse (force ${FX}, ${FY} N over ${DT}s) -> target velocity (${VX}, ${VY}) m/s"

timeout "${CALL_TIMEOUT}" ign topic -t "/world/${WORLD}/wrench" -m ignition.msgs.EntityWrench \
  -p "entity: {name: '${MODEL}', type: MODEL}, wrench: {force: {x: ${FX}, y: ${FY}}}" || true

echo "Done."
