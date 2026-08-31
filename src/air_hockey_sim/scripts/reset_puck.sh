#!/usr/bin/env bash
# Reset the puck to a resting position and zero its velocity.
#
# Usage: reset_puck.sh [X Y]
#   X, Y   optional CAD-frame position in meters (same frame the table and
#          puck models are placed in - see air_hockey_table.sdf). Default
#          is the table center (-0.99, -0.5325). Z is set a few mm above
#          the play surface so the puck drops and settles cleanly; RPY is
#          reset to identity.
#
# Requires the sim to already be running (ros2 launch air_hockey_sim
# air_hockey_table.launch.py). Uses gz-sim's world "set_pose" service,
# which also zeroes the entity's velocity - confirmed by testing (no
# post-reset drift).
#
# Also clears any active push force first: if this runs while
# launch_puck.sh is still mid-flight (e.g. hitting Reset in the GUI right
# after a launch button), that force would otherwise keep shoving the
# puck right after the teleport, looking like the reset "didn't take" or
# the puck immediately drifting off again.
#
# Every `ign` call below is wrapped in `timeout` (some individual calls
# were seen, in testing, to occasionally take a couple seconds or hang -
# see launch_puck.sh's notes) so a single slow call can't wedge this
# script, and the GUI along with it, indefinitely.
set -euo pipefail

X="${1:--0.99}"
Y="${2:--0.5325}"
Z="0.017"
CALL_TIMEOUT=1

WORLD="air_hockey_world"
MODEL="puck"

timeout "${CALL_TIMEOUT}" ign topic -t "/world/${WORLD}/wrench/clear" -m ignition.msgs.Entity \
  -p "name: '${MODEL}', type: MODEL" || true

echo "Resetting '${MODEL}' to (${X}, ${Y}, ${Z})"

timeout "${CALL_TIMEOUT}" ign service -s "/world/${WORLD}/set_pose" \
  --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 2000 \
  --req "name: '${MODEL}', position: {x: ${X}, y: ${Y}, z: ${Z}}, orientation: {w: 1}" || true

# Clear again after the teleport too, in case a launch_puck.sh "set" that
# was still in flight lands after this reset's own clear above.
timeout "${CALL_TIMEOUT}" ign topic -t "/world/${WORLD}/wrench/clear" -m ignition.msgs.Entity \
  -p "name: '${MODEL}', type: MODEL" || true
