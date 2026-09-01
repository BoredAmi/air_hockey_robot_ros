cmake_minimum_required(VERSION 3.8)
project(air_hockey_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/launch_puck.sh
  scripts/reset_puck.sh
  scripts/puck_control_gui.py
  scripts/egm_robot_sim.py
  scripts/camera_sim_bridge.py
  scripts/inject_gz_joint_control.py
  DESTINATION lib/${PROJECT_NAME})

# egm_robot_sim.py imports egm_pb2 from ../egm relative to its own
# installed location (lib/${PROJECT_NAME}/../egm = lib/egm) - keep this
# install destination in sync with that script's sys.path setup if it
# ever changes.
install(DIRECTORY egm DESTINATION lib)

ament_package()
