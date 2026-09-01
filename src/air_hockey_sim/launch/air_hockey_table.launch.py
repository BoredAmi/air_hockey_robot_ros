import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('air_hockey_sim')
    world_path = os.path.join(share_dir, 'worlds', 'air_hockey_table.sdf')
    models_path = os.path.join(share_dir, 'models')
    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    control_gui_path = os.path.join(
        get_package_prefix('air_hockey_sim'), 'lib', 'air_hockey_sim', 'puck_control_gui.py')
    egm_sim_path = os.path.join(
        get_package_prefix('air_hockey_sim'), 'lib', 'air_hockey_sim', 'egm_robot_sim.py')
    camera_bridge_path = os.path.join(
        get_package_prefix('air_hockey_sim'), 'lib', 'air_hockey_sim', 'camera_sim_bridge.py')
    inject_joint_control_path = os.path.join(
        get_package_prefix('air_hockey_sim'), 'lib', 'air_hockey_sim', 'inject_gz_joint_control.py')
    # The URDF->SDF conversion (ros_gz_sim's `create`) resolves package://
    # mesh URIs to model://abb_irb1200_support/..., which gz-sim looks up by
    # searching GZ_SIM_RESOURCE_PATH for a subfolder literally named
    # abb_irb1200_support - so the *parent* of that package's share dir
    # (.../share, which contains share/abb_irb1200_support/meshes/...) needs
    # to be on the path too, not just air_hockey_sim's own models/ dir.
    abb_irb1200_resource_path = os.path.dirname(get_package_share_directory('abb_irb1200_support'))

    gui_arg = DeclareLaunchArgument(
        'control_gui', default_value='true',
        description='Also open the puck control panel (launch/reset buttons) alongside Gazebo.')
    egm_sim_arg = DeclareLaunchArgument(
        'egm_sim', default_value='true',
        description="Whether this machine's own egm_robot_sim.py plays the EGM robot-controller "
                    "role. Set true (default) for a single-machine setup - it stands in for the "
                    "real robot, teleporting the placeholder paddle to match. Set false when a "
                    "*separate* machine is providing the real robot side instead - e.g. RobotStudio "
                    "(see abb_ros2/robot_studio_resources/IRB1200_5_90.rspag) on another PC pointed "
                    "at this one's IP:6511, which MovementController::startEgmServer() already "
                    "accepts (it binds INADDR_ANY, not just localhost) with zero code changes. "
                    "Never run both at once - they'd both act as \"the robot\" and race each other "
                    "sending EgmRobot packets to the same movement_node.")
    camera_sim_arg = DeclareLaunchArgument(
        'camera_sim', default_value='true',
        description="Also bridge the simulated overhead camera onto /camera/raw_stream, so "
                    "pc_node's perception_node can run against it completely unmodified.")
    spawn_irb1200_arg = DeclareLaunchArgument(
        'spawn_irb1200', default_value='true',
        description="Spawn the real ABB IRB1200 URDF/mesh (from the abb_ros2/abb_irb1200_support "
                    "package cloned into src/) in place of the old placeholder paddle. Each joint "
                    "runs gz-sim's JointPositionController (PID, holds/tracks a commanded angle "
                    "against gravity - see inject_gz_joint_control.py); egm_robot_sim.py (when "
                    "egm_sim:=true) computes IK from the EGM-tracked position and drives it.")

    gz_image_topic = '/table_camera/image'
    ros_image_topic = '/table_camera/image'

    # Robot base placement: centered behind the band, at
    # MovementController::reachCircleCenter()'s own CAD position
    # (movement.hpp's BASE_TO_EDGE_OFFSET_MM=400 behind the band,
    # PHYSICAL_TABLE_HEIGHT/2 laterally centered) - physically where a real
    # robot would actually be mounted, rather than at robot-frame (0,0)'s
    # position (a table corner - geometrically valid too, but not where the
    # real robot sits). egm_robot_sim.py's IK targets are built relative to
    # this same reachCircleCenter point (see its own comment) so the world
    # origin/coordinate mapping established when this bug was first fixed
    # stays correct regardless of where the base itself is spawned.
    #
    # Yaw=pi flips robotX into CAD -X (reaching into the table); corner 1's
    # transform (this project's real config.json robot_origin_corner) has NO
    # corresponding Y flip - a reflection, not a rotation, which no single
    # yaw can reproduce alone. egm_robot_sim.py negates ry before feeding the
    # IK chain to supply that missing flip; see its own comment for the full
    # derivation.
    robot_base_x = 0.4
    robot_base_y = -0.5325
    robot_base_z = 0.0
    robot_base_yaw = '3.14159'

    irb1200_xacro_path = os.path.join(
        get_package_share_directory('abb_irb1200_support'), 'urdf', 'irb1200_5_90.xacro')
    # abb_ros2's own "simulation" mode never involves Gazebo physics at all
    # (it's ros2_control's mock_components/GenericSystem, meant for RViz-only
    # visualization), so the xacro itself holds nothing against gravity -
    # inject_gz_joint_control.py adds a real per-joint PID controller
    # (gz-sim's JointPositionController) instead of standing up a full
    # ros2_control + gz_ros2_control stack for joints nothing else needs to
    # command. Command substitution runs its resolved string through
    # shlex.split and subprocess.run WITHOUT a shell (no shell=True), so a
    # literal `|` here would just be passed to xacro as a bogus argument -
    # routing the whole pipeline through `bash -c "..."` (one shlex token,
    # quoted) gets it interpreted as a real pipe.
    robot_description = Command([
        'bash -c "xacro ', irb1200_xacro_path,
        ' use_fake_hardware:=true | ', inject_joint_control_path, '"',
    ])

    # egm_robot_sim.py (in its Gazebo-sync thread) publishes plain rclpy
    # Float64 messages on the ROS side of the joint command bridge - much
    # cheaper per-tick than the `ign topic`/`ign service` subprocess calls
    # used elsewhere in that script (which the script's own docstring already
    # notes are too slow for anything near EGM's real rate). The GZ-side
    # topic each JointPositionController actually listens on embeds a literal
    # "/0/" axis-index segment before cmd_pos (confirmed via the plugin's own
    # debug log: "Topic: [/model/irb1200/joint/joint_1/0/cmd_pos]") that ROS2
    # topic names can't represent (a bare numeric token between slashes is
    # invalid), so the two sides need different names - the simple CLI bridge
    # form can't do that, hence the YAML config below instead.
    irb1200_model_name = 'irb1200'
    joint_bridge_config = os.path.join(share_dir, 'config', 'irb1200_joint_bridge.yaml')

    return LaunchDescription([
        gui_arg,
        egm_sim_arg,
        camera_sim_arg,
        spawn_irb1200_arg,
        # Lets the world file's `<uri>model://air_hockey_table</uri>` /
        # `model://puck` includes resolve to this package's models/ dir.
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', models_path + os.pathsep + abb_irb1200_resource_path),
        SetEnvironmentVariable(
            'IGN_GAZEBO_RESOURCE_PATH', models_path + os.pathsep + abb_irb1200_resource_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={'gz_args': f'-r {world_path}'}.items(),
        ),
        ExecuteProcess(
            cmd=[control_gui_path],
            condition=IfCondition(LaunchConfiguration('control_gui')),
        ),
        ExecuteProcess(
            cmd=[egm_sim_path],
            # Python stdout is block-buffered (not line-buffered) once
            # piped through ExecuteProcess rather than a real TTY, so its
            # print() diagnostics (including whether IK setup succeeded)
            # could sit invisible in a buffer for a long time otherwise.
            additional_env={'PYTHONUNBUFFERED': '1'},
            condition=IfCondition(LaunchConfiguration('egm_sim')),
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 f'{gz_image_topic}@sensor_msgs/msg/Image[gz.msgs.Image'],
            condition=IfCondition(LaunchConfiguration('camera_sim')),
        ),
        ExecuteProcess(
            cmd=[camera_bridge_path, '--image-topic', ros_image_topic],
            condition=IfCondition(LaunchConfiguration('camera_sim')),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(LaunchConfiguration('spawn_irb1200')),
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', irb1200_model_name,
                '-x', str(robot_base_x), '-y', str(robot_base_y), '-z', str(robot_base_z),
                '-Y', robot_base_yaw,
            ],
            condition=IfCondition(LaunchConfiguration('spawn_irb1200')),
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 '--ros-args', '-p', f'config_file:={joint_bridge_config}'],
            condition=IfCondition(LaunchConfiguration('spawn_irb1200')),
        ),
    ])
