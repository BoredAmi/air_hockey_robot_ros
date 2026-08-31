import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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

    gui_arg = DeclareLaunchArgument(
        'control_gui', default_value='true',
        description='Also open the puck control panel (launch/reset buttons) alongside Gazebo.')
    egm_sim_arg = DeclareLaunchArgument(
        'egm_sim', default_value='true',
        description="Also run the EGM robot simulator, so pc_node's movement_node can drive the "
                    "placeholder paddle over the same protocol it'd use with the real ABB robot.")

    return LaunchDescription([
        gui_arg,
        egm_sim_arg,
        # Lets the world file's `<uri>model://air_hockey_table</uri>` /
        # `model://puck` includes resolve to this package's models/ dir.
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', models_path),
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
            condition=IfCondition(LaunchConfiguration('egm_sim')),
        ),
    ])
