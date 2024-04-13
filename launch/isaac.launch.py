#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, Shutdown
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import Shutdown

def generate_launch_description():

    # Get the Isaac Sim python executable
    isaac_sim_path = os.getenv('ISAACSIM_PATH')
    isaac_sim_python = os.path.join(isaac_sim_path, 'python.sh')

    # Get the Isaac Sim configuration file to be launched
    files_path = os.path.join(get_package_share_directory('pegasus_isaac'), 'config')
    
    # Arguments to be passed to the launch file
    config_dir = DeclareLaunchArgument('config_dir', default_value=files_path, description='The directory where the configuration files are stored')
    config_arg = DeclareLaunchArgument('config', default_value='isaac_px4.py', description='The configuration to be launched by Isaac Sim')

    world = DeclareLaunchArgument('world', default_value='Warehouse', description='The world to be launched by Isaac Sim')

    # The process that will launch isaac sim
    process = ExecuteProcess(
        cmd=[isaac_sim_python, [LaunchConfiguration('config_dir'), '/', LaunchConfiguration('config')], LaunchConfiguration('world')],
        output='screen',
        shell=False,
        cwd=[files_path],
        on_exit=Shutdown())

    return LaunchDescription([
        config_dir,
        config_arg,
        world,
        process
    ])