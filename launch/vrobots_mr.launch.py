from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    package_dir = FindPackageShare('vrobots_mr').find('vrobots_mr')

    # Path to your Python script relative to the package root
    python_script_path = os.path.join(package_dir, 'vr_bridge.py')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', python_script_path],
            shell=True
        ),
        Node(
            package='vrobots_mr', 
            executable='mr', 
            name='multirotor_pubsub',  
            output='screen',             
        ),
    ])