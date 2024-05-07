import os
from ament_index_python.packages import get_package_prefix
import launch.substitutions
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, ExecuteProcess


def generate_launch_description():

    emit_shutdown_action = launch.actions.Shutdown(
        reason='launch is shutting down')

    config = os.path.join(
        get_package_prefix('neobotix_coordinator'),
        '..',
        '..',
        'src',
        'neobotix_coordinator',
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='neobotix_coordinator',
            executable='Coordinator',
            # name='Coordinator',
            output='screen',
            parameters=[config],
            on_exit=[LogInfo(
                msg=["Coordinator has stopped. Stopping everything..."]), emit_shutdown_action],
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'groot', 'Groot', 
                '--mode=monitor', 
                '--address=localhost', 
                '--publisher_port=1666', 
                '--server_port=1667', 
                '--autoconnect'],
            shell=False,
        ),
    ])
