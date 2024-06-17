from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    params = join(
        get_package_share_directory('pillar_based_removal_py'), 'params', 'param.yaml'
    )
    bag_directory = LaunchConfiguration('bag_directory', default='/root/pillar_based_removal/data/2011_09_26_drive_0002_sync')

    pillar_based_removal_node = Node(
        package='pillar_based_removal_py',
        executable='pillar_based_removal_node',
        name='pillar_based_removal_node',
        output='screen',
        parameters=[params]
    )

    playback = ExecuteProcess(
        cmd=[[
            "ros2 ",
            "bag ",
            "play ",
            bag_directory,
            " -l ",
            "--clock "
        ]],
        shell=True
    )

    return LaunchDescription([
        pillar_based_removal_node,
        playback
    ])