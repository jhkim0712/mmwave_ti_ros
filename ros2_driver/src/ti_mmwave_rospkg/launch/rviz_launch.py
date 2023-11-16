import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Enter Path and Name Here
    my_package_dir = get_package_share_directory('ti_mmwave_rospkg')

    Rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(my_package_dir, 'launch', 'rviz.rviz')]
    )

    ld = LaunchDescription([
        Rviz2
    ])

    return ld