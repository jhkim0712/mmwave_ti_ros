import os
import launch
import launch_ros.actions
import pytest
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    my_package_dir = get_package_share_directory('ti_mmwave_rospkg')
    path = os.path.join(my_package_dir,'cfg','6432_Standard_Uncompressed.cfg')
    device = "6432"
    name = "/mmWaveCLI"
    command_port = "/dev/ttyACM0"
    command_rate = "115200"
    data_port = "/dev/ttyACM0"
    data_rate = "115200"

    ld = LaunchDescription()
    ConfigParameters = os.path.join(
        my_package_dir,
        'config',
        'global_params.yaml',
        'launch/*.rviz'
    )
    global_param_node = Node(
        package='ti_mmwave_rospkg',
        executable='ConfigParameterServer',
        name='ConfigParameterServer',
        parameters=[ConfigParameters]
    )

    mmWaveCommSrv = Node(
    package="ti_mmwave_rospkg",
    executable="mmWaveCommSrv",
    name="mmWaveCommSrv",
    output="screen",
    emulate_tty=True,
    parameters=[
        {"command_port": command_port},
        {"command_rate": command_rate},
        {"data_port": data_port},
        {"data_rate": data_rate},
        {"max_allowed_elevation_angle_deg": "90"},
        {"max_allowed_azimuth_angle_deg": "90"},
        {"frame_id": "/ti_mmwave_0"},
        {"mmwavecli_name": name},
        {"mmwavecli_cfg": path}
        ]
    )

    mmWaveQuickConfig = Node(
        package="ti_mmwave_rospkg",
        executable="mmWaveQuickConfig",
        name="mmWaveQuickConfig",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"mmwavecli_name": name},
            {"mmwavecli_cfg": path}
        ]
    )

    ParameterParser = Node(
        package="ti_mmwave_rospkg",
        executable="ParameterParser",
        name="ParameterParser",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"device_name": device},
            {"mmwavecli_name": name},
            {"mmwavecli_cfg": path}
        ]
    )

    delay = TimerAction(
        period=5.0,
        actions=[Node(
        package="ti_mmwave_rospkg",
        executable="DataHandlerClass",
        name="DataHandlerClass",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"mmwavecli_name": name},
            {"mmwavecli_cfg": path},
            {"data_port": data_port},
            {"data_rate": data_rate}
        ]
        ),
        Node(
             package='rviz2',
             executable='rviz2',
             arguments=['-d', os.path.join(my_package_dir, 'launch', 'rviz.rviz')]

        )]
    )

    ld.add_action(global_param_node)
    ld.add_action(mmWaveCommSrv)
    ld.add_action(mmWaveQuickConfig)
    ld.add_action(ParameterParser)
    ld.add_action(delay)

    return ld

