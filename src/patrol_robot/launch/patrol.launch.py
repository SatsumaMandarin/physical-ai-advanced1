import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('patrol_robot')
    world = os.path.join(pkg, 'worlds', 'patrol_room.sdf')
    bridge_config = os.path.join(pkg, 'config', 'bridge.yaml')
    patrol_config = os.path.join(pkg, 'config', 'patrol.yaml')
    detector_config = os.path.join(pkg, 'config', 'detector.yaml')

    # model://patrol_bot などを見つけられるようにする
    resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models'))

    # Gazebo（-r：起動と同時にシミュレーションを開始）
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world}'}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen',
    )

    patrol = Node(
        package='patrol_robot',
        executable='patrol_node',
        parameters=[patrol_config, {'use_sim_time': True}],
        output='screen',
    )

    detector = Node(
        package='patrol_robot',
        executable='detector_recorder_node',
        parameters=[detector_config, {'use_sim_time': True}],
        output='screen',
    )

    viewer = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/detections/image'],
        output='screen',
    )

    return LaunchDescription([
        resource_path,
        gazebo,
        bridge,
        patrol,
        detector,
        viewer,
    ])
