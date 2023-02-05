import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf_file_name = 'aaa.urdf'
    urdf = os.path.join(
        get_package_share_directory('aaa'),
        'urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    params = {'robot_description': robot_desc}

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[params],
    )

    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz_config_file = get_package_share_directory(
        'aaa') + '/rviz/rviz.rviz'

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    pub = Node(
        package='publisher',
        executable='publisher',
    )

    srv = Node(
        package='server',
        executable='server',
    )

    return LaunchDescription([srv, pub, rsp, jsp, rviz])
