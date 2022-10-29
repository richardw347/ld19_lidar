import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="ld19_lidar",
                executable="ld19_node",
                name="ld19_node",
                output="screen",
                parameters=[
                    {"port": "/dev/ld19_lidar"},
                    {"frame_id": "laser"},
                    {"topic_name": "scan"},
                ],
            )
        ]
    )
