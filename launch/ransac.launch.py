import os
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    sensor_link_stp = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sensor_link_stp",
        arguments=[ "0.1293", "-0.0109", "-0.0193","2.3856", "1.5542", "-2.3088", "camera_link", "lidar_link"]
    )

    point_in_polygon_node = Node(
        package='camera_lidar_fusion_precision',
        executable='pts_in_polygon',   
        name="pts_in_polygon",
        output="screen",
       parameters=[{'use_sim_time': False}]
    )

    return launch.LaunchDescription([
        point_in_polygon_node,
        sensor_link_stp
    ])
   