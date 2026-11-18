import os
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    params = os.path.join(
        get_package_share_directory('cam_lid_fusion_precision_pkg'),
        'config', 'params.yaml'
    )
     
    listener_node =  Node(
        package='cam_lid_fusion_precision_pkg',
        executable='ransac_real_time',   
        name="cam_lid_fusion_precision_pkg",
        output="screen",
       parameters=[{'use_sim_time': True}]
       #parameters= [params]
    )

    sensor_link_stp = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sensor_link_stp",
        arguments=[ "0.1293", "-0.0109", "-0.0193","2.3856", "1.5542", "-2.3088", "camera_link", "lidar_link"] # checker1
        # arguments=[ "0.2007", "-0.0400" , "-0.0193" ,"-0.3646 ", "1.5515", " 1.2181", "pylon_camera", "lidar_top/os_lidar"] # checker2
        # arguments=[ "0.2636", "-0.0247", "-0.0310" ,"-2.9113","1.5653","-1.3437", "pylon_camera", "lidar_top/os_lidar"] # checker3
        # arguments=[ "0.1482" , "-0.0449", "-0.0525" ,"-3.1097 ","1.3135", "-1.5298", "pylon_camera", "lidar_top/os_lidar"] # checker4
        # arguments=[ "0.1118 ",  "-0.0350"," 0.0102" ,"-0.0205 ","1.2875 "," 1.5532", "pylon_camera", "lidar_top/os_lidar"] # checker5
        # arguments=["0.2084", " -0.0276 ", "-0.0573" ,"-3.1334 ","1.3765 " ,"-1.5635", "pylon_camera", "lidar_top/os_lidar"] # checker6
        # arguments=[" 0.1889 ","-0.0295 ","0.0281" ,"-0.0298" ,"1.3073 ","1.5561", "pylon_camera", "lidar_top/os_lidar"] # checker7
    )

    desk_segment_node =  Node(
        package='cam_lid_fusion_precision_pkg',
        executable='desk_segment',   
        name="desk_segment",
        output="screen",
       parameters=[{'use_sim_time': True}]
       #parameters= [params]
    )

    checker_detect_node = Node(
        package='cam_lid_fusion_precision_pkg',
        executable='checker_detect',   
        name="checker_detect",
        output="screen",
       parameters=[{'use_sim_time': True}]
       #parameters= [params]
    )

    point_in_polygon_node = Node(
        package='cam_lid_fusion_precision_pkg',
        executable='pts_in_polygon',   
        name="pts_in_polygon",
        output="screen",
       parameters=[{'use_sim_time': True}]
       #parameters= [params]
    )

    final_checker_spehre_fusion_node = Node(
        package='cam_lid_fusion_precision_pkg',
        executable='final_checker_sphere_fusion',   
        name="final_checker_sphere_fusion",
        output="screen",
       parameters=[{'use_sim_time': True}]
       #parameters= [params]
    )

    return launch.LaunchDescription([
        # listener_node,
        point_in_polygon_node,
        # final_checker_spehre_fusion_node,
        sensor_link_stp
        #desk_segment_node
        
    ])
   