cmake_minimum_required(VERSION 3.5)
project(camera_lidar_fusion_precision)

## Compile as C++17, supported in ROS2
set(CMAKE_CXX_STANDARD 17)

## Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(image_geometry REQUIRED)
find_package(message_filters REQUIRED)
#find_package(mrs_lib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(message_filters REQUIRED)
#find_package(PCL REQUIRED COMPONENTS common filters)

## Declare dependencies
#ament_export_dependencies(talker pcl_conversions rclcpp roscpp rospy sensor_msgs std_msgs pcl_ros)

## Declare dependencies
set(dependencies
rclcpp
std_msgs
sensor_msgs
pcl_conversions
pcl_ros
cv_bridge
OpenCV
image_transport
tf2
tf2_eigen
image_geometry
message_filters
tf2_ros
tf2_geometry_msgs
geometry_msgs
PCL
message_filters
)

## Specify additional locations of header files

#include_directories(
# include
# ${PCL_INCLUDE_DIRS}
#)

## Declare a C++ executable

add_executable(ransac_real_time src/ransac_node.cpp)
ament_target_dependencies(ransac_real_time ${dependencies})

add_executable(desk_segment src/desk_segment2.cpp)
ament_target_dependencies(desk_segment ${dependencies})

add_executable(checker_detect src/checkerboard_detection.cpp)
ament_target_dependencies(checker_detect ${dependencies})

add_executable(pts_in_polygon src/points_in_polygon.cpp)
ament_target_dependencies(pts_in_polygon ${dependencies})

add_executable(final_checker_sphere_fusion src/points_in_polygon.cpp)
ament_target_dependencies(final_checker_sphere_fusion ${dependencies})

#add_executable(fusion src/fusion_node.cpp)
#ament_target_dependencies(fusion ${dependencies})
#ament_target_dependencies(talker pcl_conversions rclcpp sensor_msgs std_msgs pcl_ros cv_bridge image_geometry tf2 tf2_geometry_msgs message_filters)
#add_executable(calib src/calib.cpp)

## Specify libraries to link a library or executable target against

target_link_libraries(ransac_real_time
${PCL_LIBRARIES}
${PCL_COMMON_LIBRARIES}
${PCL_FILTERS_LIBRARIES}
${tf2_LIBRARIES}
${tf2_geometry_msgs_LIBRARIES} 
${ament_LIBRARIES}
)

target_link_libraries(desk_segment
${PCL_LIBRARIES}
${PCL_COMMON_LIBRARIES}
${PCL_FILTERS_LIBRARIES}
${tf2_LIBRARIES}
${tf2_geometry_msgs_LIBRARIES} 
${ament_LIBRARIES}
)

target_link_libraries(checker_detect
${PCL_LIBRARIES}
${PCL_COMMON_LIBRARIES}
${PCL_FILTERS_LIBRARIES}
${tf2_LIBRARIES}
${tf2_geometry_msgs_LIBRARIES} 
${ament_LIBRARIES}
)

target_link_libraries(pts_in_polygon
${PCL_LIBRARIES}
${PCL_COMMON_LIBRARIES}
${PCL_FILTERS_LIBRARIES}
${tf2_LIBRARIES}
${tf2_geometry_msgs_LIBRARIES} 
${ament_LIBRARIES}
)

target_link_libraries(final_checker_sphere_fusion
${PCL_LIBRARIES}
${PCL_COMMON_LIBRARIES}
${PCL_FILTERS_LIBRARIES}
${tf2_LIBRARIES}
${tf2_geometry_msgs_LIBRARIES} 
${ament_LIBRARIES}
)

install(TARGETS ransac_real_time
DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS desk_segment
DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS checker_detect
DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS pts_in_polygon
DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS final_checker_sphere_fusion
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)


#target_link_libraries(fusion
# ${PCL_LIBRARIES}
# ${PCL_COMMON_LIBRARIES}
# ${PCL_FILTERS_LIBRARIES}
# ${tf2_LIBRARIES}
# ${tf2_geometry_msgs_LIBRARIES} 
# ${ament_LIBRARIES}
#)
#install(TARGETS fusion
# DESTINATION lib/${PROJECT_NAME}
#)

ament_export_dependencies(${dependencies})
#target_link_libraries(calib
# ${PCL_LIBRARIES}
# ${PCL_COMMON_LIBRARIES}
# ${PCL_FILTERS_LIBRARIES}
# ${ament_LIBRARIES}
#)

ament_package()