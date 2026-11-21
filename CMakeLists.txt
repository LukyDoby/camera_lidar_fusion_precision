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
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(message_filters REQUIRED)

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

## Declare a C++ executable

add_executable(pts_in_polygon src/points_in_polygon.cpp)
ament_target_dependencies(pts_in_polygon ${dependencies})

## Specify libraries to link a library or executable target against

target_link_libraries(pts_in_polygon
${PCL_LIBRARIES}
${PCL_COMMON_LIBRARIES}
${PCL_FILTERS_LIBRARIES}
${tf2_LIBRARIES}
${tf2_geometry_msgs_LIBRARIES} 
${ament_LIBRARIES}
)

install(TARGETS pts_in_polygon
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})

ament_package()