cmake_minimum_required(VERSION 2.8.3)
project(test_sim)
set(CMAKE_CXX_STANDARD 11)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules/")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    pcl_ros
    pcl_conversions
    sensor_msgs
    octomap_ros
    octomap_msgs
    cmake_modules
)

catkin_package(
  CATKIN_DEPENDS std_msgs
  )

include_directories(
    "${PROJECT_SOURCE_DIR}/src/"
    ${catkin_INCLUDE_DIRS}
    )

add_executable(monitor src/monitor_node.cpp)

target_link_libraries(monitor
  ${catkin_LIBRARIES}
)

add_executable(save_test_data src/save_test_data.cpp)

target_link_libraries(save_test_data
  ${catkin_LIBRARIES}
)

add_executable(save_octomap_to_file src/save_octomap_to_file.cpp)

target_link_libraries(save_octomap_to_file
  ${catkin_LIBRARIES}
)