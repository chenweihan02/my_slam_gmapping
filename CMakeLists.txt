cmake_minimum_required(VERSION 3.0.2)
project(my_slam_gmapping)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  std_msgs
  tf
)


catkin_package(
  # CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(my_slam_gmapping src/part_ros/main.cpp
                                src/part_ros/my_slam_gmapping.cpp
                                src/part_data/lidar_undistortion/lidar_undistortion.cpp
                                src/part_slam/gridfastslam/gridslamprocessor.cpp
                                src/part_slam/sensor_range/rangereading.cpp
                                src/part_slam/motionmodel/motionmodel.cpp
                                src/part_slam/scanmatcher/scanmatcher.cpp
                                )


target_link_libraries(my_slam_gmapping ${catkin_LIBRARIES})

# add_executable(${PROJECT_NAME}_readbag_node src/readbag.cpp)
# add_dependencies(${PROJECT_NAME}_readbag_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(${PROJECT_NAME}_readbag_node
#   ${catkin_LIBRARIES}
# )


