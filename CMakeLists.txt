cmake_minimum_required(VERSION 3.0.2)
project(my_slam_gmapping)
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  std_msgs
  nav_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_readbag_node src/readbag.cpp)
add_dependencies(${PROJECT_NAME}_readbag_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_readbag_node
  ${catkin_LIBRARIES}
)


# add_executable(readbag
#   src/readbag.cpp)

# target_link_libraries(occupany_mapping
#   ${catkin_LIBRARIES}
#   ${EIGEN_LIBRARIES}
# )