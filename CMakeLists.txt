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


add_executable(${PROJECT_NAME}_readbag_node src/readbag.cpp)
add_dependencies(${PROJECT_NAME}_readbag_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_readbag_node
  ${catkin_LIBRARIES}
)


