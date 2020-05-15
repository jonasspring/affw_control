cmake_minimum_required(VERSION 2.8.3)
project(affw_pos2vel)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs tf
)

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(affw_pos2vel src/affw_pos2vel.cpp)
target_link_libraries(affw_pos2vel ${catkin_LIBRARIES})
