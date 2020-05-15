cmake_minimum_required(VERSION 2.8.3)
project(affw_traj)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	move_base_msgs 
	actionlib
	tf
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS roscpp affw_ctrl move_base_msgs actionlib tf
)
include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(affw_traj src/affw_traj.cpp)
target_link_libraries(affw_traj ${catkin_LIBRARIES})
