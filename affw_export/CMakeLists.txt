#cmake_minimum_required(VERSION 2.8.3)
#project(affw_export)

#find_package(catkin REQUIRED COMPONENTS
#	roscpp
#	affw_msgs
#)

#find_package(Boost REQUIRED COMPONENTS filesystem)

#catkin_package(
#  CATKIN_DEPENDS roscpp affw_msgs
#)

#add_executable(affw_export src/affw_export.cpp)
#add_dependencies(affw_export ${${PROJECT_NAME}_EXPORTED_TARGETS} affw_msgs_generate_messages_cpp)
#target_link_libraries(affw_export ${catkin_LIBRARIES})

##############
### Install ##
##############

## Mark executables and/or libraries for installation
#install(TARGETS affw_export
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(DIRECTORY
#  launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

cmake_minimum_required(VERSION 2.8.3)
project(affw_export)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        affw_msgs
        cmake_modules
)

find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
  #CATKIN_DEPENDS roscpp affw_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(affw_export src/affw_export.cpp)
add_dependencies(affw_export ${${PROJECT_NAME}_EXPORTED_TARGETS} affw_msgs_generate_messages_cpp)
target_link_libraries(affw_export ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS affw_export
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
