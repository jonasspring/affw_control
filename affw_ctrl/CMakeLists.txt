#cmake_minimum_required(VERSION 2.8.3)
#project(affw_ctrl)

#include(ExternalProject)

#add_subdirectory(lib/affw)
#include_directories(lib/affw/include)

#include_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/affw/lwpr/include/)
#include_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/affw/fann/include/)
#include_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/affw/otl/include/)

#link_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/affw/lwpr/lib/)
#link_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/affw/fann/lib/)
#link_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/affw/otl/lib/)

#find_package(catkin REQUIRED COMPONENTS
#	roscpp
#	affw_msgs
#	message_filters
#)

#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

#find_package(Boost REQUIRED COMPONENTS system)


#include_directories(BEFORE ${affw_INCLUDES})
#add_executable(affw_ctrl src/affw_ctrl.cpp)
#add_dependencies(affw_ctrl affw ${${PROJECT_NAME}_EXPORTED_TARGETS} affw_msgs_generate_messages_cpp)#LIB_AFFW)
#target_link_libraries(affw_ctrl ${catkin_LIBRARIES} affw ${affw_LIBRARIES})


#catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp affw_msgs
#)

cmake_minimum_required(VERSION 2.8.3)
project(affw_ctrl)

find_package(catkin REQUIRED COMPONENTS roscpp affw_msgs message_filters cmake_modules)

find_package(Boost REQUIRED COMPONENTS system)

include(ExternalProject)

catkin_package(
  #LIBRARIES afw_lib
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

include_directories(${catkin_INCLUDE_DIRS} include)

add_subdirectory(lib/affw)
include_directories(lib/affw/include)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/affw/lwpr/include/)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/affw/fann/include/)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/affw/otl/include/)

link_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/affw/lwpr/lib/)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/affw/fann/lib/)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/affw/otl/lib/)

include_directories(lib/affw/include/affw)
include_directories(lib/affw/include/affw/learner)
include_directories(lib/affw/include/affw/mapping)
include_directories(lib/affw/include/affw/osvr)
include_directories(lib/affw/include/affw/otl)
include_directories(lib/affw/include/affw/util)

link_directories(lib/affw/src)
link_directories(lib/affw/src/learner)
link_directories(lib/affw/src/mapping)
link_directories(lib/affw/src/otl)
link_directories(lib/affw/src/util)

#add_library(affw_lib lib/affw/src/Config.cpp)
##lib/affw/src/learner/DummyLearner.cpp lib/affw/src/learner/FeedbackController.cpp #lib/affw/src/learner/FANNLearner.cpp
##lib/affw/src/learner/LWPRLearner.cpp lib/affw/src/learner/ModelLearner.cpp lib/affw/src/learner/OESGPLearner.cpp
##lib/affw/src/learner/OSVRLearner.cpp lib/affw/src/learner/RLSESNLearner.cpp lib/affw/src/learner/RLSLearner.cpp
##lib/affw/src/learner/SOGPLearner.cpp lib/affw/src/learner/STORKGPLearner.cpp lib/affw/src/learner/WrapperLearner.cpp
##lib/affw/src/mapping/DataMapper.cpp lib/affw/src/mapping/KTermStateTarget2ActionCompMapper.cpp
##lib/affw/src/osvr/CrossValidation.cpp lib/affw/src/osvr/Example.cpp lib/affw/src/osvr/File.cpp lib/affw/src/osvr/Forget.cpp
##lib/affw/src/osvr/Kernel.cpp lib/affw/src/osvr/Main.cpp lib/affw/src/osvr/OnlineSVR.cpp lib/affw/src/osvr/Show.cpp
##lib/affw/src/osvr/Stabilize.cpp lib/affw/src/osvr/Train.cpp lib/affw/src/osvr/Variations.cpp
##lib/affw/src/otl/OESGPasync.cpp lib/affw/src/otl/otl_rls_osn.cpp lib/affw/src/otl/STORKGP_async.cpp
##lib/affw/src/test/lwprPerfTest.cpp #lib/affw/src/test/batchLearnFann.cpp lib/affw/src/test/batchLearnOTL.cpp
##lib/affw/src/test/testDelayEstimator.cpp lib/affw/src/test/testLearner.cpp
##lib/affw/src/util/affw_factory.cpp lib/affw/src/util/DelayEstimator.cpp lib/affw/src/util/pidnn.c)
#add_dependencies(affw_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} affw_msgs_generate_messages_cpp)
#target_link_libraries(affw_lib ${catkin_LIBRARIES})


add_executable(affw_ctrl src/affw_ctrl.cpp)
add_dependencies(affw_ctrl affw ${${PROJECT_NAME}_EXPORTED_TARGETS} affw_msgs_generate_messages_cpp)
target_link_libraries(affw_ctrl  ${catkin_LIBRARIES} affw ${affw_LIBRARIES})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS affw_ctrl
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

