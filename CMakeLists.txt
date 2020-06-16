cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(basler_tof)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  dynamic_reconfigure
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/Modules)
find_package(GenTL REQUIRED)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -march=native -Wl,--enable-new-dtags -Wl,-rpath,${GENTL_LIBRARY_DIR}")

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(cfg/BaslerToF.cfg)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS sensor_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GENTL_INCLUDE_DIRS}
)

add_executable(basler_tof_node src/basler_tof_node.cpp)
add_dependencies(basler_tof_node ${PROJECT_NAME}_gencfg)
add_dependencies(basler_tof_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(basler_tof_node
  ${catkin_LIBRARIES}
  ${GENTL_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS basler_tof_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
