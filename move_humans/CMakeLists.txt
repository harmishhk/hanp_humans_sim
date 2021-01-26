cmake_minimum_required(VERSION 2.8.3)

## enable c++11 mode
set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)

project(move_humans)

## search for dependencies
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  hanp_msgs
  message_generation
  nav_core
  pluginlib
  roscpp
  std_srvs
  tf
)
find_package(Eigen3 REQUIRED)

# add message, serviece and action files from msg, srv and action directories
add_message_files(
  FILES
    HumanPose.msg
    HumanPoseArray.msg
)
add_service_files(
  FILES
    HumanUpdate.srv
)
add_action_files(
  FILES
    MoveHumans.action
)

# generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
)

# add dynamic reconfigure config files from cfg directory
generate_dynamic_reconfigure_options(
  cfg/MoveHumans.cfg
)

# declare catkin package
catkin_package(
  INCLUDE_DIRS
    include
#   LIBRARIES
#     move_humans
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    costmap_2d
    dynamic_reconfigure
    hanp_msgs
    geometry_msgs
    nav_core
    pluginlib
    roscpp
    std_srvs
    tf
#   DEPENDS
#     system_lib
)



## build ##

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

# declare a c++ library
add_library(${PROJECT_NAME}
  src/move_humans.cpp
  src/move_humans_client.cpp
)

# cmake target dependencies of the c++ library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# libraries to link the target c++ library against
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

# declare a c++ executable
add_executable(${PROJECT_NAME}_node
  src/move_humans_node.cpp
)

# rename c++ executable without prefix, for shorter version
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ${PROJECT_NAME} PREFIX "")

# cmake target dependencies of the c++ executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# libraries to link the target c++ executable against
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)



## install ##

# executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install scripts
catkin_install_python(PROGRAMS scripts/relay_human_twist.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# cpp-header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
