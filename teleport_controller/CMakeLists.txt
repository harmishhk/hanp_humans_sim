cmake_minimum_required(VERSION 2.8.3)

## enable c++11 mode
set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)

project(teleport_controller)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  hanp_msgs
  hanp_prediction
  move_humans
  nav_core
  nav_msgs
  pluginlib
  pcl_conversions
  roscpp
  tf
  visualization_msgs
)

# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)


## install python modules and global scripts
# catkin_python_setup()

## add message, serviece and action files from msg, srv and action directories

# add_message_files(
#   FILES
#     Message1.msg
# )

# add_service_files(
#   FILES
#     Service1.srv
# )

# add_action_files(
#   FILES
#     Action1.action
# )

## generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   hanp_msgs
# )

# add dynamic reconfigure config files from cfg directory
generate_dynamic_reconfigure_options(
  cfg/TeleportController.cfg
)

## declare catkin package
catkin_package(
  INCLUDE_DIRS
    include
#   LIBRARIES
#     teleport_controller
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    hanp_msgs
    hanp_prediction
    move_humans
    nav_core
    nav_msgs
    pluginlib
    pcl_conversions
    roscpp
    tf
    visualization_msgs
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
  src/teleport_controller.cpp
)

# cmake target dependencies of the c++ library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# libraries to link the target c++ library against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## declare a c++ executable
# add_executable(${PROJECT_NAME}_node
#   src/teleport_controller_node.cpp
# )

## rename c++ executable without prefix, for shorter version
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## cmake target dependencies of the c++ executable
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## libraries to link the target c++ executable against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )



## install ##

## executable scripts (python etc.) for installation
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## cpp-header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## other files for installation (launch, data, bag files)
# foreach(dir launch data bags)
#    install(DIRECTORY ${dir}/
#       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

## other files for installation
# install(
#   FILES
#     myfile1
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )



## testing ##

## add catch based cpp test target
# catkin_add_catchtest(${PROJECT_NAME}-test test/test_teleport_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## add nosetest file folders
# catkin_add_nosetests(test)
