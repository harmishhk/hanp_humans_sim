cmake_minimum_required(VERSION 2.8.3)

## enable c++11 mode
set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)

project(multigoal_planner)

## search for dependencies
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  global_planner
  hanp_msgs
  move_humans
  nav_core
  nav_msgs
  pluginlib
  roscpp
)

# add dynamic reconfigure config files from cfg directory
generate_dynamic_reconfigure_options(
  cfg/MultiGoalPlanner.cfg
)

## declare catkin package
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    multigoal_planner
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    global_planner
    hanp_msgs
    move_humans
    nav_core
    nav_msgs
    pluginlib
    roscpp
  # DEPENDS
  #   system_lib
)



## build ##

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# declare a c++ library
add_library(${PROJECT_NAME}
  src/multigoal_planner.cpp
)

# cmake target dependencies of the c++ library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# libraries to link the target c++ library against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)



## install ##

# executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# cpp-header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# other files for installation
install(
  FILES
    planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
