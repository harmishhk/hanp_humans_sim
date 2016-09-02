cmake_minimum_required(VERSION 2.8.3)

## enable c++11 mode
set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)

project(move_humans_config)

## search for dependencies
find_package(catkin REQUIRED)

## declare catkin package
catkin_package()



## install ##

# other files for installation (launch, data, bag files)
foreach(dir config launch maps rviz)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
