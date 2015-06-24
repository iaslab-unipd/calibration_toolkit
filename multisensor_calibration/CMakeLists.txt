cmake_minimum_required(VERSION 2.8.3)
project(multisensor_calibration)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    calibration_ros
    calibration_msgs
    calibration_algorithms
)

## System dependencies are found with CMake's conventions
find_package(Ceres 1.10 REQUIRED)

set(yaml-cpp_INCLUDE_DIRS /usr/include/yaml-cpp/)
set(yaml-cpp_LIBRARIES yaml-cpp)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    calibration_ros
    calibration_msgs
    calibration_algorithms
  DEPENDS
   yaml-cpp
)

###########
## Build ##
###########

add_custom_target(${PROJECT_NAME}_HEADERS
  SOURCES
    include/${PROJECT_NAME}/common.h
    include/${PROJECT_NAME}/device_node.h
    include/${PROJECT_NAME}/master_node.h
    include/${PROJECT_NAME}/calibration.h
    include/${PROJECT_NAME}/optimization.h

    include/${PROJECT_NAME}/corner_based/errors.h
    include/${PROJECT_NAME}/corner_based/plane_errors.h
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

## Declare a cpp library
#add_library(${PROJECT_NAME}
#)

## Declare a cpp executable
add_executable(master_node
  src/${PROJECT_NAME}/master_node.cpp
  src/${PROJECT_NAME}/calibration.cpp
  src/${PROJECT_NAME}/optimization.cpp
)

add_executable(device_node
  src/${PROJECT_NAME}/device_node.cpp
)

#add_executable(pinhole_rgb_camera_node
#  src/${PROJECT_NAME}/devices/pinhole_rgb_camera.cpp
#  src/${PROJECT_NAME}/pinhole_rgb_camera_node.cpp
#)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(master_node calibration_msgs)
add_dependencies(device_node calibration_msgs)
#add_dependencies(pinhole_rgb_camera_node calibration_msgs)

## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)

target_link_libraries(master_node
  ${catkin_LIBRARIES}
  ${yaml-cpp_LIBRARIES}
  ${CERES_LIBRARIES}
)

target_link_libraries(device_node
  ${catkin_LIBRARIES}
)

#target_link_libraries(pinhole_rgb_camera_node
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS multisensor_calibration multisensor_calibration_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_multisensor_calibration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

