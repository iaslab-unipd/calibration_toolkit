cmake_minimum_required(VERSION 2.8.3)
project(calibration_ros)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    image_transport
    cv_bridge
    calibration_pcl
    tf
    eigen_conversions
)
#find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED
  COMPONENTS
  common
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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
  INCLUDE_DIRS include
  LIBRARIES
    ${PROJECT_NAME}_sensors
    ${PROJECT_NAME}_devices
    ${PROJECT_NAME}_visualization
  CATKIN_DEPENDS
    image_transport
    cv_bridge
    calibration_pcl
    tf
    eigen_conversions
#  DEPENDS system_lib
)

add_custom_target(${PROJECT_NAME}_HEADERS
  SOURCES
    include/${PROJECT_NAME}/sensors/ros_sensor.h
    include/${PROJECT_NAME}/sensors/pinhole_camera.h
    include/${PROJECT_NAME}/sensors/depth_sensor.h

    include/${PROJECT_NAME}/devices/ros_device.h

    include/${PROJECT_NAME}/visualization/objects.h
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_sensors
  src/${PROJECT_NAME}/sensors/pinhole_camera.cpp
  src/${PROJECT_NAME}/sensors/depth_sensor.cpp
)

add_library(${PROJECT_NAME}_devices
  src/${PROJECT_NAME}/devices/ros_device.cpp
)

add_library(${PROJECT_NAME}_visualization
  src/${PROJECT_NAME}/visualization/objects.cpp
)

## Declare a cpp executable
# add_executable(calibration_ros_node src/calibration_ros_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(calibration_ros_node calibration_ros_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_sensors
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_devices
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_sensors
)

target_link_libraries(${PROJECT_NAME}_visualization
  ${catkin_LIBRARIES}
)

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
# install(TARGETS calibration_ros calibration_ros_node
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

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
