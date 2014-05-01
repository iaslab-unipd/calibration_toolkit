cmake_minimum_required(VERSION 2.8.3)
project(kinect)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
             COMPONENTS cmake_modules
                        calibration_common)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

#add_message_files(
#  FILES
#  CheckerboardMsg.msg
#  CheckerboardArray.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS calibration_common
  DEPENDS pcl eigen opencv2 ceres
)

###########
## Build ##
###########

set(INCLUDE_FILES
  include/kinect/depth/camera_model.h
  include/kinect/depth/depth.h
  include/kinect/depth/polynomial_function.h
  include/kinect/depth/polynomial_function_fit.h
  include/kinect/depth/polynomial_matrix.h
  include/kinect/depth/polynomial_matrix_fit.h
  include/kinect/depth/polynomial_matrix_io.h
  include/kinect/depth/polynomial_matrix_model.h
  include/kinect/depth/sensor.h
  include/kinect/depth/two_steps_model.h
  include/kinect/depth/two_steps_undistortion.h

  include/impl/kinect/depth/polynomial_matrix_io.hpp
  include/impl/kinect/depth/polynomial_matrix_fit.hpp
  include/impl/kinect/depth/polynomial_function_fit.hpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

#add_executable(mainfile
#  test/read_write_test.cpp
#  ${INCLUDE_FILES}
#)

#target_link_libraries(mainfile
#  ${catkin_LIBRARIES}
#  ${PCL_LIBRARIES}
#  ${CERES_LIBRARIES}
#)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS calibration calibration_node
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

# Add gtest based cpp test target and link libraries
catkin_add_gtest(polynomial_matrix_io-test
  test/polynomial_matrix_io-test.cpp
  ${INCLUDE_FILES}
)

if(TARGET polynomial_matrix_io-test)
  target_link_libraries(polynomial_matrix_io-test
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CERES_LIBRARIES}
  )
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
