cmake_minimum_required(VERSION 2.8.3)
project(calibration_common)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
             COMPONENTS cmake_modules
                        tf
                        image_geometry
                        eigen_conversions
                        visualization_msgs
                        calibration_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)

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

set(INCLUDE_FILES
  include/calibration_common/algorithms/algorithms.h
  include/calibration_common/algorithms/point_on_plane_calibration.h
  include/calibration_common/algorithms/automatic_checkerboard_finder.h
  include/calibration_common/algorithms/interactive_checkerboard_finder.h
  include/calibration_common/algorithms/plane_extractor.h
  include/calibration_common/algorithms/plane_to_plane_calibration.h
  include/calibration_common/algorithms/checkerboard_finder.h

  include/calibration_common/pinhole/pinhole.h
  include/calibration_common/pinhole/sensor.h
  include/calibration_common/pinhole/view.h
  include/calibration_common/pinhole/camera_model.h

  include/calibration_common/calibration_common.h

  include/calibration_common/base/matrix.h
  include/calibration_common/base/opencv_conversion.h
  include/calibration_common/base/math.h
  include/calibration_common/base/point_matrix.h
  include/calibration_common/base/pcl_conversion.h
  include/calibration_common/base/base.h

  include/calibration_common/depth/undistortion_model.h
  include/calibration_common/depth/sensor.h
  include/calibration_common/depth/depth.h
  include/calibration_common/depth/view.h
  include/calibration_common/depth/traits.h
  include/calibration_common/depth/undistortion_model_fit.h

  include/calibration_common/color/color.h
  include/calibration_common/color/sensor.h
  include/calibration_common/color/view.h
  include/calibration_common/color/camera_model.h

  include/calibration_common/rgbd/data.h

  include/calibration_common/objects/sensor.h
  include/calibration_common/objects/checkerboard.h
  include/calibration_common/objects/globals.h
  include/calibration_common/objects/planar_object.h
  include/calibration_common/objects/base_object.h
  include/calibration_common/objects/view.h
  include/calibration_common/objects/objects.h
  
  include/calibration_common/ceres/polynomial_fit.h
  include/calibration_common/ceres/plane_fit.h
  include/calibration_common/ceres/math.h
  include/calibration_common/ceres/ceres.h

  include/impl/calibration_common/algorithms/plane_extractor.hpp
  include/impl/calibration_common/pinhole/camera_model.hpp
  include/impl/calibration_common/base/opencv_conversion.hpp
  include/impl/calibration_common/base/pcl_conversion.hpp
  include/impl/calibration_common/depth/view.hpp
  include/impl/calibration_common/ceres/polynomial_fit.hpp
  include/impl/calibration_common/ceres/plane_fit.hpp
)

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} calibration_algorithms
  CATKIN_DEPENDS tf image_geometry eigen_conversions visualization_msgs calibration_msgs
  DEPENDS pcl eigen opencv2 ceres
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/calibration_common/objects/base_object.cpp
  src/calibration_common/objects/checkerboard.cpp
  src/calibration_common/objects/planar_object.cpp
  src/calibration_common/pinhole/camera_model.cpp
  src/calibration_common/rgbd/data.cpp
  ${INCLUDE_FILES}
)

add_library(calibration_algorithms
  src/calibration_common/algorithms/automatic_checkerboard_finder.cpp
  src/calibration_common/algorithms/interactive_checkerboard_finder.cpp
  src/calibration_common/algorithms/plane_to_plane_calibration.cpp
  src/calibration_common/algorithms/point_on_plane_calibration.cpp
)

## Declare a cpp executable

## Add dependencies to the executable
# add_dependencies(calibration_node ${PROJECT_NAME})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(calibration_algorithms
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)

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

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
  test/base/run_tests.cpp
  test/base/base_object_test.cpp
  test/base/checkerboard_test.cpp
  test/base/matrix_test.cpp
  test/base/sensor_test.cpp
)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
