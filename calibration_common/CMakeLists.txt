cmake_minimum_required(VERSION 2.8.3)
project(calibration_common)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    image_geometry
)

find_package(Eigen REQUIRED)

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
  INCLUDE_DIRS include ${EIGEN_INCLUDE_DIRS}
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    cmake_modules
    image_geometry
#  DEPENDS
#    Eigen
)

###########
## Build ##
###########

add_custom_target(${PROJECT_NAME}_HEADERS
  SOURCES
    include/${PROJECT_NAME}/base/base.h
    include/${PROJECT_NAME}/base/definitions.h
    include/${PROJECT_NAME}/base/geometry.h
    include/${PROJECT_NAME}/base/eigen_cloud.h
    include/${PROJECT_NAME}/base/opencv_eigen_conversions.h
    include/${PROJECT_NAME}/base/polynomial.h
    include/${PROJECT_NAME}/base/eigen_bugfix.h

    include/${PROJECT_NAME}/depth/depth.h
    include/${PROJECT_NAME}/depth/sensor.h
    include/${PROJECT_NAME}/depth/view.h

    include/${PROJECT_NAME}/objects/base_object.h
    include/${PROJECT_NAME}/objects/checkerboard.h
    include/${PROJECT_NAME}/objects/objects.h
    include/${PROJECT_NAME}/objects/planar_object.h
    include/${PROJECT_NAME}/objects/sensor.h
    include/${PROJECT_NAME}/objects/view.h
    include/${PROJECT_NAME}/objects/with_points.h

    include/${PROJECT_NAME}/pinhole/camera_model.h
    include/${PROJECT_NAME}/pinhole/pinhole.h
    include/${PROJECT_NAME}/pinhole/sensor.h
    include/${PROJECT_NAME}/pinhole/view.h
    include/impl/${PROJECT_NAME}/pinhole/camera_model.hpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

# add_dependencies(calibration_common_node calibration_common_generate_messages_cpp)

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
# install(TARGETS calibration_common calibration_common_node
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

catkin_add_gtest(${PROJECT_NAME}-test
#add_executable(${PROJECT_NAME}-test
  test/test_${PROJECT_NAME}.cpp
  test/base/test_polynomial.cpp
  test/base/test_opencv_eigen_conversions.cpp
  test/objects/test_checkerboard.cpp
  test/pinhole/test_camera_model.cpp
)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test
    ${catkin_LIBRARIES}
  )
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
