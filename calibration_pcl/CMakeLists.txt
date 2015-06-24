cmake_minimum_required(VERSION 2.8.3)
project(calibration_pcl)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    calibration_common
)
find_package(Boost REQUIRED
  COMPONENTS
    system
    signals
)
find_package(PCL 1.7 REQUIRED)

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
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    calibration_common
  DEPENDS
    Boost
    PCL
)

###########
## Build ##
###########

add_custom_target(${PROJECT_NAME}_HEADERS
  SOURCES
  include/${PROJECT_NAME}/base/base.h
  include/${PROJECT_NAME}/base/definitions.h
  include/${PROJECT_NAME}/base/pcl_eigen_conversions.h

  include/${PROJECT_NAME}/depth/view.h

  include/${PROJECT_NAME}/utilities/plane_extraction.h
  include/${PROJECT_NAME}/utilities/point_plane_extraction.h
  include/${PROJECT_NAME}/utilities/point_plane_extraction_gui.h
  include/impl/${PROJECT_NAME}/utilities/point_plane_extraction.hpp
  include/impl/${PROJECT_NAME}/utilities/point_plane_extraction_gui.hpp
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a cpp library
#add_library(calibration_pcl
#  src/${PROJECT_NAME}/calibration_pcl.cpp
#)


## Declare a cpp executable
#add_executable(dummy src/dummy.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(calibration_pcl_node calibration_pcl_generate_messages_cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(checkerboard_corners_extraction
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
# install(TARGETS calibration_pcl calibration_pcl_node
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
  test/test_${PROJECT_NAME}.cpp
  test/base/test_pcl_eigen_conversions.cpp
)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test
    ${PCL_LIBS}
  )
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
