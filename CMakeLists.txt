cmake_minimum_required(VERSION 2.8.3)
project(rgbd_odometry)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
FIND_PACKAGE(CUDA REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  message_generation
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   RGBDFrame.msg
   RGBDFramePyd.msg
#   Message1.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   sensor_msgs
 )

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
#  LIBRARIES rgbd_odometry
#  CATKIN_DEPENDS cv_bridge image_transport roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########
set(CMAKE_CXX_FLAGS "-g -fpermissive -fopenmp -ffast-math")

# CUDA flags
# help - http://choorucode.com/2013/09/13/how-to-build-cuda-programs-using-cmake/
SET(CUDA_NVCC_FLAGS "-arch=sm_30" CACHE STRING "nvcc flags" FORCE)
#SET(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)




## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include
  ${catkin_INCLUDE_DIRS}
     /usr/local/include/eigen3/
    /usr/local/include/igl/
)

LINK_DIRECTORIES(/usr/local/cuda-5.5/lib64/)

## Declare a cpp library
# add_library(rgbd_odometry
#   src/${PROJECT_NAME}/rgbd_odometry.cpp
# )

## Declare a cpp executable
# add_executable(rgbd_odometry_node src/rgbd_odometry_node.cpp)
add_executable(publisher
    src/publisher.cpp
    src/PrimeSenseCam.cpp
    include/PrimeSenseCam.h
)


add_executable( publisherPyd
    src/publisherPyD.cpp
    src/PrimeSenseCam.cpp
    include/PrimeSenseCam.h
)


add_executable(rgbdSubsc
    src/rgbdSubscriber.cpp
    src/RGBDOdometry.cpp
    include/RGBDOdometry.h
    src/EPoseEstimator.cpp
    include/EPoseEstimator.h
    src/PyramidalStorage.cpp
    include/PyramidalStorage.h
)

add_executable(pnp
    src/pnp.cpp
    src/SolvePnP.cpp
    include/SolvePnP.h
)


add_executable(dvo
    src/dvo.cpp
    src/SolveDVO.cpp
    include/SolveDVO.h
    include/FColorMap.h
    src/FColorMap.cpp
)


cuda_add_executable(cuda_node
    src/cudaMain.cpp
    src/cuda.cu
)


add_executable(imuTest
    src/imuTest.cpp
)


add_executable(imuDR
    src/imuDRMain.cpp
    src/ImuDeadReckon.cpp
    include/ImuDeadReckon.h
)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(rgbd_odometry_node rgbd_odometry_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(publisher
   ${catkin_LIBRARIES} OpenNI2
 )

 target_link_libraries(publisherPyd
   ${catkin_LIBRARIES} OpenNI2
 )


 target_link_libraries(rgbdSubsc
   ${catkin_LIBRARIES}
 )


 target_link_libraries(pnp
   ${catkin_LIBRARIES}
 )

 target_link_libraries(dvo
   ${catkin_LIBRARIES}
 )

 target_link_libraries(cuda_node
   ${catkin_LIBRARIES}
    cublas
 )


 target_link_libraries(imuTest
   ${catkin_LIBRARIES}
 )

 target_link_libraries(imuDR
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
# install(TARGETS rgbd_odometry rgbd_odometry_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rgbd_odometry.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
