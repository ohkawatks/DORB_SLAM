cmake_minimum_required(VERSION 2.8.3)
project(dorbslam)

set(CMAKE_C_FLAGS "${CMAKE_C_fLAGS} -Wall -O3 -march=native")# -g -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O3 -march=native")# -g -O0")
set(CMAKE_BUILD_TYPE Release)
#if would you like to see performance,please comment out following line.
#add_definitions(-DENABLE_PERFORM)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  cmake_modules
  roscpp
  cv_bridge
  message_generation
)
find_package(Eigen REQUIRED)
find_package(OpenCV  REQUIRED)
find_package(Pangolin REQUIRED)
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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   KeyPoint.msg
   OrbDescriptor.msg
   ExtractorSettings.msg
   Edge.msg
   VertexKF.msg
   VertexMP.msg
   LocalGraph.msg
   TrackingState.msg
   TimeStamp.msg
 )

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   BundleAdjustment.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  LIBRARIES orbslam 
  CATKIN_DEPENDS roscpp image_transport cv_bridge
  message_generation
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  src/include
  ${EIGEN_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  .
  Thirdparty/g2o/g2o/core
  ../measurment_controller/include    
  )

# comment out lines below if you don't need timestamp topics published
message(STATUS "timestamp enabled")
ADD_DEFINITIONS("-DPUBLISH_TIME_STAMP")

# dont define the DENABLE_EXTERNAL_LOCALBUNDLE_ADJUSTMENT 
# if you like to build the orbslam library without ROS

#add_definitions( -DENABLE_EXTERNAL_LOCALBUNDLE_ADJUSTMENT)

add_library(${PROJECT_NAME} SHARED
src/${PROJECT_NAME}/System.cc
src/${PROJECT_NAME}/Tracking.cc
src/${PROJECT_NAME}/LocalMapping.cc
src/${PROJECT_NAME}/LoopClosing.cc
src/${PROJECT_NAME}/ORBextractor.cc
src/${PROJECT_NAME}/ORBmatcher.cc
src/${PROJECT_NAME}/FrameDrawer.cc
src/${PROJECT_NAME}/Converter.cc
src/${PROJECT_NAME}/MapPoint.cc
src/${PROJECT_NAME}/KeyFrame.cc
src/${PROJECT_NAME}/Map.cc
src/${PROJECT_NAME}/MapDrawer.cc
src/${PROJECT_NAME}/Optimizer.cc
src/${PROJECT_NAME}/PnPsolver.cc
src/${PROJECT_NAME}/Frame.cc
src/${PROJECT_NAME}/KeyFrameDatabase.cc
src/${PROJECT_NAME}/Sim3Solver.cc
src/${PROJECT_NAME}/Initializer.cc
src/${PROJECT_NAME}/Viewer.cc
../measurment_controller/src/measurmentManager.cpp
)

#target_compile_definitions(${PROJECT_NAME} PUBLIC ENABLE_EXTERNAL_LOCALBUNDLE_ADJUSTMENT=1 PUBLIC __DISABLE_GUI=1)
add_dependencies( ${PROJECT_NAME} SHARED measurment_controller_generate_messages_cpp)

#define macro only in the orbslam library for div2 
add_library(${PROJECT_NAME}_div2 SHARED
src/${PROJECT_NAME}/System.cc
src/${PROJECT_NAME}/Tracking.cc
src/${PROJECT_NAME}/LocalMapping.cc
src/${PROJECT_NAME}/LoopClosing.cc
src/${PROJECT_NAME}/ORBextractor.cc
src/${PROJECT_NAME}/ORBmatcher.cc
src/${PROJECT_NAME}/FrameDrawer.cc
src/${PROJECT_NAME}/Converter.cc
src/${PROJECT_NAME}/MapPoint.cc
src/${PROJECT_NAME}/KeyFrame.cc
src/${PROJECT_NAME}/Map.cc
src/${PROJECT_NAME}/MapDrawer.cc
src/${PROJECT_NAME}/Optimizer.cc
src/${PROJECT_NAME}/PnPsolver.cc
src/${PROJECT_NAME}/Frame.cc
src/${PROJECT_NAME}/KeyFrameDatabase.cc
src/${PROJECT_NAME}/Sim3Solver.cc
src/${PROJECT_NAME}/Initializer.cc
src/${PROJECT_NAME}/Viewer.cc
../measurment_controller/src/measurmentManager.cpp
)
add_dependencies( ${PROJECT_NAME}_div2 SHARED measurment_controller_generate_messages_cpp)
#target_compile_definitions(${PROJECT_NAME}_div2 PUBLIC ENABLE_EXTERNAL_LOCALBUNDLE_ADJUSTMENT=1 PUBLIC __DISABLE_GUI=1)
target_compile_definitions(${PROJECT_NAME}_div2 PUBLIC ENABLE_EXTERNAL_LOCALBUNDLE_ADJUSTMENT=1)
## Declare a C++ library
# add_library(orbslam
#   src/${PROJECT_NAME}/orbslam.cpp
# )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  )
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${OpenCV_LIBS})

target_link_libraries(${PROJECT_NAME}_div2
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  )
add_dependencies(${PROJECT_NAME}_div2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${OpenCV_LIBS})


## Declare a C++ executable
add_executable(
dorbslam_node src/dorbslam_node.cc
../measurment_controller/src/measurmentManager.cpp
)
add_dependencies( dorbslam_node SHARED measurment_controller_generate_messages_cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(dorbslam_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)


# Specify libraries to link a library or executable target against
target_link_libraries(dorbslam_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  )

add_executable(orb_extractor_node 
src/orb_extractor_mono.cpp
../measurment_controller/src/measurmentManager.cpp )

target_link_libraries(orb_extractor_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  )
add_dependencies( orb_extractor_node measurment_controller_generate_messages_cpp)

add_executable( dorbslam_node_div1 src/dorbslam_wo_extractor.cc
../measurment_controller/src/measurmentManager.cpp )
add_dependencies( dorbslam_node_div1 measurment_controller_generate_messages_cpp)

 ## Add cmake target dependencies of the executable
 ## same as for the library above
 add_dependencies(dorbslam_node_div1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(dorbslam_node_div1
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  )


#add_executable(orbslam_node_div2 src/dorb_slam_div1.cc)
add_executable(dorbslam_node_div2 src/dorbslam_node.cc
../measurment_controller/src/measurmentManager.cpp )
add_dependencies( dorbslam_node_div2 measurment_controller_generate_messages_cpp)

 ## Add cmake target dependencies of the executable
 ## same as for the library above

add_dependencies(dorbslam_node_div2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )


target_link_libraries(dorbslam_node_div2
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_div2
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  )
set_target_properties(dorbslam_node_div2 PROPERTIES COMPILE_DEFINITIONS "ENABLE_EXTERNAL_LOCALBUNDLE_ADJUSTMENT=1")
set_target_properties(dorbslam_node_div2 PROPERTIES COMPILE_DEFINITIONS "VALIDATION=1")

#add_executable(orbslam_node_div2 src/dorb_slam_div1.cc)
add_executable(dorbslam_node_div3 src/dorbslam_wo_extractor.cc
../measurment_controller/src/measurmentManager.cpp )
 ## Add cmake target dependencies of the executable
 ## same as for the library above

add_dependencies(dorbslam_node_div3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies( dorbslam_node_div3 measurment_controller_generate_messages_cpp)


target_link_libraries(dorbslam_node_div3
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_div2
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  )
set_target_properties(dorbslam_node_div3 PROPERTIES COMPILE_DEFINITIONS "ENABLE_EXTERNAL_LOCALBUNDLE_ADJUSTMENT=1")



## Declare a C++ executable
add_executable(bundle_adjustment_node src/bundle_adjustment_node.cc
../measurment_controller/src/measurmentManager.cpp )
add_dependencies( bundle_adjustment_node measurment_controller_generate_messages_cpp)

target_link_libraries(bundle_adjustment_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
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
# install(TARGETS orbslam orbslam_node
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

# Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_orbslam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
