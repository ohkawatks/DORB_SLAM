cmake_minimum_required(VERSION 2.8.3)
project(measurment_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  sensor_msgs
  tf
  message_generation
)

find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread signals)

add_service_files(
  DIRECTORY srv
 FILES 
 measurmentSrv.srv 
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs sensor_msgs
  DEPENDS system_lib Eigen
)

add_definitions("-Wall -std=c++11")
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable( measurment_client_node_div
	src/measurment_client.cpp 
	src/measurmentManager.cpp )

target_link_libraries( measurment_client_node_div  ${catkin_LIBRARIES} )
add_dependencies( measurment_client_node_div measurment_controller_generate_messages_cpp ${catkin_EXPORTED_TARGETS})


add_executable( measurment_client_node_normal
	src/measurment_client_none.cpp 
	src/measurmentManager.cpp )
target_link_libraries( measurment_client_node_normal  ${catkin_LIBRARIES} )
add_dependencies( measurment_client_node_normal measurment_controller_generate_messages_cpp)

add_executable( measurment_server_node
	src/measurment_server.cpp 
	src/measurmentManager.cpp )
target_link_libraries( measurment_server_node  ${catkin_LIBRARIES} )
add_dependencies( measurment_server_node measurment_controller_generate_messages_cpp)


