cmake_minimum_required(VERSION 3.0.2)
project(zmq_bridge)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs franka_panda_description)
find_package(cppzmq REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS roscpp sensor_msgs geometry_msgs franka_panda_description
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp src/ZMQBridge.cpp)
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   cppzmq
 )
