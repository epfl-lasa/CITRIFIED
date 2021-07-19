cmake_minimum_required(VERSION 3.0.2)
project(pure_zmq_joy)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(cppzmq REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS roscpp joy sensor_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME}_node src/pure_zmq_joy_node.cpp src/PureZMQJoy.cpp)
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   cppzmq
 )
