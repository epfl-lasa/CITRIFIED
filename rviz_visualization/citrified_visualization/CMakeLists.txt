cmake_minimum_required(VERSION 2.8.3)
project(citrified_visualization)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
    rospy
    tf2_ros
    geometry_msgs
    )

catkin_python_setup()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
    scripts/write_data_to_rosbag
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
