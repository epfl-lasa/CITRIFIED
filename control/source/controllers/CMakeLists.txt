target_sources(lib_controllers PUBLIC CartesianPoseController.cpp KDLRobot.cpp KinematicController.cpp)

target_include_directories(lib_controllers PUBLIC ${Eigen3_INCLUDE_DIRS})
target_link_libraries(lib_controllers Eigen3::Eigen cppzmq state_representation dynamical_systems passive-ds-control orocos-kdl)

add_executable(test_kdl_robot test_kdl_robot.cpp)
target_link_libraries(test_kdl_robot lib_controllers)
add_test(NAME test_kdl_robot COMMAND test_kdl_robot)