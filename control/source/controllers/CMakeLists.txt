target_sources(controllers
        PUBLIC
            CartesianPoseController.cpp
            KDLRobot.cpp
            KinematicController.cpp
            AdaptiveForceFollower.cpp)

target_include_directories(controllers PUBLIC ${Eigen3_INCLUDE_DIRS})
target_link_libraries(controllers Eigen3::Eigen cppzmq state_representation dynamical_systems passive-ds-control orocos-kdl)

if(BUILD_TESTING)
    file(GLOB_RECURSE MODULE_TEST_SOURCES tests test_*.cpp)
    target_sources(run_tests PRIVATE ${MODULE_TEST_SOURCES})
    target_include_directories(run_tests PUBLIC ${Eigen3_INCLUDE_DIRS})
    target_link_libraries(run_tests controllers)
endif()