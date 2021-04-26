target_sources(local_controllers
        PUBLIC
            KDLRobot.cpp
            KinematicController.cpp
            TwistController.cpp
            IncisionTrialSystem.cpp
            CutProber.cpp)

target_include_directories(local_controllers PUBLIC ${Eigen3_INCLUDE_DIRS})
target_link_libraries(local_controllers Eigen3::Eigen cppzmq state_representation dynamical_systems orocos-kdl controllers)

if(BUILD_TESTING)
    file(GLOB_RECURSE MODULE_TEST_SOURCES tests test_*.cpp)
    target_sources(run_tests PRIVATE ${MODULE_TEST_SOURCES})
    target_include_directories(run_tests PUBLIC ${Eigen3_INCLUDE_DIRS})
    target_link_libraries(run_tests local_controllers)
endif()