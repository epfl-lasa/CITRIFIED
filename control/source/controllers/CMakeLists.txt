target_sources(local_controllers
        PUBLIC
            KDLRobot.cpp
            KinematicController.cpp
            IncisionTrialSystem.cpp
            CutProber.cpp
            FrankaController.cpp)

target_include_directories(local_controllers PUBLIC ${Eigen3_INCLUDE_DIRS})
target_link_libraries(local_controllers Eigen3::Eigen yaml-cpp state_representation dynamical_systems controllers orocos-kdl pthread)

if(BUILD_TESTING)
    file(GLOB_RECURSE MODULE_TEST_SOURCES tests test_*.cpp)
    target_sources(run_tests PRIVATE ${MODULE_TEST_SOURCES})
    target_include_directories(run_tests PUBLIC ${Eigen3_INCLUDE_DIRS})
    target_link_libraries(run_tests local_controllers)
    target_compile_definitions(run_tests PRIVATE CONTROLLERS_TEST_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/")
endif()