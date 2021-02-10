target_sources(motion_generators
        PRIVATE
            BaseDS.cpp
        PUBLIC
            CircularDS.cpp
        RingDS.cpp
            PointAttractorDS.cpp)
target_include_directories(motion_generators PUBLIC)
target_link_libraries(motion_generators Eigen3::Eigen cppzmq state_representation dynamical_systems)

if(BUILD_TESTING)
    file(GLOB_RECURSE MODULE_TEST_SOURCES tests test_*.cpp)
    if(MODULE_TEST_SOURCES)
        message("Adding motion generator tests")
        target_sources(run_tests PRIVATE ${MODULE_TEST_SOURCES})
        target_include_directories(run_tests PUBLIC ${Eigen3_INCLUDE_DIRS})
        target_link_libraries(run_tests motion_generators)
    endif()
endif()