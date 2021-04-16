target_sources(fast_gmm
    PUBLIC
    CDDynamics.cpp
    Gaussians.cpp
    GMRDynamics.cpp
#        main.cpp
        MJDynamics.cpp
        SVR.cpp
        ThirdPoly.cpp
    )
target_include_directories(fast_gmm PUBLIC)
target_link_libraries(fast_gmm Eigen3::Eigen yaml-cpp pthread)

#if (BUILD_TESTING)
#  file(GLOB_RECURSE MODULE_TEST_SOURCES tests test_*.cpp)
#  target_sources(run_tests PRIVATE ${MODULE_TEST_SOURCES})
#  target_include_directories(run_tests PUBLIC ${Eigen3_INCLUDE_DIRS})
#  target_link_libraries(run_tests fast_gmm yaml-cpp)
#  target_compile_definitions(run_tests PRIVATE LEARNING_TEST_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/")
#endif ()