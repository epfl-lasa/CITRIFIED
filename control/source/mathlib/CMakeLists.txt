file(GLOB_RECURSE MODULE_TEST_SOURCES *.cpp)
target_sources(mathlib
    PUBLIC
    ${MODULE_TEST_SOURCES}
    )
target_include_directories(mathlib PUBLIC)
target_link_libraries(mathlib Eigen3::Eigen yaml-cpp pthread armadillo)

#if (BUILD_TESTING)
#  file(GLOB_RECURSE MODULE_TEST_SOURCES tests test_*.cpp)
#  target_sources(run_tests PRIVATE ${MODULE_TEST_SOURCES})
#  target_include_directories(run_tests PUBLIC ${Eigen3_INCLUDE_DIRS})
#  target_link_libraries(run_tests fast_gmm yaml-cpp)
#  target_compile_definitions(run_tests PRIVATE LEARNING_TEST_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/")
#endif ()