target_sources(filters
    PUBLIC
    DigitalButterworth.cpp)
target_include_directories(filters PUBLIC)

if (BUILD_TESTING)
  file(GLOB_RECURSE MODULE_TEST_SOURCES tests test_*.cpp)
  target_sources(run_tests PRIVATE ${MODULE_TEST_SOURCES})
  target_include_directories(run_tests PUBLIC ${Eigen3_INCLUDE_DIRS})
  target_link_libraries(run_tests filters yaml-cpp)
  target_compile_definitions(run_tests PRIVATE FILTER_TEST_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/")
endif ()