target_sources(logger PUBLIC json_logger.cpp)
target_link_libraries(logger state_representation)

if (BUILD_TESTING)
  file(GLOB_RECURSE MODULE_TEST_SOURCES tests test_*.cpp)
  target_sources(run_tests PRIVATE ${MODULE_TEST_SOURCES})
  target_link_libraries(run_tests logger)
  target_compile_definitions(run_tests PRIVATE LOGGER_TEST_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/")
endif ()