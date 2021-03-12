target_sources(learning
    PUBLIC
    ESN.cpp
    )
target_include_directories(learning PUBLIC)
target_link_libraries(learning Eigen3::Eigen yaml-cpp)

if (BUILD_TESTING)
  file(GLOB_RECURSE MODULE_TEST_SOURCES tests test_*.cpp)
  if (MODULE_TEST_SOURCES)
    message("Adding motion generator tests")
    target_sources(run_tests PRIVATE ${MODULE_TEST_SOURCES})
    target_include_directories(run_tests PUBLIC ${Eigen3_INCLUDE_DIRS})
    target_link_libraries(run_tests learning)
  endif ()
endif ()