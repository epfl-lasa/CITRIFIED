if (BUILD_TESTING)
  link_directories(/usr/local/lib)
  add_executable(run_tests run_tests.cpp)
  target_link_libraries(run_tests ${GTEST_LIBRARIES} pthread)
  add_test(NAME run_tests COMMAND run_tests)
endif ()

add_library(motion_generators STATIC "")
add_subdirectory(motion_generators)

add_library(local_controllers STATIC "")
add_subdirectory(controllers)

add_library(sensors STATIC "")
add_subdirectory(sensors)

add_library(filters STATIC "")
add_subdirectory(filters)

add_library(learning STATIC "")
add_subdirectory(learning)

add_library(logger STATIC "")
add_subdirectory(logger)

add_library(fast_gmm STATIC "")
add_subdirectory(fast_gmm)

add_library(mathlib STATIC "")
add_subdirectory(mathlib)