if(BUILD_TESTING)
    add_executable(run_tests run_tests.cpp controllers/tests/test_kdl_robot.cpp)
    target_link_libraries(run_tests ${GTEST_LIBRARIES} pthread)
    add_test(NAME run_tests COMMAND run_tests)
endif()

add_library(motion_generators STATIC "")
add_subdirectory(motion_generators)

add_library(controllers STATIC "")
add_subdirectory(controllers)

add_library(sensors STATIC "")
add_subdirectory(sensors)