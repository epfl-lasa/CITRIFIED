target_sources(sensors
    PRIVATE
    netft_rdt_driver/NetFTRDTDriver.cpp
    netft_rdt_driver/MockNetFTRDTDriver.cpp
    PUBLIC
    ForceTorqueSensor.cpp
    RigidBodyTracker.cpp
    )
target_include_directories(sensors PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(sensors ${Boost_LIBRARIES} state_representation cppzmq)

add_executable(test_rigid_body_tracker tests/test_rigid_body_tracker_real.cpp)
target_link_libraries(test_rigid_body_tracker sensors)

if (BUILD_TESTING)
  file(GLOB_RECURSE MODULE_TEST_SOURCES tests test_*.cpp)
  target_sources(run_tests PRIVATE ${MODULE_TEST_SOURCES})
  target_include_directories(run_tests PUBLIC ${Eigen3_INCLUDE_DIRS})
  target_link_libraries(run_tests sensors)
endif ()