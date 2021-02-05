target_sources(lib_sensors
    PRIVATE
    NetFTRDTDriver.cpp
    PUBLIC
    ForceTorqueSensor.cpp
    )
target_include_directories(lib_sensors PUBLIC ${Eigen3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(lib_sensors ${Boost_LIBRARIES} state_representation)