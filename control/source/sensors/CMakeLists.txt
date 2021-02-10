target_sources(sensors
    PRIVATE
    NetFTRDTDriver.cpp
    PUBLIC
    ForceTorqueSensor.cpp
    )
target_include_directories(sensors PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(sensors ${Boost_LIBRARIES} state_representation)