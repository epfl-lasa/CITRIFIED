target_sources(lib_motion_gen
        PRIVATE
            BaseDS.cpp
        PUBLIC
            CircularDS.cpp
        RingDS.cpp
            PointAttractorDS.cpp)
target_include_directories(lib_motion_gen PUBLIC ${Eigen3_INCLUDE_DIRS})
target_link_libraries(lib_motion_gen Eigen3::Eigen cppzmq state_representation dynamical_systems)