target_sources(lagsDS
        PUBLIC
        GMM.cpp
        GPRwrap.cpp
        lagsDS.cpp
        utils.cpp
        )
target_include_directories(lagsDS PUBLIC)
target_link_libraries(lagsDS Eigen3::Eigen yaml-cpp pthread gaussian-process-regression armadillo mathlib)