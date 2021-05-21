#target_sources(gaussian-process-regression
#
#        )
target_include_directories(gaussian-process-regression PUBLIC)
target_link_libraries(gaussian-process-regression Eigen3::Eigen yaml-cpp pthread armadillo mathlib)