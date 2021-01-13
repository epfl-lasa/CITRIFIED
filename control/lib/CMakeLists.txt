add_library(passive-ds-control /usr/local/src/passive-ds-control/passive_ds_controller.cpp)
target_include_directories(passive-ds-control PRIVATE /usr/local/include/passive-ds-control)
target_link_libraries(passive-ds-control Eigen3::Eigen)