cmake_minimum_required(VERSION 3.13)
set(CMAKE_CXX_STANDARD 17)

project(CITRIFIED VERSION 0.1)

find_package(Eigen3 REQUIRED)
find_package(cppzmq REQUIRED)

include_directories(include)

add_subdirectory(lib)
add_subdirectory(source)
add_subdirectory(executables)