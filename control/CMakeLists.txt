cmake_minimum_required(VERSION 3.13)
set(CMAKE_CXX_STANDARD 17)

project(CITRIFIED VERSION 0.2)

enable_testing()

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen3 REQUIRED)
find_package(cppzmq REQUIRED)

if(BUILD_TESTING)
    find_package(GTest REQUIRED)
else()
    find_package(GTest QUIET)
endif()

find_library(state_representation REQUIRED)
find_library(dynamical_systems REQUIRED)
find_library(controllers REQUIRED)
find_library(limbo REQUIRED)

include_directories(include ${Eigen_INCLUDE_DIRS})

add_subdirectory(lib)
add_subdirectory(source)
add_subdirectory(executables)
