cmake_minimum_required (VERSION 3.0.0)

project(optimize)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

# set default to release
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
add_compile_options(-O3)

# flightlib
include(cmake/yaml.cmake)
include_directories(../../../flightlib/include)
find_library(FLIGHTLIB flightlib ../../../flightlib/build)

# flightlib deps
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(optimizer src/optimize.cpp)
target_link_libraries(optimizer ${catkin_LIBRARIES} ${FLIGHTLIB} yaml-cpp)