cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(world SHARED world.cc)
target_link_libraries(world ${GAZEBO_LIBRARIES} nos igris crow rabbit ralgo)

add_library(cow SHARED cow.cc)
target_link_libraries(cow ${GAZEBO_LIBRARIES} nos igris crow rabbit ralgo)
