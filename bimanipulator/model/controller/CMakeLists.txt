cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(controller main.cpp ../protoout/user_message.pb.cc)
target_include_directories(controller PRIVATE ../protoout)
target_link_libraries(controller ${GAZEBO_LIBRARIES} nos igris crow tbb)