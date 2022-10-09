cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(bimanip)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(remotectr SHARED 
    remotectr.cc
    protoout/user_message.pb.cc
)

add_custom_command(OUTPUT protoout/user_message.pb.cc
    COMMAND protoc user_message.proto --cpp_out protoout
    DEPENDS user_message.proto
)

target_include_directories(remotectr PRIVATE protoout)
target_link_libraries(remotectr ${GAZEBO_LIBRARIES} nos igris crow 
    mosquittopp
    mosquitto_client)
