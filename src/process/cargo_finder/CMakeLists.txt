cmake_minimum_required(VERSION 2.8.3)
project(cargo_finder_proc)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        process_interface
        geometry_msgs
        tf
        move_base_msgs
        actionlib
        )

set(CMAKE_CXX_FLAGS "-std=c++14")

include_directories(${catkin_INCLUDE_DIRS} include)
add_library(${PROJECT_NAME}_class STATIC
        include/cargo_finder.hpp src/cargo_finder.cpp)


add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_class ${catkin_LIBRARIES})