cmake_minimum_required(VERSION 2.8.3)
project(navigation_class)
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

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   )



#Add the google test subdirectory
#add_subdirectory(/home/user/GTests)

#include the googlemock/include dir
#include_directories(/home/user/GTests/googlemock/include)


set(CMAKE_CXX_FLAGS "-std=c++14")

include_directories(${catkin_INCLUDE_DIRS} include)
add_library(navigation_class
        include/navigation_class.hpp src/navigation_class.cpp)


add_executable(navigation_graph src/main.cpp)
target_link_libraries(navigation_class ${catkin_LIBRARIES})

#Link with GoogleTest
#target_link_libraries(navigation_graph navigation_class gtest gtest_main)
#Link with GoogleMock
#target_link_libraries(navigation_graph navigation_class gmock gmock_main)



