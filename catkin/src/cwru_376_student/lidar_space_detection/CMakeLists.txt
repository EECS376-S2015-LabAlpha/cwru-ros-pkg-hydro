cmake_minimum_required(VERSION 2.8.3)
project(lidar_space_detection)

find_package(catkin_simple REQUIRED COMPONENTS
  roscpp
  tf
  geometry_msgs
  message_generation
  nav_msgs
  std_msgs
)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(lidar_space_detection src/lidar_space.cpp)
# target_link_library(example my_lib)

cs_install()
cs_export()
    
