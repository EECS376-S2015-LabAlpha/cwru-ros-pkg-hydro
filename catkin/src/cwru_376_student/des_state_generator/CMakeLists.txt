cmake_minimum_required(VERSION 2.8.3)
project(des_state_generator)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# C++0x support - not quite the same as final C++11!
 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(destination_state_generator src/des_state_generator.cpp)
cs_add_executable(path_sender src/path_sender.cpp)
cs_add_executable(path_sender_starting_pen src/path_sender_starting_pen.cpp)
# cs_add_executable(example src/example.cpp)
# target_link_library(example my_lib)

cs_install()
cs_export()
    
