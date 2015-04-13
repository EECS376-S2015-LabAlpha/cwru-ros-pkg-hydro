cmake_minimum_required(VERSION 2.8.3)
project(pc_processing)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   

# Executables
# cs_add_executable(example src/example.cpp)
#cs_add_executable(process_pcl src/process_pcl.cpp)
#cs_add_executable(process_pcl_v2 src/process_pcl_v2.cpp)
#cs_add_executable(pcd_save src/pcd_save.cpp)
#cs_add_executable(process_pcd src/process_pcd.cpp)
#cs_add_executable(process_pcd_dev src/process_pcd_dev.cpp)
cs_add_executable(find_can src/find_can.cpp)

# Target Libraries
# target_link_library(example my_lib)
target_link_libraries(process_pcl ${PCL_LIBRARIES})
target_link_libraries(process_pcl_v2 ${PCL_LIBRARIES})
target_link_libraries(pcd_save ${PCL_LIBRARIES}) #don't need this?

cs_install()
cs_export()
    