cmake_minimum_required(VERSION 3.0.2)
project(my_pcl_tutorial)


set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/PCLConfig.cmake")
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)
find_package(PCL REQUIRED COMPONENTS)
find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(


 CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES}  ${PCL_LIBRARIES})
