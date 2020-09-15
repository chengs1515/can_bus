cmake_minimum_required(VERSION 3.0.2)
add_compile_options(-std=c++11) 
project(can_bus)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    autoware_msgs
    autoware_can_msgs
    )
catkin_package()

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(can_start 
   ./src/byte.cpp 
   ./src/can_bus_node.cpp 
   ./src/CANBus.cpp 
   ./src/EsdCAN.cpp
   ./src/Interpolation2d.cpp
)

add_executable(teleop ./src/teleop.cc)

 target_link_libraries(can_start
  ${catkin_LIBRARIES}
  ntcan
 )
 
 target_link_libraries(teleop 
 ${catkin_LIBRARIES}
 )