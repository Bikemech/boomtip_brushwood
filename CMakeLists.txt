cmake_minimum_required(VERSION 3.0.2)
project(boomtip_brushwood)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_geometry_msgs
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(main
        src/bt_sequence/main.cpp
        src/bt_sequence/task_handler/task.cpp
        src/bt_sequence/task_handler/obstacle.cpp
        src/bt_sequence/task_handler/task_manager.cpp
        )

target_link_libraries(main ${catkin_LIBRARIES})