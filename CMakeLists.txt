cmake_minimum_required(VERSION 3.0.2)
project(briit)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mavros
  roscpp
  rospy
  roslaunch
  geometry_msgs
  message_generation
  mavros_msgs
  sensor_msgs
  std_msgs
)
catkin_python_setup()


roslaunch_add_file_check(launch)


add_message_files(
  FILES
  GPSCoordinate.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  message_runtime
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
    src/vtol_api.cpp
 )

target_link_libraries( ${PROJECT_NAME} 
  
  ${catkin_LIBRARIES})

add_executable(run_node 
  src/run.cpp)
add_dependencies(${PROJECT_NAME}
  briit_generate_messages_cpp)
target_link_libraries(run_node 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES})


catkin_install_python(PROGRAMS
  src/briit/run_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  #PATTERN ".svn" EXCLUDE
)
