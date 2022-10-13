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
  geographic_msgs
  nav_msgs
  std_msgs
)


roslaunch_add_file_check(launch)

# Call setup.py
catkin_python_setup()

add_message_files(
  FILES
  GPSCoordinate.msg
)
generate_messages(DEPENDENCIES 
                  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  message_runtime
  roscpp
  rospy
  std_msgs
  geometry_msgs
  geographic_msgs
  sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ Library
add_library(${PROJECT_NAME}
    src/vtol_api.cpp
 )

target_link_libraries( ${PROJECT_NAME} 
  
  ${catkin_LIBRARIES}
)

add_executable(run_node 
  src/run.cpp)
target_link_libraries(run_node 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES})
add_dependencies(run_node
  briit_generate_messages_cpp)

catkin_install_python(PROGRAMS
  src/run_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS run_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  #PATTERN ".svn" EXCLUDE
)
