cmake_minimum_required(VERSION 2.8.3)
project(coil_pid_controller)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_interface
  hardware_interface
  realtime_tools
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES coil_pid_controller
  CATKIN_DEPENDS control_toolbox controller_interface hardware_interface realtime_tools roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include 
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS} 
  hardware_interface
)

add_library(${PROJECT_NAME} src/coil_pid_controller.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(FILES ${PROJECT_NAME}_plugins.xml 
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
