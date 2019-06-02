cmake_minimum_required(VERSION 2.8.3)
project(poseff_controllers)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  controller_interface
  forward_command_controller
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES poseff_controllers
  CATKIN_DEPENDS pluginlib roscpp forward_command_controller controller_interface
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/joint_group_poseff_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${CATKIN_LIBRARIES})

install(FILES ${PROJECT_NAME}_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)