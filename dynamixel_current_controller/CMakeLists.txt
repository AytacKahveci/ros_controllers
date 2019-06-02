cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_current_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_interface
  hardware_interface
  realtime_tools
  roscpp
  std_msgs
  pluginlib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS control_toolbox controller_interface hardware_interface realtime_tools roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
 ${Boost_INCLUDE_DIR}
 ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/dynamixelHardwareInterfaceCurrent.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES dynamixel_current_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})