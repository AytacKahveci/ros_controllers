cmake_minimum_required(VERSION 2.8.3)
project(coil_state_controller)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS 
                             realtime_tools 
                             roscpp 
                             hardware_interface 
                             pluginlib 
                             controller_interface 
                             sensor_msgs 
                             coil_states_msgs)

catkin_package(
  CATKIN_DEPENDS realtime_tools roscpp hardware_interface pluginlib controller_interface sensor_msgs coil_states_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/coil_state_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} coil_states_msgs)


# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES coil_state_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
