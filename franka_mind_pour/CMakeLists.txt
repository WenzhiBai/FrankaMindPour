cmake_minimum_required(VERSION 3.4)
project(franka_mind_pour)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  message_generation
  pluginlib
  realtime_tools
  roscpp
  std_msgs
)

add_message_files(
  FILES
  PourCommand.msg
  PourState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    mind_pour_controller
  CATKIN_DEPENDS
    controller_interface
    hardware_interface
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    std_msgs
)

# mind_pour_controller: the franka_hw controller plugin
file(GLOB_RECURSE MIND_POUR_CONTROLLER_SRC src/*.cpp)
add_library(mind_pour_controller ${MIND_POUR_CONTROLLER_SRC})

add_dependencies(mind_pour_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mind_pour_controller PUBLIC
  ${catkin_LIBRARIES}
)

target_include_directories(mind_pour_controller SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(mind_pour_controller PUBLIC
  include
)

## Installation
install(TARGETS
  mind_pour_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES franka_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(PROGRAMS
  scripts/udp_bridge.py
  scripts/session_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/record_bag.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
