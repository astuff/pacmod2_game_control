cmake_minimum_required(VERSION 2.8.3)
project(pacmod2_game_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roslint
  roscpp
  std_msgs
  sensor_msgs
  pacmod_msgs
  joy
)

catkin_package(CATKIN_DEPENDS
  std_msgs
  sensor_msgs
  pacmod_msgs
  joy
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pacmod2_game_control_node
  src/pacmod_game_control_node.cpp
  src/startup_checks.cpp
  src/publish_control_board_rev2.cpp
  src/publish_control_board_rev3.cpp
  src/publish_control.cpp
  src/publish_control_factory.cpp
)

add_dependencies(pacmod2_game_control_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(pacmod2_game_control_node
  ${catkin_LIBRARIES}
)

set(ROSLINT_CPP_OPTS "--filter=-build/namespaces,-build/c++11")
roslint_cpp()

install(TARGETS pacmod2_game_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
