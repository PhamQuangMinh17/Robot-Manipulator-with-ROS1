cmake_minimum_required(VERSION 3.0.2)
project(minh_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib 
  std_msgs
  message_generation
  controller_manager
  hardware_interface
  moveit_ros_planning_interface
)

add_service_files(
  FILES
  AngleConvert_IF.srv
)
add_action_files(
  FILES
  CommandIF.action
)
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    controller_manager
    hardware_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(anglesconvert_services src/anglesconvert_services.cpp)
add_dependencies(anglesconvert_services ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(anglesconvert_services ${catkin_LIBRARIES})

add_executable(robot_interface src/robot_interface.cpp)
add_dependencies(robot_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_interface ${catkin_LIBRARIES})

add_executable(cmd_server src/cmd_server.cpp)
add_dependencies(cmd_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cmd_server ${catkin_LIBRARIES})

add_executable(cmd_client src/cmd_client.cpp)
add_dependencies(cmd_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cmd_client ${catkin_LIBRARIES})