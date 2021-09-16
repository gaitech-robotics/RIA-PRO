cmake_minimum_required(VERSION 2.8.3)
project(ria_pro_msgs)

find_package(catkin REQUIRED COMPONENTS 
  message_generation
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
)

add_message_files(
  FILES
  Command.msg
  Feedback.msg
  ControllerState.msg
  MotorState.msg
  GPIO.msg
  IR.msg
)

add_service_files(
  FILES
  Reset.srv
)

add_action_files(
  FILES
  Dock.action
  Undock.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

install(
  DIRECTORY action msg srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)