cmake_minimum_required(VERSION 2.8.3)
project(simple_arm_server)

find_package(catkin REQUIRED genmsg geometry_msgs actionlib actionlib_msgs)

add_message_files(FILES
                  ArmAction.msg
                 )

add_service_files(FILES
                  GetTrajectoryValidity.srv
                 )

add_action_files(FILES
                  MoveArm.action
                 )

generate_messages(DEPENDENCIES geometry_msgs actionlib_msgs)

catkin_package(DEPENDS geometry_msgs)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY nodes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
