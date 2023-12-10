project(ros_custom_mes)
if(CMAKE_MW MATCHES ROS)
  find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge pcl_ros std_msgs message_generation)
  add_message_files(DIRECTORY msg
                    FILES Oem7Header.msg
                          PositionOrVelocityType.msg
                          INSExtendedSolutionStatus.msg
                          InertialSolutionStatus.msg
                          INSPVAX.msg
                          CameraData.msg
                          lanefusionDebug.msg)
  generate_messages(DEPENDENCIES std_msgs)
  catkin_package(CATKIN_DEPENDS roscpp std_msgs message_runtime)
endif()

add_subdirectory(test)