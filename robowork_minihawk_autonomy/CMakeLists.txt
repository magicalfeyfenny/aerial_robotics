cmake_minimum_required(VERSION 2.8.3)
project(robowork_minihawk_autonomy)

find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
  mavros_msgs
  roslaunch
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/auto_arm_node.py
  scripts/sitl_wrapper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

roslaunch_add_file_check(launch)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
