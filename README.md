# aerial_robotics

fork of https://github.com/robowork/aerial_robotics

original readme is at README_original.md

goals:

source the catkin workspace environment
launch ardupilot gazebo sitl
launch rviz
load waypoints in gazebo sitl

create a ros node
in that node
	- call a ros service that puts the vehicle into auto mode
	- call a ros service that arms the vehicle
	- subscribe a ros topic to find the apriltag detections
	- publish the ros topic once it has apriltag detections
	- listen to those topics interpret them as orientation and xyz offsets to do control commands
	- control commands will be sent out by a publisher
	- invoke a service in order to make the vehicle mode to go to loiter to lkisten to contorl commands

currently we have nothing

