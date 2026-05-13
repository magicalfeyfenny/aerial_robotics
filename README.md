# aerial_robotics

fork of https://github.com/robowork/aerial_robotics

original readme is at README_original.md

our work is in robowork_minihawk_autonomy

how to build:
-	clone this into $HOME/aerial_robotics_ws/src/aerial_robotics
-	source $HOME/aerial_robotics_ws/devel/setup.bash
-	cd $HOME/aerial_robotics_ws/
-	catkin build

how to run:
-	source $HOME/aerial_robotics_ws/devel/setup.bash
-	roslaunch robowork_minihawk_autonomy sim_auto.launch

sim_auto.launch
-	launches gazebo and MAVROS through their .launch files
-	create a node that launches SITL with sitl_wrapper.py
-	launches auto_arm.launch

auto_arm.launch
-	subscribes to mission and state topics
-	connects to arm and set_mode services
-	publishes an RC override topic if we need to manually reposition
-	confirms connection to FCU
-	confirms mission waypoint list
-	enables autopilot
-	arms vehicle for takeoff