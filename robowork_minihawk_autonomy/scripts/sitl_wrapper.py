# !/usr/bin/env python

# Wrapper file used to launch the Gazebo SITL simulation

# libraries
import rospy
import sys

# class
class SitlWrapper():
    def __init__(self):
        pass
    
    def run(self):
        pass

# main
def main():
    #try initializing
    try:
        #connect as a rospy node named "sitl_wrapper"
        rospy.init_node("sitl_wrapper")
        #initialize the 
        sitl = SitlWrapper()
    except (ValueError, RuntimeError) as exc:
        rospy.logerr("Failed to initialize node: %s", exc)
        sys.exit(1)

    #try running the simulation app
    try:
        sitl.run()
    except (ValueError, RuntimeError) as exc:
        rospy.logerr("Failed to launch ArduPilot SITL: %s", exc)
        sys.exit(1)

if __name__ == "__main__":
    main()