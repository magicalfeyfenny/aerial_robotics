#!/usr/bin/env python

#libraries
import sys
import rospy

from mavros_msgs.msg import State
from mavros_msgs.msg import WaypointList
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool

#class definition
class AutoArmNode(object):
    def __init__(self):
        #pull params from node
        robot_namespace = rospy.get_param("~robot_namespace")
        self.robot_namespace = self._normalize_namespace(robot_namespace)
        self.timeout = rospy.get_param("~timeout")
        self.rate_freq = rospy.get_param("~rate_freq")

        #cache mavros state
        self.state = None
        self.mission = None

        #resolve mavros topics
        state_topic = self._mavros_name("state")
        mission_topic = self._mavros_name("mission/waypoints")
        set_mode_service = self._mavros_name("set_mode")
        arm_service = self._mavros_name("cmd/arming")
        rc_override_topic = self._mavros_name("rc/override")

        #subscribe to topics
        rospy.loginfo("Subscribing to MAVROS topics")
        rospy.Subscriber(state_topic, State, self._state_cb, queue_size=1)
        rospy.Subscriber(mission_topic, WaypointList, self._mission_cb, queue_size=1)

        #wait for services
        rospy.loginfo("Waiting for MAVROS services")
        rospy.wait_for_service(set_mode_service, timeout=self.timeout)
        rospy.wait_for_service(arm_service, timeout=self.timeout)

        #proxy services
        rospy.loginfo("Services found, proxying")
        self.set_mode = rospy.ServiceProxy(set_mode_service, SetMode)
        self.arm = rospy.ServiceProxy(arm_service, CommandBool)

        #publish RC override topic
        rospy.loginfo("Publishing RC override topic")
        self.rc_override = rospy.Publisher(rc_override_topic, OverrideRCIn, queue_size=1)
        rospy.on_shutdown(self._release_rc_override)

        rospy.loginfo("Initialization complete.")
        

    def run(self):
        self._connect_to_FCU()
        self._get_waypoints()
        self._enable_autopilot()
        self._arm_takeoff()

    def _connect_to_FCU(self):
        rospy.loginfo("Connecting to FCU . . .")
        deadline = rospy.Time.now() + rospy.Duration.from_sec(self.timeout)
        rate = rospy.Rate(self.rate_freq) #hz

        while not rospy.is_shutdown():
            if self.state is not None:
                if self.state.connected:
                    rospy.loginfo("Connected to FCU!")
                    return
                
            if rospy.Time.now() >= deadline:
                rospy.logerr("Could not connect to FCU.")

            rate.sleep()
            
    def _get_waypoints(self):
        rospy.loginfo("Checking mission waypoints . . .")

        deadline = rospy.Time.now() + rospy.Duration.from_sec(self.timeout)
        rate = rospy.Rate(self.rate_freq)

        while not rospy.is_shutdown():
            if self.mission is not None:
                #if subscriber has gotten waypoint list, it would have put it in _mission_cb
                if len(self.mission.waypoints > 0):
                    rospy.loginfo("Found %d waypoints!", len(self.mission.waypoints))
                    return
                
            if rospy.Time.now() >= deadline:
                rospy.logerr("Could not find waypoints.")

            rate.sleep()

    def _enable_autopilot(self):
        rospy.loginfo("Enabling AUTO mode . . .")

        deadline = rospy.Time.now() + rospy.Duration.from_sec(self.timeout)
        rate = rospy.Rate(self.rate_freq)

        while not rospy.is_shutdown():
            #send a request every tick
            request = self.set_mode(base_mode=0, custom_mode="AUTO")
            if not request.mode_sent:
                rospy.logwarn("AUTO mode request rejected by MAVROS.")
            else:
                rospy.loginfo("AUTO mode request sent to MAVROS.")

            #confirm if it's reached the correct mode
            if self.state is not None:
                if self.state.mode == "AUTO":
                    rospy.loginfo("AUTO mode enabled!")
                    return
                
            if rospy.Time.now() >= deadline:
                rospy.logerr("Could not enable AUTO mode.")
            
            rate.sleep()

    def _arm_takeoff(self):
        rospy.loginfo("Arming vehicle for takeoff . . .")

        deadline = rospy.Time.now() + rospy.Duration.from_sec(self.timeout)
        rate = rospy.Rate(self.rate_freq)

        while not rospy.is_shutdown():
            #send arm request every tick
            request = self.arm(value=True)
            if not request.success:
                rospy.logwarn("Takeoff request rejected by MAVROS.")
            else:
                rospy.loginfo("Takeoff request sent to MAVROS.")
            
            if self.state is not None:
                if self.state.armed:
                    rospy.loginfo("Vehicle armed, initiating takeoff!")
                    return
            
            if rospy.Time.now() >= deadline:
                rospy.logerr("Could not arm vehicle for takeoff.")
            
            rate.sleep()


    #remove overwritten channels from self.rc_override
    def _release_rc_override(self):
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_RELEASE] * 18
        self.rc_override.publish(msg)

    #convert from names here to mavros name
    def _mavros_name(self, suffix):
        base_name = "/" + self.robot_namespace + "/mavros"
        if not suffix:
            return base_name
        return base_name + "/" + suffix

    #convert from mavros name to names here
    def _normalize_namespace(self, robot_namespace):
        # remove whitespace and surrounding slashes
        value = str(robot_namespace).strip()
        if not value:
            raise ValueError("~robot_namespace cannot be empty.")
        return value.strip("/")

    #called with state subscriber, sets self.state to msg
    def _state_cb(self, msg):
        self.state = msg

    #called with mission subscriber, sets self.mission to msg
    def _mission_cb(self, msg):
        self.mission = msg

    #generic delay for a specified amount of time
    def _wait_delay(self, delay_secs):
        deadline = rospy.Time.now() + rospy.Duration.from_sec(delay_secs)
        rate = rospy.Rate(self.rate_freq) #hz
        #sleep for 1/hz secs repeatedly until either rospy shuts down or the delay_secs timer is up
        while not rospy.is_shutdown():
            if rospy.Time.now() >= deadline:
                return
            rate.sleep()

#main
def main():
    # init node then handle everything in AutoArmNode
    rospy.init_node("auto_arm_node")
    try:
        #__init__ then run() for the class
        AutoArmNode().run()
    except rospy.ROSException as exc:
        rospy.logerr("ROS error during autonomy sequence: %s", exc)
        sys.exit(1)
    except rospy.ServiceException as exc:
        rospy.logerr("Service error during autonomy sequence: %s", exc)
        sys.exit(1)
    except (RuntimeError, ValueError) as exc:
        rospy.logerr("Autonomy sequence failed: %s", exc)
        sys.exit(1)

if __name__ == "__main__":
    main()