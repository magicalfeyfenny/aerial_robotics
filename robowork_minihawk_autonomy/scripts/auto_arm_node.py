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
        self.service_timeout = rospy.get_param("~service_timeout")

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
        rospy.wait_for_service(set_mode_service, timeout=self.service_timeout)
        rospy.wait_for_service(arm_service, timeout=self.service_timeout)

        #proxy services
        rospy.loginfo("Services found, proxying")
        self.set_mode = rospy.ServiceProxy(set_mode_service, SetMode)
        self.arm = rospy.ServiceProxy(arm_service, CommandBool)

        #publish RC override topic
        self.rc_override = rospy.Publisher(rc_override_topic, OverrideRCIn, queue_size=1)
        rospy.on_shutdown(self._release_rc_override)
        

    def run(self):
        pass

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

    #called with state subscriber
    def _state_cb(self, msg):
        self.state = msg

    #called with mission subscriber
    def _mission_cb(self, msg):
        self.mission = msg


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