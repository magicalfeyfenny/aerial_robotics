#!/usr/bin/env python

#libraries
import sys
import rospy

from mavros_msgs.msg import State
from mavros_msgs.msg import WaypointList
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool

from apriltag_ros.msg import AprilTagDetectionArray

#class definition
class AutoArmNode(object):
    #grab params, subscribe and publish topics
    def __init__(self):
        #pull params from node
        robot_namespace = rospy.get_param("~robot_namespace")
        self.robot_namespace = self._normalize_namespace(robot_namespace)
        self.tick_rate = rospy.get_param("~tick_rate") #in hz
        self.timeout = rospy.get_param("~timeout")
        self.camera_frame = str(rospy.get_param("~camera_frame"))
        self.target_tag_id = int(rospy.get_param("~target_tag_id"))

        #mavros state variables
        self.state = None
        self.mission = None
        self.tag_detection = None
        self.tag_detection_time = None

        #set paths for mavros topics/services
        state_topic = self._mavros_name("state")
        mission_topic = self._mavros_name("mission/waypoints")
        set_mode_service = self._mavros_name("set_mode")
        arm_service = self._mavros_name("cmd/arming")
        rc_override_topic = self._mavros_name("rc/override")
        tag_detection_topic = self._tag_detection_name()

        #subscribe to topics
        rospy.loginfo("Subscribing to MAVROS topics")
        rospy.Subscriber(state_topic, State, self._state_cb, queue_size=1)
        rospy.Subscriber(mission_topic, WaypointList, self._mission_cb, queue_size=1)
        rospy.Subscriber(tag_detection_topic, AprilTagDetectionArray, self._tag_detection_cb, queue_size=1)

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

    #run launch sequence        
    def run(self):
        self._wait_delay(10) #race condition with connecting to subscriber
        self._connect_to_FCU()
        self._get_waypoints()
        self._mode_auto()
        self._arm_takeoff()
        self._detect_tags()
        self._mode_loiter()
        self._move_over_tag()
        self._initiate_landing()

## mission sequence

    #connect to the FCU
    def _connect_to_FCU(self):
        rospy.loginfo("Connecting to FCU . . .")

        rate = rospy.Rate(self.tick_rate) #tick rate

        while not rospy.is_shutdown():
            if self.state is not None:
                if self.state.connected:
                    rospy.loginfo("Connected to FCU!")
                    return

            rate.sleep()

    #make sure waypoints list exists
    def _get_waypoints(self):
        rospy.loginfo("Checking mission waypoints . . .")

        rate = rospy.Rate(self.tick_rate)

        while not rospy.is_shutdown():
            if self.mission is not None:
                #if subscriber has gotten waypoint list, it would have put it in _mission_cb
                if self.mission.waypoints:
                    rospy.loginfo("Found %d waypoints!", len(self.mission.waypoints))
                    return

            rate.sleep()

    #set AUTO mode
    def _mode_auto(self):
        rospy.loginfo("Enabling AUTO mode . . .")

        rate = rospy.Rate(self.tick_rate)

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
            
            rate.sleep()

    #arm vehicle
    def _arm_takeoff(self):
        rospy.loginfo("Arming vehicle for takeoff . . .")

        rate = rospy.Rate(self.tick_rate)

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
            
            rate.sleep()

    #search for the AprilTag
    def _detect_tags(self):
        rospy.loginfo("Beginning tag search . . .")
        rate = rospy.Rate(self.tick_rate)

        while not rospy.is_shutdown():
            if self.tag_detection and self.tag_detection_time:
                rospy.loginfo("Found AprilTag with ID %d", self.target_tag_id)
                return

            rate.sleep()

    #set QLOITER mode
    def _mode_loiter(self):
        rospy.loginfo("Enabling QLOITER mode . . .")

        rate = rospy.Rate(self.tick_rate)

        #set sticks to neutral
        rospy.loginfo("Setting position to neutral")
        self._publish_rc_override(1500, 1500, 1500, 1500)


        while not rospy.is_shutdown():
            #send a request every tick
            request = self.set_mode(base_mode=0, custom_mode="QLOITER")
            if not request.mode_sent:
                rospy.logwarn("QLOITER mode request rejected by MAVROS.")
            else:
                rospy.loginfo("QLOITER mode request sent to MAVROS.")

            #confirm if it's reached the correct mode
            if self.state is not None:
                if self.state.mode == "QLOITER":
                    rospy.loginfo("QLOITER mode enabled!")
                    return
            
            rate.sleep()

    #hover directly over tag
    def _move_over_tag(self):
        pass

    #land
    def _initiate_landing(self):
        pass

## rc override

    #publish channels with specified roll/pitch/throttle/yaw
    def _publish_rc_override(self, roll, pitch, throttle, yaw):
        msg = OverrideRCIn()
        msg.channels = [ roll, pitch, throttle, yaw, 1800, 1000, 1000, 1800 ].append(
                        [OverrideRCIn.CHAN_RELEASE] * 10 )
        self.rc_override.publish(msg)

    #release all channels
    def _release_rc_override(self):
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_RELEASE] * 18
        self.rc_override.publish(msg)

## file paths

    #convert from names here to mavros name
    def _mavros_name(self, suffix):
        base_name = "/" + self.robot_namespace + "/mavros"
        if not suffix:
            return base_name
        return base_name + "/" + suffix

    #path to the camera
    def _tag_detection_name(self):
        name = "/" + self.robot_namespace + "/" + self.camera_frame + "/tag_detections"
        return name

    #convert from mavros name to names here
    def _normalize_namespace(self, robot_namespace):
        # remove whitespace and surrounding slashes
        value = str(robot_namespace).strip()
        if not value:
            raise ValueError("~robot_namespace cannot be empty.")
        return value.strip("/")

## state callbacks

    #called with state subscriber, sets self.state to msg
    def _state_cb(self, msg):
        self.state = msg

    #called with mission subscriber, sets self.mission to msg
    def _mission_cb(self, msg):
        self.mission = msg

    #called with tag detection subscriber, sets self.tag_detection and self.tag_detection_time
    def _tag_detection_cb(self, msg):
        tag_found = None
        for detection in msg.detections:
            #compare each apriltag against the ID of the specified node tag ID, use first match
            for tag_id in detection.id:
                if tag_id == self.target_tag_id:
                    tag_found = detection
                    break
        
        #if no tag found, report nothing
        if tag_found is None:
            self.tag_detection = None
            self.tag_detection_time = None
            return

        #otherwise, we want the tag position
        position = tag_found.pose.pose.pose.position
        self.tag_detection = {
            "x": position.x,
            "y": position.y,
            "z": position.z
        }
        self.tag_detection_time = rospy.Time.now()

## other helpers

    #generic delay for a specified amount of time
    def _wait_delay(self, delay_secs):
        deadline = rospy.Time.now() + rospy.Duration.from_sec(delay_secs)
        rate = rospy.Rate(self.tick_rate) #hz
        #sleep for 1/hz secs repeatedly until either rospy shuts down or the delay_secs timer is up
        while not rospy.is_shutdown():
            if rospy.Time.now() >= deadline:
                return
            rate.sleep()

# minimal main
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