# !/usr/bin/env python

# Wrapper file used to launch the Gazebo SITL simulation

# libraries
import rospy
import sys
import os
import subprocess
import time
import signal

# class
class SitlWrapper():
    def __init__(self):
        self.ardupilot_dir = rospy.get_param("~ardupilot_dir")
        self.sim_vehicle = rospy.get_param("~sim_vehicle")
        self.vehicle = rospy.get_param("~vehicle")
        self.frame = rospy.get_param("~frame")
        self.model = rospy.get_param("~model")
        self.console = bool(rospy.get_param("~console", False))
        self.show_map = bool(rospy.get_param("~map", False))
        self.process = None
    
    def run(self):
        #build command out of __init__ params
        command = [
            self.sim_vehicle,
            "-v", self.vehicle,
            "-f", self.frame,
            "--model", self.model,
        ]
        if self.console:
            command.append("--console")
        if self.show_map:
            command.append("--map")

        # launch SITL process
        rospy.loginfo("Starting ArduPilot SITL: %s", " ".join(command))
        self.process = subprocess.Popen( 
            command,
            cwd=self.ardupilot_dir,
            preexec_fn=os.setsid,
        )

        # wait for SITL process to shut down
        rospy.on_shutdown(self._shutdown_child)
        return_code = self.process.wait()
        if rospy.is_shutdown():
            return
        raise RuntimeError("ArduPilot SITL exited with status %s." % return_code)

    def _shutdown_child(self):
        #exit if the process is already exited
        if self.process is None:
            return
        if self.process.poll() is not None:
            return
        
        #try to SIGINT first
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
        except OSError:
            return
        
        #if the SIGINT doesn't report an error but it's taking too long, SIGTERM it
        try:
            timeout = 10.0
            check_wait = 1
            deadline = time.time() + timeout
            while time.time() < deadline:
                #stop trying if process exited
                if self.process.poll() is not None:
                    return
                time.sleep(check_wait)
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
        except OSError:
            return

    def _validate(self):
        #make sure we're in the right directory
        if not self.ardupilot_dir:
            raise ValueError("~ardupilot_dir cannot be empty.")
        if not os.path.isdir(self.ardupilot_dir):
            raise ValueError("ArduPilot directory does not exist: %s" % self.ardupilot_dir)

        #make sure the sim_vehicle.py file exists in the ardupilot_dir directory
        executable_path = self.sim_vehicle
        if not os.path.isabs(executable_path):
            executable_path = os.path.join(self.ardupilot_dir, executable_path)
        if not os.path.isfile(executable_path):
            raise ValueError("sim_vehicle.py not found: %s" % executable_path)
        
        #make sure the other parameters exist
        if not self.vehicle:
            raise ValueError("~vehicle cannot be empty.")
        if not self.frame:
            raise ValueError("~frame cannot be empty.")
        if not self.model:
            raise ValueError("~model cannot be empty.")

# main
def main():
    #try initializing
    try:
        rospy.init_node("sitl_wrapper")
        sitl = SitlWrapper()
    except (ValueError, RuntimeError) as exc:
        rospy.logerr("Failed to initialize node: %s", exc)
        sys.exit(1)

    #try running the simulation app
    try:
        sitl._validate()
        sitl.run()
    except (ValueError, RuntimeError) as exc:
        rospy.logerr("Failed to launch ArduPilot SITL: %s", exc)
        sys.exit(1)

if __name__ == "__main__":
    main()