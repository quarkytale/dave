#!/usr/bin/env python
'''
Maneuver the vehicle like merry-go-round

'''

# System imports
import math

# ROS/Gazebo imports
import rospy
import tf
from rosgraph_msgs.msg import Clock

# For GetModelState Service
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist, Transform

# For lat/lon coordinate frame transformations
from osgeo import ogr
from osgeo import osr

class MerryGoRound:

    def __init__(self):

        # Parse parameters
        self.model_name = "rexrov"
        self.center_lat = 36.800
        self.center_lon = -121.815
        self.radius = 300
        self.depth = -30
        self.angularSpeed = 5

        # Calculate center coordinates in X/Y (UTM; epsg3857)
        source = osr.SpatialReference()
        source.ImportFromEPSG(4326)
        target = osr.SpatialReference()
        target.ImportFromEPSG(3857)
        transform = osr.CoordinateTransformation(source, target)
        point = ogr.CreateGeometryFromWkt("POINT (" + repr(self.center_lat) + " " + repr(self.center_lon) + ")")
        point.Transform(transform)
        self.xCenter = float(point.ExportToWkt().split(" ")[1].split("(")[1])
        self.yCenter = float(point.ExportToWkt().split(" ")[2].split(")")[0])

        # Wait for ROS init
        self.t0 = rospy.get_time()
        while self.t0 < 0.01:
            rospy.logwarn("Waiting for ROS time to be received")
            rospy.sleep(0.5)
            self.t0 = rospy.get_time()
            self.t0_init = self.t0

        # Launch
        self.launch()

    def launch(self):

        # Wait for Gazebo services
        rospy.loginfo("Starting Merry-Go-Round maneuver...")
        rospy.wait_for_service("/gazebo/set_model_state")
        wait_for_sim_to_start = rospy.wait_for_message('/clock', Clock)

        # Prepare initialization SetModelState service
        self.set_model_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        rospy.loginfo("Vehicle will be maneuvered to roate with (center_latitude,center_longitude) = (" \
            + repr(self.center_lat) + "," + repr(self.center_lon) + ") with radius of " + repr(self.radius) \
            + " [m] for " + repr(self.angularSpeed) + " [deg/sim_sec] angular speed at " + repr(self.depth) \
            + " [m] depth")

    def maneuver(self):

        # Timing
        now = rospy.get_time()
        dt = now - self.t0
        if dt < 1.0e-5:
            # rospy.logwarn("Timestep is too small (%f) - skipping this update"
            #               %dt)
            return
        self.t0 = now

        # Generate ModelState msg
        pose = Pose()
        pose.position.x = self.xCenter + self.radius*math.sin(now*self.angularSpeed/180.0*math.pi)
        pose.position.y = self.yCenter + self.radius*math.cos(now*self.angularSpeed/180.0*math.pi)
        pose.position.z = self.depth

        # meneuver
        self.set_model_srv(ModelState(self.model_name, pose, Twist(), "world"))

if __name__ == '__main__':

    # Start node
    rospy.init_node('merry_go_round')
    MerryGoRound = MerryGoRound()

    # Update rate
    update_rate = rospy.get_param('~update_rate', 5.0)

    # Spin
    r = rospy.Rate(update_rate)
    try:
        while not rospy.is_shutdown():
            MerryGoRound.maneuver()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
