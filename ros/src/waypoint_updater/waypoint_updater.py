#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
LOOP_RATE = 2 # Rate that waypoints are updated

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', ?, self.obstacle_cb) - Future

        # Publishers
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Member variables from subscribers
        self.current_pose = None
        self.base_waypoints = None
        self.traffic_waypoint = None
        self.obstacle_waypoint = None # Future
        self.frame_id = None
        
        # Other member variables
        rate = rospy.Rate(LOOP_RATE)
        start_time = 0

        while not start_time:
            start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            if self.current_pose is None or self.base_waypoints is None or self.frame_id is None:
                continue
            self.final_waypoints_pub.publish(get_waypoints())
            rate.sleep()

    def pose_cb(self, msg):
        # Update members
        self.current_pose = msg.pose
        self.frame_id = msg.header.frame_id

    def waypoints_cb(self, waypoints):
        # Update member
        self.base_waypoints = waypoints

    def traffic_cb(self, msg):
        # Update member
        self.traffic_waypoint = msg.data

    def obstacle_cb(self, msg): # Future
        # Update member
        self.obstacle_waypoint = msg.data

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist
        
    def get_waypoints(self):
        # Create lane
        lane = Lane()
        
        # TODO - Define waypoints
        
        return lane


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
