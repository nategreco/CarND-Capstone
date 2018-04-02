#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import tf

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

LOOKAHEAD_WPS = 100         # waypoints
LOOP_RATE = 1               # hz
MAX_SPD = 20.0 * 0.44704    # m/s
ACCEL = 1.0                 # m/s^2
DECEL = 1.0                 # m/s^2
STOP_AHEAD = 5.0            # m

dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        # rospy.Subscriber('/obstacle_waypoint', ?, self.obstacle_cb) - Future

        # Publishers
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Member variables from subscribers
        self.current_pose = None
        self.base_waypoints = None
        self.traffic_waypoint = None
        self.obstacle_waypoint = None # Future
        self.seq = 0
        self.frame_id = None
        
        # Other member variables
        self.current_velocity = None
        self.waypoints = None
        self.nearest_waypoint_idx = -1
        rate = rospy.Rate(LOOP_RATE)
        start_time = 0

        while not start_time:
            start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            if self.current_pose is None or self.base_waypoints is None or self.frame_id is None:
                continue
            self.final_waypoints_pub.publish(self.get_publish_data())
            rate.sleep()

    def pose_cb(self, msg):
        # Update members
        self.current_pose = msg.pose
        self.frame_id = msg.header.frame_id

    def waypoints_cb(self, waypoints):
        # Update member
        self.base_waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # Update member
        self.traffic_waypoint = msg.data

    def velocity_cb(self, msg):
        # Update member
        self.current_velocity = msg.twist

    def obstacle_cb(self, msg): # Future
        # Update member
        self.obstacle_waypoint = msg.data

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def check_waypoint_behind(self, pose, waypoint):
        # Transform waypoint to car coordinates
        tf.transformations.euler_from_quaternion([pose.orientation.x,
                                                  pose.orientation.y,
                                                  pose.orientation.z,
                                                  pose.orientation.w])
        
        # Now check if waypoint is ahead or behind
        return ((waypoint.pose.pose.position.x - pose.position.x) < 0)
    
    def get_nearest_waypoint(self, pose, waypoints):
        # Find index of nearest waypoint ahead of vehicle
        best_dist = float('inf')
        best_idx = 0
        
        # If current waypoint index not defined, search whole list
        if self.nearest_waypoint_idx < 0:
            # Loop through all basepoints and find shortest distance to pose
            for i in range(0, len(waypoints)):
                dist = dl(waypoints[i].pose.pose.position, pose.position)
                if dist < best_dist:
                    best_idx = i
                    best_dist = dist
        else:
            # Start at previous nearest waypoint index - saves processing time
            for i in range(self.nearest_waypoint_idx, len(waypoints)):
                dist = dl(waypoints[i].pose.pose.position, pose.position)
                if dist < best_dist:
                    best_idx = i
                    best_dist = dist
                else:
                    break
            
        # Now check if waypoint is behind
        if self.check_waypoint_behind(pose, waypoints[best_idx]):
            best_idx += 1
                       
        # Return index
        self.nearest_waypoint_idx = best_idx
        return best_idx
    
    def get_new_vel(self, trgt, vel, dist):
        new_vel = vel
        if (vel < trgt):
            new_vel = math.sqrt(vel*vel + 2 * ACCEL * dist) # Kinematic eq - Vf^2 = Vi^2 + 2*a*d
            if (vel > trgt): # Don't overshoot
                new_vel = trgt
        elif (vel > trgt):
            new_vel = math.sqrt(max(0,vel*vel - 2 * DECEL * dist)) # Kinematic eq - Vf^2 = Vi^2 + 2*a*d
            if (vel < trgt): # Don't overshoot
                vel = trgt
        return new_vel
        
        
    def get_waypoints(self):
        # Get waypoints in lookahead distance
        idx1 = self.get_nearest_waypoint(self.current_pose, self.base_waypoints)
        idx2 = idx1 + LOOKAHEAD_WPS - 1
        indices = None
        # Check for wrap around
        if idx2 > len(self.base_waypoints):
            indices = range(idx1, len(self.base_waypoints))
            indices += range(0, idx2 - len(self.base_waypoints))
        else:
            indices = range(idx1, idx2)
        wps = [self.base_waypoints[i] for i in indices]
        
        # Constant accel/decel to max speed, ignore jerk
        vel = self.get_new_vel(MAX_SPD, self.current_velocity.linear.x, dl(wps[0].pose.pose.position, self.current_pose.position))
        self.set_waypoint_velocity(wps, 0, vel)
        for i in range(0, len(wps) - 1):
            vel = self.get_new_vel(MAX_SPD, self.current_velocity.linear.x, self.distance(wps, i, i + 1))
            self.set_waypoint_velocity(wps, i + 1, vel)
        
        # Now check if we should stop for a traffic light
        if not (self.traffic_waypoint is None or self.traffic_waypoint == -1 or self.traffic_waypoint > self.waypoints[-1]):
            # Re-profile velocity to stop at traffic light
            vel = self.get_waypoint_velocity(wps[0])
            for i in range(0, len(wps) - 1):
                light_dist = dl(wps[0].pose.pose.position, self.base_waypoints[self.traffic_waypoint].pose.pose.position) - STOP_AHEAD
                stop_dist = (vel * vel) / (2 * DECEL) # Kinematic eq - Vf^2 = Vi^2 + 2*a*d
                if (light_dist > stop_dist):
                    continue
                vel = self.get_new_vel(0, vel, self.distance(wps, i, i + 1))
                self.set_waypoint_velocity(wps, i + 1, vel)
        
        # Return waypoints
        return wps
        
    def get_publish_data(self):
        # Create lane
        lane = Lane()
        
        # Set header
        lane.header.seq = self.seq
        self.seq += 1 # Increment
        lane.header.stamp = rospy.Time.now()
        lane.header.frame_id = self.frame_id
        
        # Update waypoints
        self.waypoints = self.get_waypoints()
        
        # Add waypoints to lane
        lane.waypoints = self.waypoints
        
        # Return lane
        return lane


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
