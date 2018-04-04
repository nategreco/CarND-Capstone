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
STOP_AHEAD = 5.0            # m - Not in use

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
        
        # Loop through all basepoints and find shortest distance to pose
        for i in range(0, len(waypoints)):
            dist = dl(waypoints[i].pose.pose.position, pose.position)
            if dist < best_dist:
                best_idx = i
                best_dist = dist
            
        # Now check if waypoint is behind
        if self.check_waypoint_behind(pose, waypoints[best_idx]):
            best_idx += 1
                       
        # Return index
        return best_idx
    
    def get_new_vel(self, trgt, vel, dist, accel, decel):
        # Check if we need to accel
        if (vel < trgt):
            vel = math.sqrt(vel*vel + 2 * accel * dist) # Kinematic eq - Vf^2 = Vi^2 + 2*a*d
            if (vel > trgt): # Don't overshoot
                vel = trgt
        # Check if we need to decel
        elif (vel > trgt):
            vel = math.sqrt(max(0,vel*vel - 2 * decel * dist)) # Kinematic eq - Vf^2 = Vi^2 + 2*a*d
            if (vel < trgt): # Don't overshoot
                vel = trgt
        
        # Apply universal limits and return
        return max(min(vel,MAX_SPD),0)
        
        
    def get_waypoints(self):
        # Get waypoints in lookahead distance
        idx1 = self.get_nearest_waypoint(self.current_pose, self.base_waypoints)
        # rospy.logwarn("Nearest waypoint: " + str(idx1))
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
        vel = self.current_velocity.linear.x
        if not self.waypoints is None:
            ref_vel_idx = self.get_nearest_waypoint(self.current_pose, self.waypoints)
            if ref_vel_idx:
                vel = self.get_waypoint_velocity(self.waypoints[ref_vel_idx])
            vel = self.get_new_vel(MAX_SPD, vel, dl(self.waypoints[ref_vel_idx].pose.pose.position, self.current_pose.position), ACCEL, DECEL)
        self.set_waypoint_velocity(wps, 0, vel)
        for i in range(0, len(wps) - 1):
            vel = self.get_new_vel(MAX_SPD, vel, self.distance(wps, i, i + 1), ACCEL, DECEL)
            self.set_waypoint_velocity(wps, i + 1, vel)
        
        # Now check if we should stop for a traffic light
        if not ((self.traffic_waypoint is None) or (self.traffic_waypoint == -1)):
            # Re-profile velocity to stop at traffic light
            vel = self.get_waypoint_velocity(wps[0])
            stop_pt = False
            for i in range(0, len(wps) - 1):
                # Check for stop trigger
                light_dist = dl(wps[i].pose.pose.position, self.base_waypoints[self.traffic_waypoint].pose.pose.position)
                stop_dist = (vel * vel) / (2 * DECEL) # Kinematic eq - Vf^2 = Vi^2 + 2*a*d
                if (light_dist <= stop_dist) or (light_dist < 1):
                    stop_pt = True
                # Handle stop trigger
                if not stop_pt:
                    vel = self.get_waypoint_velocity(wps[i])
                else:
                    light_dist = max(0.1, light_dist) # Prevent division by zero or negative decel
                    vel = max(0, vel) # Prevent negative decel
                    new_decel = (vel * vel) / (2 * light_dist) # Calculate decel needed to make the light
                    vel = self.get_new_vel(0, vel, self.distance(wps, i, i + 1), 0, new_decel)
                    self.set_waypoint_velocity(wps, i, vel)
        
        # for i in range(0, 10): # len(wps)):
        #    rospy.logwarn("Waypoint " + str(i) + " speed is: " + str(self.get_waypoint_velocity(wps[i])))

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
