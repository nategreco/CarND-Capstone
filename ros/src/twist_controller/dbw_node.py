#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import twist_controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

# Credit Statement: this code is based on 'DBW walkthrough' presentation

class vehicle_properties(object):
    '''
    vehicle_properties class provides a an easy way to pass
    properties as a single parameter. Only provides convenience
    '''

    def __init__(self):
        # all vehicle properties in one place
        self.vehicle_mass = None
        self.fuel_capacity = None
        self.brake_deadband = None
        self.decel_limit = None
        self.accel_limit = None
        self.wheel_radius = None
        self.wheel_base = None
        self.steer_ratio = None
        self.max_lat_accel = None
        self.max_steer_angle = None
        self.min_speed = None

class DBWNode(object):
    '''
    DBWNode is a ROS node that is responsible for taking twist_cmd data
    published from the Waypoint Follower ROS node, and information from the
    car/simulator from ROS topics: current_velocity and dbw_enabled. This ROS
    node publishes topics for brake control, steering, and throttle. If
    dbw_enabled, the car/simulator will take the control data and operate the
    vehicle
    '''
    def __init__(self):
        rospy.init_node('dbw_node')

        self.vp = vehicle_properties()

        # ask ros for parameters
        self.vp.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        self.vp.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        self.vp.brake_deadband = rospy.get_param('~brake_deadband', .1)
        self.vp.decel_limit = rospy.get_param('~decel_limit', -5)
        self.vp.accel_limit = rospy.get_param('~accel_limit', 1.)
        self.vp.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        self.vp.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        self.vp.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        self.vp.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        self.vp.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # class variables
        self.dbw_enabled = True        # drive by wire enabledb
        self.current_velocity = None           # current velocity
        self.twist_cmd = None          # current twist command

        # Create `Controller` object
        self.controller = twist_controller.Controller(vehicle_properties=self.vp)

        # Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size=5)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=5)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1)

        # first run flag
        self.first_run_flag = 250  # @50/second time to hold break at starup - teakable

        self.loop()


    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data


    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x


    def twist_cmd_cb(self, msg):
        self.twist_cmd = msg


    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # Get predicted throttle, brake, and steering using `twist_controller`

            if self.first_run_flag > 0:
                # initially no steering, no throattle and full break
                self.publish(0., 400., 0.)
                self.first_run_flag -= 1
                # rospy.logwarn("dbw_node: pre-brakes applied")
                rate.sleep()
                continue

            do_process = (self.current_velocity is not None and
                          self.twist_cmd is not None)
            if do_process:
                # get control data from a twist_controller
                throttle, brake, steering = self.controller.control(
                    self.twist_cmd,
                    self.current_velocity,
                    self.dbw_enabled
                    )

                if self.dbw_enabled:
                    self.publish(throttle, brake, steering)

            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
