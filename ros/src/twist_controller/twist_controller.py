
import rospy
from geometry_msgs.msg import TwistStamped
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
from dbw_node import vehicle_properties

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_properties):
        # store vehicle properties
        self.vp = vehicle_properties

        # create yaw controller
        self.yaw = YawController(
            wheel_base = self.vp.wheel_base,
            steer_ratio = self.vp.steer_ratio,
            min_speed = 0.1, # min speed
            max_lat_accel = self.vp.max_lat_accel,
            max_steer_angle = self.vp.max_steer_angle
        )

        # create pid Controller (TODO: kp, ki, kd are guesses or from walkthrough)
        self.pid = PID(
            kp=0.3,                  # kp is proportional term
            ki=0.1,                  # ki is integral term
            kd=0.,                 # kd is derivative term
            mn = self.vp.decel_limit,     # min
            mx = self.vp.accel_limit      # max
        )

        # create filters (TODO: tau and ts are guesses - from walkthrough)
        self.steering_filter = LowPassFilter(tau = 0.5, ts = .02) # TODO: needed?
        self.velocity_filter = LowPassFilter(tau = 0.5, ts = .02)
        self.prior_timestamp = rospy.get_time() # initial time stamp

        self.torque = ((self.vp.vehicle_mass + self.vp.fuel_capacity * GAS_DENSITY) *
                       self.vp.wheel_radius)
        # Subscribers
        # TODO: any Subscribers needed?

        # Publishers
        #self.current_velocity_pub = rospy.Publisher('current_velocity', TwistStamped, queue_size=1)

        # TODO: Implement
        pass

    def control(self, twist_cmd, current_vel, dbw_enabled):

        # rospy.logwarn("dbw_enabled: {0}".format(dbw_enabled))

        # dbw not enabled
        if not dbw_enabled:
            self.pid.reset()
            return 0., 0., 0,

        # smoothout income velocity values
        current_vel = self.velocity_filter.filt(current_vel)

        linear_vel = twist_cmd.twist.linear.x
        angular_vel = twist_cmd.twist.angular.z

        # get steering value from yaw_controller
        steering_control = self.yaw.get_steering(
            linear_vel,
            angular_vel,
            current_vel
            )

        # filter the steering jitter TODO: need this?
        # attempt to damp steering
        steering_control = self.steering_filter.filt(steering_control)

        # get acceration from PID
        current_time = rospy.get_time()  # find time gap beween calls
        delta_time = current_time - self.prior_timestamp
        self.prior_timestamp = current_time

        vel_error = linear_vel - current_vel
        throttle_control = self.pid.step(vel_error, delta_time)

        brake_control = 0.0

        if linear_vel == 0. and current_vel < .01:
            throttle_control = 0
            brake_control = 400 #N*m - to hold car if stopped

        elif throttle_control < 0.1 and vel_error < 0:
            throttle_control = 0
            decel = max(vel_error, self.vp.decel_limit)
            brake_control = abs(decel) * self.torque

        # Return throttle, brake, steer
        return throttle_control, brake_control, steering_control
