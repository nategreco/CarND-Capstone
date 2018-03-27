
from geometry_msgs.msg import TwistStamped

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):

        # Subscribers

        # Publishers
        self.current_velocity_pub = rospy.Publisher('current_velocity', TwistStamped, queue_size=1)
		
        # TODO: Implement
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
