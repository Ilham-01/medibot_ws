
from geometry_msgs.msg import Twist


class MedibotPath():

    def drive_circle(radius, velocity):
        twist = Twist()
        linear_velocity = velocity  # unit: m/s
        angular_velocity = linear_velocity / radius  # unit: rad/s

        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity

        return twist
