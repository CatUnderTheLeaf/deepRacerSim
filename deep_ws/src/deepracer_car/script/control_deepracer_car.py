#!/usr/bin/env python3
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

from collections import OrderedDict
import math



# List of required velocity topics, one topic per wheel
VELOCITY_TOPICS = ['/left_rear_wheel_velocity_controller/command',
                   '/right_rear_wheel_velocity_controller/command',
                   '/left_front_wheel_velocity_controller/command',
                   '/right_front_wheel_velocity_controller/command']

# List of required steering hinges
STEERING_TOPICS = ['/left_steering_hinge_position_controller/command',
                   '/right_steering_hinge_position_controller/command']

# SPEED is linear velocity in m/s.
MAX_SPEED = 4
MIN_SPEED = 0.1
# ANGLE is the steering angle value in degree.
MAX_ANGLE = 30.0
MIN_ANGLE = -30.0

class CarController():

    def __init__(self):
        # param_name = rospy.search_param('global_example')
        # v = rospy.get_param(param_name)
        # rospy.logdebug(v)
        self._wheel_radius_ = 0.035
        self._steer_ang = 0.0
        self._steer_ang_vel = 0.0
        self._speed = 0.0
        self._accel = 0.0
        # Create publishers for controlling the car
        self._velocity_pub_dict_ = OrderedDict()
        self._steering_pub_dict_ = OrderedDict()

        for topic in VELOCITY_TOPICS:
            self._velocity_pub_dict_[topic] = rospy.Publisher(topic, Float64, queue_size=1)
        for topic in STEERING_TOPICS:
            self._steering_pub_dict_[topic] = rospy.Publisher(topic, Float64, queue_size=1)
        
        self._ackermann_cmd_sub = rospy.Subscriber("ackermann_cmd", AckermannDriveStamped,
                             self.ackermann_cmd_cb, queue_size=1)

        # # init car with zero values
        # self.publish_commands(0, 0)

        self.control()

    def control(self):
        rate = rospy.Rate(50) # 50Hz

        last_time = rospy.get_time()

        while not rospy.is_shutdown():
            t = rospy.get_time()
            delta_t = t - last_time
            last_time = t

            # # Clip the angle to be between min and max angle allowed
            # steering_angle = max(min(MAX_ANGLE, self._steer_ang), MIN_ANGLE)
            # steering_angle = steering_angle * math.pi / 180.0
            
            # action_speed = max(min(MAX_SPEED, self._speed), MIN_SPEED)
            # action_speed = float(action_speed / self._wheel_radius_)
            # self.publish_commands(steering_angle, action_speed)           
            
            rate.sleep()


    def ackermann_cmd_cb(self, ackermann_cmd):
        """Ackermann driving command callback

        :Parameters:
          ackermann_cmd : ackermann_msgs.msg.AckermannDriveStamped
            Ackermann driving command.
        """
        self._last_cmd_time = rospy.get_time()
        
        self._steer_ang = ackermann_cmd.drive.steering_angle
        self._steer_ang_vel = ackermann_cmd.drive.steering_angle_velocity
        self._speed = ackermann_cmd.drive.speed #/0.1
        self._accel = ackermann_cmd.drive.acceleration        

         # Clip the angle to be between min and max angle allowed
        steering_angle = max(min(MAX_ANGLE, self._steer_ang), MIN_ANGLE)
        steering_angle = steering_angle * math.pi / 180.0
        
        action_speed = max(min(MAX_SPEED, self._speed), MIN_SPEED)
        action_speed = float(action_speed / self._wheel_radius_)
        self.publish_commands(steering_angle, action_speed)        


    def publish_commands(self, steering_angle, speed):
        '''Publishes the given action to all the topics in the given dicts
        velocity_pub_dict - Dictionary containing all the velocity joints
        steering_pub_dict - Dictionary containing all the movable joints
        steering_angle - Desired amount, in radians, to move the movable joints by
        speed - Angular velocity which the velocity joints should rotate with
        '''
        for _, pub in self._velocity_pub_dict_.items():
            pub.publish(speed)

        for _, pub in self._steering_pub_dict_.items():
            pub.publish(steering_angle)
        


if __name__ == '__main__':
    try:
        rospy.init_node('control_deepracer_car', anonymous=True, log_level=rospy.INFO)
        node = CarController()
    except KeyboardInterrupt:
        print("Shutting down ROS control_deepracer_car node")