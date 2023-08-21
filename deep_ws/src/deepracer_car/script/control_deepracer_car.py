#!/usr/bin/env python3
import rospy
# from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64MultiArray


from collections import OrderedDict
import math
import threading


# List of required velocity topics, one topic per wheel
VELOCITY_TOPICS = ['/left_rear_wheel_velocity_controller/command',
                   '/right_rear_wheel_velocity_controller/command',
                   '/left_front_wheel_velocity_controller/command',
                   '/right_front_wheel_velocity_controller/command']

# List of required steering hinges
STEERING_TOPICS = [('left','/left_steering_hinge_position_controller/command'),
                   ('right','/right_steering_hinge_position_controller/command')]

# SPEED is linear velocity in m/s.
MAX_SPEED = 4
MIN_SPEED = 0.0
# ANGLE is the steering angle value in rad.
MAX_ANGLE = 0.523599
MIN_ANGLE = -0.523599

class CarController():

    def __init__(self):
        # TODO
        # get from params or urdf etc.
        self._wheel_radius = 0.03
        self._wheel_separation = 0.159202
        self._wheel_base = 0.164023

        self._cmd_lock = threading.Lock()
        # Car speed (m/s)
        self.speed = 0
        # Steering angle (rad)
        self.steering_angle = 0

        # Create publishers for controlling the car
        self._velocity_pub_dict_ = OrderedDict()
        self._steering_pub_dict_ = OrderedDict()

        for topic in VELOCITY_TOPICS:
            self._velocity_pub_dict_[topic] = rospy.Publisher(topic, Float64MultiArray, queue_size=1)
        for l_r, topic in STEERING_TOPICS:
            self._steering_pub_dict_[l_r] = rospy.Publisher(topic, Float64MultiArray, queue_size=1)
        
        self.cmd_sub = rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped,
                             self.ackermann_cmd_cb, queue_size=1)

        self.control()

    def control(self):
        update_rate = 50 # Hz
        rate = rospy.Rate(update_rate) 
        update_period = 1/update_rate
        # rospy.loginfo("update_period {}".format(update_period))

        last_time = rospy.get_time()

        while not rospy.is_shutdown():
            t = rospy.get_time()
            delta_t = t - last_time
            
            if delta_t>=update_period:
                # TODO
                # publish odometry and TF

                #  Calculate target velocity and position values and publish the commands
                with self._cmd_lock:
                    t_speed, t_left_steering, t_right_steering = self.calcTargetVelAndPosition(delta_t)
                self.publish_commands(t_speed, t_left_steering, t_right_steering)
                last_time = t

            rate.sleep()

    def calcTargetVelAndPosition(self, delta_t):
        
        target_speed = max(min(MAX_SPEED, self.speed), MIN_SPEED)
        target_steer_angle = self.steering_angle * math.copysign(1.0, self.speed)
        target_steer_angle = max(min(MAX_ANGLE, target_steer_angle), MIN_ANGLE)

        tanSteer = math.tan(target_steer_angle)

        t_left_steering = math.atan2(tanSteer, 1.0 - self._wheel_separation / 2.0 / self._wheel_base * tanSteer)
        t_right_steering = math.atan2(tanSteer, 1.0 + self._wheel_separation / 2.0 / self._wheel_base * tanSteer)

        t_speed = target_speed / self._wheel_radius

        return t_speed, t_left_steering, t_right_steering

    def ackermann_cmd_cb(self, msg):
        # rospy.loginfo("received msg")
        with self._cmd_lock:
            self.speed = msg.drive.speed
            self.steering_angle = msg.drive.steering_angle  
        # rospy.loginfo("from msg speed {}".format(self.speed)) 
        # rospy.loginfo("from msg steering_angle {}".format(self.steering_angle))       


    def publish_commands(self, t_speed, t_left_steering, t_right_steering):
        '''Publishes the given action to all the topics in the given dicts
        velocity_pub_dict - Dictionary containing all the velocity joints
        steering_pub_dict - Dictionary containing all the movable joints
        t_left_steering, t_right_steering - Desired amount, in radians, to move the movable joints by
        t_speed - Angular velocity which the velocity joints should rotate with
        '''
        speed_msg = Float64MultiArray()
        speed_msg.data.append(t_speed)

        left_steering_msg = Float64MultiArray()
        left_steering_msg.data.append(t_left_steering)

        right_steering_msg = Float64MultiArray()
        right_steering_msg.data.append(t_right_steering)

        for _, pub in self._velocity_pub_dict_.items():
            pub.publish(speed_msg)

        self._steering_pub_dict_['left'].publish(left_steering_msg)
        self._steering_pub_dict_['right'].publish(right_steering_msg)
        


if __name__ == '__main__':
    try:
        rospy.init_node('control_deepracer_car', anonymous=True, log_level=rospy.INFO)
        node = CarController()
    except KeyboardInterrupt:
        print("Shutting down ROS control_deepracer_car node")
