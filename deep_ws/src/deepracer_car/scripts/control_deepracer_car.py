#!/usr/bin/env python3
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64MultiArray, Float64


from collections import OrderedDict
import math
import threading
      
class CarController():

    def __init__(self):
        self.max_speed = rospy.get_param("~max_speed", 4.0)
        self.max_steering_angle = rospy.get_param("~max_steering_angle", 0.523599)
        self._wheel_radius = rospy.get_param("~wheel_radius", 0.03)
        self._wheel_separation = rospy.get_param("~wheel_separation", 0.159202)
        self._wheel_base = rospy.get_param("~wheel_base", 0.164023)
        self.update_rate = rospy.get_param("~update_rate", 50) # Hz

        self._cmd_lock = threading.Lock()
        # Car speed (m/s)
        self.speed = 0
        # Steering angle (rad)
        self.steering_angle = 0

        # Create publishers for controlling the car
        self._velocity_pub_dict_ = OrderedDict()
        self._steering_pub_dict_ = OrderedDict()

        # self._velocity_pub_dict_["l_rear_wheel"] = rospy.Publisher('/left_rear_wheel_velocity_controller/command', Float64MultiArray, queue_size=1)
        # self._velocity_pub_dict_["r_rear_wheel"] = rospy.Publisher('/right_rear_wheel_velocity_controller/command', Float64MultiArray, queue_size=1)
        # self._velocity_pub_dict_["l_front_wheel"] = rospy.Publisher('/left_front_wheel_velocity_controller/command', Float64MultiArray, queue_size=1)
        # self._velocity_pub_dict_["r_front_wheel"] = rospy.Publisher('/right_front_wheel_velocity_controller/command', Float64MultiArray, queue_size=1)
        
        # self._steering_pub_dict_['left'] = rospy.Publisher('/left_steering_hinge_position_controller/command', Float64MultiArray, queue_size=1)
        # self._steering_pub_dict_['right'] = rospy.Publisher('/right_steering_hinge_position_controller/command', Float64MultiArray, queue_size=1)
        
        self._velocity_pub_dict_["l_rear_wheel"] = rospy.Publisher('/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self._velocity_pub_dict_["r_rear_wheel"] = rospy.Publisher('/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self._velocity_pub_dict_["l_front_wheel"] = rospy.Publisher('/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
        self._velocity_pub_dict_["r_front_wheel"] = rospy.Publisher('/right_front_wheel_velocity_controller/command', Float64, queue_size=1)
        
        self._steering_pub_dict_['left'] = rospy.Publisher('/left_steering_hinge_position_controller/command', Float64, queue_size=1)
        self._steering_pub_dict_['right'] = rospy.Publisher('/right_steering_hinge_position_controller/command', Float64, queue_size=1)
        

        self.cmd_sub = rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped,
                             self.ackermann_cmd_cb, queue_size=1)

        self.control()

    def control(self):
        rate = rospy.Rate(self.update_rate) 
        update_period = 1/self.update_rate
        
        last_time = rospy.get_time()

        while not rospy.is_shutdown():
            t = rospy.get_time()
            delta_t = t - last_time
            
            if delta_t>=update_period:
                # TODO
                # publish odometry and TF

                #  Calculate target velocity and position values and publish the commands
                with self._cmd_lock:
                    t_speed, t_left_steering, t_right_steering = self.calc_target_speed_steering(delta_t)
                self.publish_commands(t_speed, t_left_steering, t_right_steering)
                last_time = t

            rate.sleep()

    def calc_target_speed_steering(self, delta_t):
        
        target_speed = max(min(self.max_speed, self.speed), -self.max_speed)
        target_steer_angle = self.steering_angle * math.copysign(1.0, self.speed)
        target_steer_angle = max(min(self.max_steering_angle, target_steer_angle), -self.max_steering_angle)

        tanSteer = math.tan(target_steer_angle)

        t_left_steering = math.atan2(tanSteer, 1.0 - self._wheel_separation / 2.0 / self._wheel_base * tanSteer)
        t_right_steering = math.atan2(tanSteer, 1.0 + self._wheel_separation / 2.0 / self._wheel_base * tanSteer)

        t_speed = target_speed / self._wheel_radius

# TODO
# don't go backwards for speed < 0
# when speed==0 center wheels, else a car will spin

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
        # speed_msg = Float64MultiArray()
        # speed_msg.data.append(t_speed)

        # left_steering_msg = Float64MultiArray()
        # left_steering_msg.data.append(t_left_steering)

        # right_steering_msg = Float64MultiArray()
        # right_steering_msg.data.append(t_right_steering)

        # for _, pub in self._velocity_pub_dict_.items():
        #     pub.publish(speed_msg)

        # self._steering_pub_dict_['left'].publish(left_steering_msg)
        # self._steering_pub_dict_['right'].publish(right_steering_msg)

        for _, pub in self._velocity_pub_dict_.items():
            pub.publish(t_speed)

        self._steering_pub_dict_['left'].publish(t_left_steering)
        self._steering_pub_dict_['right'].publish(t_right_steering)
        


if __name__ == '__main__':
    try:
        rospy.init_node('control_deepracer_car', anonymous=True, log_level=rospy.INFO)
        node = CarController()
    except KeyboardInterrupt:
        print("Shutting down ROS control_deepracer_car node")
