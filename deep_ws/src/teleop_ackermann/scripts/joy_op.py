#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy

import sys, termios

class JoyTeleopAckermann():

    def __init__(self):

        self.max_speed = rospy.get_param("~max_speed", 4.0)
        self.max_steering_angle = rospy.get_param("~max_steering_angle", 0.523599)
        self.steering_axis = rospy.get_param("axis_steering", 0)
        self.speed_axis = rospy.get_param("axis_speed", 4)
        self.stop_button = rospy.get_param("stop_button", 4)
        self.align_button = rospy.get_param("align_button", 6)

        self.update_rate = rospy.get_param("~update_rate", 50) # Hz
        self.speed = 0
        self.steering_angle = 0

        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)

        self.ack_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=1)

        # publish ackermann_msg with a rate
        rospy.Timer(rospy.Duration(1.0/self.update_rate), self.publish_message)

        # control joy input
        self.print_info()

    def joy_callback(self, data):
        """Callback on each change in joy message

        Args:
            data (sensor_msgs.Joy): joy message

        """    
        if (data.buttons[self.stop_button]==1):
            self.stop()
        elif(data.buttons[self.align_button]==1):
            self.steering_angle = 0.0
        else:
            self.speed = max(round(data.axes[self.speed_axis], 2) * self.max_speed, 0.0)
            self.steering_angle = round(data.axes[self.steering_axis], 2) * self.max_steering_angle 
            if self.speed==0:
                self.steering_angle=0

    def publish_message(self, event):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/base_link'
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steering_angle
        self.ack_pub.publish(msg)
        rospy.loginfo('\x1b[1M\r'
                '\033[34;1mSpeed: \033[32;1m%0.2f m/s, '
                '\033[34;1mSteering Angle: \033[32;1m%0.2f rad\033[0m',
                self.speed, self.steering_angle)

    def print_info(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r*********************************************')
        rospy.loginfo('\x1b[1M\rUse up/down stick to change speed')
        rospy.loginfo('\x1b[1M\rUse left/right stick to change steering angle')
        rospy.loginfo('\x1b[1M\rUse L1 button to brake and L2 button to align wheels')
        rospy.loginfo('\x1b[1M\r*********************************************')
        
    def stop(self):
        # publish last zero commands
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 0
        self.steering_angle = 0
        self.publish_message(self.speed)
        sys.exit()

if __name__ == '__main__':
    try:
        rospy.init_node('joy_teleop_ackermann_drive', anonymous=True, log_level=rospy.INFO)
        node = JoyTeleopAckermann()
    except KeyboardInterrupt:
        print("Shutting down ROS joy_teleop_ackermann_drive node")