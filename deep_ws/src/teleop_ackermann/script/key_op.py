#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

import sys, select, termios, tty

keys = {
    'up'    : '\x41',
    'down'  : '\x42',
    'right' : '\x43',
    'left'  : '\x44',
    'space' : '\x20',
    'tab'   : '\x09'}

class KeyTeleopAckermann():

    def __init__(self):

        self.max_speed = rospy.get_param("~max_speed", 4.0)
        self.max_steering_angle = rospy.get_param("~max_steering_angle", 0.523599)
        self.scale_speed = rospy.get_param("~scale_speed", 10)
        self.scale_angle = rospy.get_param("~scale_angle", 10)
        self.update_rate = rospy.get_param("~update_rate", 50) # Hz

        self.key_ops = {
            '\x41' : ( self.max_speed/self.scale_speed, 0.0),
            '\x42' : (-self.max_speed/self.scale_speed , 0.0),
            '\x43' : ( 0.0 ,-self.max_steering_angle/self.scale_angle),
            '\x44' : ( 0.0 , self.max_steering_angle/self.scale_angle)}

        self.speed = 0
        self.steering_angle = 0

        self.ack_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=1)

        # publish ackermann_msg with a rate
        rospy.Timer(rospy.Duration(1.0/self.update_rate), self.publishMessage)

        # control key input
        self.printInfo()
        self.control()

    def publishMessage(self, event):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/base_link'
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steering_angle
        self.ack_pub.publish(msg)

    def read_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        return key

    def printInfo(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r*********************************************')
        rospy.loginfo('\x1b[1M\rUse up/down arrows to change speed')
        rospy.loginfo('\x1b[1M\rUse left/right arrows to change steering angle')
        rospy.loginfo('\x1b[1M\rUse space to brake and tab to align wheels')
        rospy.loginfo('\x1b[1M\r*********************************************')
        rospy.loginfo('\x1b[1M\r'
                      '\033[34;1mSpeed: \033[32;1m%0.2f m/s, '
                      '\033[34;1mSteering Angle: \033[32;1m%0.2f rad\033[0m',
                      self.speed, self.steering_angle)

    def control(self):        
        
        while not rospy.is_shutdown():
            key = self.read_key()
            if key in keys.values():
                if key == keys['space']:
                    self.speed = 0.0
                elif key == keys['tab']:
                    self.steering_angle = 0.0
                else:
                    a_speed, a_angle = self.key_ops[key]
                    self.speed = self.speed + a_speed
                    self.steering_angle = self.steering_angle + a_angle
                    # clip in between min and max values
                    self.speed = max(min(self.max_speed, self.speed), -self.max_speed)                        
                    self.steering_angle = max(min(self.max_steering_angle, self.steering_angle), -self.max_steering_angle)
                self.printInfo()            
            elif key == '\x03':  # ctr-c or q
                break
            else:
                continue

        sys.exit()
        # publish last zero commands
        self.speed = 0
        self.steering_angle = 0
        self.publishMessage()



if __name__ == '__main__':
    try:
        rospy.init_node('key_teleop_ackermann_drive', anonymous=True, log_level=rospy.INFO)
        node = KeyTeleopAckermann()
    except KeyboardInterrupt:
        print("Shutting down ROS key_teleop_ackermann_drive node")