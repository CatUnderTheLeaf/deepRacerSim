#!/usr/bin/env python3
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import threading
import time
import cv2
import os
import csv
      
class CollectData():

    def __init__(self):
        self.update_rate = rospy.get_param("~update_rate", 30) # Hz

        self._lock = threading.Lock()
        # Car speed (m/s)
        self.speed = 0
        # Steering angle (rad)
        self.steering_angle = 0
        self.cv_image = None
        self.data = []
        self.image_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'images')
        self.csv_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'data', 'data.csv')

        self.cmd_sub = rospy.Subscriber("/ackermann_cmd", AckermannDriveStamped, self.ackermann_cmd_cb, queue_size=1)
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/zed/rgb/image_rect_color", Image, self.image_cb, queue_size=1)

        self.control()

    def control(self):
        rate = rospy.Rate(self.update_rate) 
        rospy.on_shutdown(self.save_data)
        rospy.loginfo("started collecting data")                
        while not rospy.is_shutdown():
            self.collect_data()
            rate.sleep()

    def ackermann_cmd_cb(self, msg):
        # rospy.loginfo("received ackermann_cmd_cb")
        with self._lock:
            self.speed = msg.drive.speed
            self.steering_angle = msg.drive.steering_angle

    def image_cb(self, msg):
        # rospy.loginfo("received image_cb")
        try:
            with self._lock:
                self.cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except CvBridgeError as e:
            print(e)
        
    def collect_data(self):
        if self.cv_image is not None:
            with self._lock:
                if self.speed>0.0:
                    name = os.path.join(self.image_path, str(time.time())+'.jpg')
                    cv2.imwrite(name, self.cv_image)
                    self.data.append([name, self.speed, self.steering_angle])
    
    def save_data(self):
        rospy.loginfo("saving data")
        with open(self.csv_path, 'w') as f:
            writer = csv.writer(f)
            writer.writerows(self.data)


if __name__ == '__main__':
    try:
        rospy.init_node('collect_training_data', anonymous=True, log_level=rospy.INFO)
        node = CollectData()
    except KeyboardInterrupt:
        print("Shutting down ROS collect_training_data node")
