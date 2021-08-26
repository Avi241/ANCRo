#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
rospy.init_node('opencv_pub', anonymous=True)
pub = rospy.Publisher('image_raw',Image,queue_size=10)
# Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
rospy.loginfo("Hello ROS!")
# Initialize the CvBridge class
rate = rospy.Rate(20)
bridge = CvBridge()
cam = cv2.VideoCapture(1)

while not rospy.is_shutdown():
    _,img = cam.read()
    # img = cv2.resize(img,(640,240))
    image_message = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    pub.publish(image_message)
    # cv2.imshow("hii",img)
    rate.sleep()

cam.release()



    