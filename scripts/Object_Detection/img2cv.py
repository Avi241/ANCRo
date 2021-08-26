#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

print("Hello!")

rospy.init_node('opencv_example', anonymous=True)
rospy.loginfo("Hello ROS!")
bridge = CvBridge()

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

def image_callback(img_msg):
    rospy.loginfo(img_msg.header)
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    cv2.imshow("Image Window", cv_image)
    cv2.waitKey(3)

sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

cv2.namedWindow("Image Window", 1)

while not rospy.is_shutdown():
    rospy.spin()