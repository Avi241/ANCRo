#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import jetson.inference
import jetson.utils
import time
import cv2
import numpy as np 
image=[]

timeStamp=time.time()
fpsFilt=0
net=jetson.inference.detectNet('ssd-mobilenet-v2',threshold=.5)
dispW=1280
dispH=720
flip=2
font=cv2.FONT_HERSHEY_SIMPLEX

# Gstreamer codee for improvded Raspberry Pi Camera Quality
#camSet='nvarguscamerasrc wbmode=3 tnr-mode=2 tnr-strength=1 ee-mode=2 ee-strength=1 ! video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! videobalance contrast=1.5 brightness=-.2 saturation=1.2 ! appsink'
#cam=cv2.VideoCapture(camSet)
#cam=jetson.utils.gstCamera(dispW,dispH,'0')

# cam=cv2.VideoCapture('/dev/video0')
# cam.set(cv2.CAP_PROP_FRAME_WIDTH, dispW)
# cam.set(cv2.CAP_PROP_FRAME_HEIGHT, dispH)

rospy.init_node('opencv_example', anonymous=True)
rospy.loginfo("Hello ROS!")
bridge = CvBridge()

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

def image_callback(img_msg):
    rospy.loginfo(img_msg.header)
    img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    height=img.shape[0]
    width=img.shape[1]

    frame=cv2.cvtColor(img,cv2.COLOR_BGR2RGBA).astype(np.float32)
    frame=jetson.utils.cudaFromNumpy(frame)

    detections=net.Detect(frame, width, height)
    for detect in detections:
        #print(detect)
        ID=detect.ClassID
        top=detect.Top
        left=detect.Left
        bottom=detect.Bottom
        right=detect.Right
        item=net.GetClassDesc(ID)
        print(item,top,left,bottom,right)
    

sub_image = rospy.Subscriber("/image_raw", Image, image_callback)

while not rospy.is_shutdown():
    rospy.spin()

