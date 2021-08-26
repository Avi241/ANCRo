#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from subprocess import call
"""
rospy.init_node("topic_publisher")
pub = rospy.Publisher('faces',String,queue_size=10)

rate = rospy.Rate(10)
msg_str = String()
msg_str = ""
"""
import cv2
import numpy
import os
from cv2 import face

dataset = "datasets"
(images,labels,names,id) = ([],[],{},0)


for (subdirs,dirs,files) in os.walk(dataset):
    for subdir in dirs:
        names[id] = subdir
        subjectpath = os.path.join(dataset,subdir)
        for filename in os.listdir(subjectpath):
            path = subjectpath + "/" +filename
            label = id
            images.append(cv2.imread(path,0))
            labels.append(int(label))
        id += 1
(width,height) = (130,100)
(images,labels) = [numpy.array(lis) for lis in [images,labels]]
model = face.createFisherFaceRecognizer()
model.train(images,labels)
print("Trainig done")

file = "haarcascade_frontalface_default.xml"
haar = cv2.CascadeClassifier(file)
cam = cv2.VideoCapture(0)

(width,height) = (130,100)
count = 0

while True:
    _,img = cam.read()

    # imgResp = urllib.request.urlopen(url)
	# imgNp = np.array(bytearray(imgResp.read()),dtype=np.uint8)
	# frame = cv2.imdecode(imgNp,-1)
    # frame = img

    print(count)
    grayImg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    faces = haar.detectMultiScale(grayImg,1.3,5)
    for (x,y,w,h) in faces:
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
        onlyface = grayImg[y:y+h,x:x+w]
        resizeImg = cv2.resize(onlyface,(width,height))
        prediction = model.predict(resizeImg)
        if  prediction[1]<600:
            cv2.putText(img,'%s - %f' % (names[prediction[0]],prediction[1]),(x-10,y-10),cv2.FONT_HERSHEY_PLAIN,1,(0,255,0),2)
            #pub.publish(names[prediction[0]])
            #call(["espeak","-s140 -ven+18 -z","Hello"+names[prediction[0]]])
            count = 0
        else:
            count +=1
            cv2.putText(img,'Unknown',(x-10,y-10),cv2.FONT_HERSHEY_PLAIN,1,(0,255,0),2)
            if(count>100):
                print("Unknown Person")
                # cv2.imwrite("input.jpg",img)
                count = 0
    cv2.imshow("Face recognition",img)
    #rate.sleep()
    key = cv2.waitKey(10)
    if key == 27:
        break
cam.release()
cv2.destroyAllWindows()
