#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import speech_recognition as sr
from subprocess import call


r = sr.Recognizer()


def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    with sr.Microphone() as source:
        call(["espeak","-s140 -ven+18 -z","Hello Sir,     How are You ?"])
        print("Hello Sir,     How are You ?")
        r.adjust_for_ambient_noise(source)
        audio = r.listen(source)
    #print("Sphinx thinks you said " + r.recognize_sphinx(audio))
    data = r.recognize_google(audio)
    data = data.lower()
    print("Sphinx thinks you said " + data)
    place = ""
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    if 'kitchen' in data:
        call(["espeak","-s140 -ven+18 -z","Going to kitchen"])
        goal.target_pose.pose.position.x = 6.14083003998
        goal.target_pose.pose.position.y = -2.16255235672
        goal.target_pose.pose.orientation.w =  0.999742097682
        client.send_goal(goal) 
        place="kitchen"       

    elif 'hall' in data:
        call(["espeak","-s140 -ven+18 -z","Going to hall"])
        goal.target_pose.pose.position.x = 0
        goal.target_pose.pose.position.y = 0
        goal.target_pose.pose.orientation.w =  0
        client.send_goal(goal)
        place="hall"

    elif 'living' in data:
        call(["espeak","-s140 -ven+18 -z","Going to living room"])
        goal.target_pose.pose.position.x = -4.83513975143
        goal.target_pose.pose.position.y = 0.163211867213
        goal.target_pose.pose.orientation.w =  0.685605059046
        client.send_goal(goal)
        place="living room"

    elif 'bed room' in data:
        call(["espeak","-s140 -ven+18 -z","Going to bed room"])
        goal.target_pose.pose.position.x = -5.12075853348
        goal.target_pose.pose.position.y = -5.67159795761
        goal.target_pose.pose.orientation.w = 0.743818342819
        client.send_goal(goal)
        place="bed room"
    
    elif 'hii' in data:
        call(["espeak","-s140 -ven+18 -z","hello Sir How Are You ?"])
    
    elif 'name' in data:
        call(["espeak","-s140 -ven+18 -z","My name is ANCRo"])

    else:
        print("No Goal")

    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result(),place

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        while not rospy.is_shutdown():
            result,pl = movebase_client()
            if result:
                rospy.loginfo("Goal execution done!")
                text="Reached to" + pl
                call(["espeak","-s140 -ven+18 -z",text])







    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
