#! /usr/bin/env python3
import rospy
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from subprocess import call
rospy.init_node("GoalPublisher")


def callback(data):
    result,pl = movebase_client(str(data.data))
    if result:
        rospy.loginfo("Goal Execution Done")
        text = "Reached to " + pl
        call(["espeak","-s140 -ven+18 -z",text])

def movebase_client(data):
    print(data)
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    data = data.lower()
    print("ANCRo thinks you said " + data)
    place = ""
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    if 'kitchen' in data:
        print("Going to kitchen")
        call(["espeak","-s140 -ven+18 -z","Going to kitchen"])
        goal.target_pose.pose.position.x = 1.12364828587
        goal.target_pose.pose.position.y = -5.29889297485
        goal.target_pose.pose.orientation.w =  -0.151284012867
        client.send_goal(goal) 
        place="kitchen"       

    elif 'haul' in data:
        call(["espeak","-s140 -ven+18 -z","Going to hall"])
        goal.target_pose.pose.position.x = 0
        goal.target_pose.pose.position.y = 0
        goal.target_pose.pose.orientation.w =  0
        client.send_goal(goal)
        place="hall"

    elif 'bedroom' in data:
        print("Going to bedroom")
        call(["espeak","-s140 -ven+18 -z","Going to bedroom"])
        goal.target_pose.pose.position.x = 0.582225322723
        goal.target_pose.pose.position.y = 4.40308570862
        goal.target_pose.pose.orientation.w =  0.668215870857
        client.send_goal(goal)
        place="living room"

    elif 'living' in data:
        print("Going to living room")
        call(["espeak","-s140 -ven+18 -z","Going to living room"])
        goal.target_pose.pose.position.x = -0.113405890763
        goal.target_pose.pose.position.y = 6.83526706696
        goal.target_pose.pose.orientation.w = 0.142804607749
        client.send_goal(goal)
        place="bed room"

    elif 'hi' in data:
        print("Hello Sir")
        call(["espeak","-s140 -ven+18 -z","Hello Sir"])
    
    elif 'name' in data:
        print("I am Ancro")
        call(["espeak","-s140 -ven+18 -z","My Name is Ancro"])

    else:
        print("No Goal")

    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        # rospy.signal_shutdown("Action server not available!")
        return False,""
    else:
        return client.get_result(),place


sub = rospy.Subscriber('/phrases',String,callback)
rospy.spin()
