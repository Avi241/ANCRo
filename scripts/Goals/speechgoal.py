#! /usr/bin/env python3
import queue
import sounddevice as sd
import vosk
import rospy
from std_msgs.msg import String
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from subprocess import call


a=True
c=True
f=True
now=0
prev=0

rospy.init_node("Speech_Recognition")
pub = rospy.Publisher("phrases", String, queue_size=10)
rate = rospy.Rate(2)
msg_str = String()
msg_str = ""

q = queue.Queue()

def wakecb(hey):
    print(hey)
    global a
    global prev,now
    a=True
    prev=now

def flagcb():
    f=True

def callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))


model = vosk.Model("model")
device_info = sd.query_devices(None, "input")
sub = rospy.Subscriber('/wake',String,wakecb)
fsub = rospy.Subscriber('/flag',String,flagcb)

def movebase_client(data):
    print(data)
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    data = data.lower()
    print("Sphinx thinks you said " + data)
    place = ""
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    if 'kitchen' in data:
        print("Going to kitchen")
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
        print("Going to living room")
        call(["espeak","-s140 -ven+18 -z","Going to living room"])
        goal.target_pose.pose.position.x = -4.83513975143
        goal.target_pose.pose.position.y = 0.163211867213
        goal.target_pose.pose.orientation.w =  0.685605059046
        client.send_goal(goal)
        place="living room"

    elif 'bedroom' in data:
        print("Going to bed room")
        call(["espeak","-s140 -ven+18 -z","Going to bed room"])
        goal.target_pose.pose.position.x = -5.12075853348
        goal.target_pose.pose.position.y = -5.67159795761
        goal.target_pose.pose.orientation.w = 0.743818342819
        client.send_goal(goal)
        place="bed room"

    elif 'hii' in data:
        print("Hello Sir")
        call(["espeak","-s140 -ven+18 -z","Hello Sir"])
    
    elif 'name' in data:
        print("My Name is Ancro")
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


with sd.RawInputStream(
    blocksize=8000,
    dtype="int16",
    channels=1,
    callback=callback,
    samplerate=int(device_info["default_samplerate"]),
):
    print("#" * 80)
    print("Press Ctrl+C to stop the recording")
    print("#" * 80)

    rec = vosk.KaldiRecognizer(model, int(device_info["default_samplerate"]))

    # while not rospy.is_shutdown():
    #     data = q.get()
    #     if rec.AcceptWaveform(data):
    #         a = (eval(rec.Result()))["text"]
    #         print(a)
    #         pub.publish(a)

    while not rospy.is_shutdown():
        # print(a)
        if c==True:
            prev=now
            c=False
        now = int(time.time())
        data = q.get()
        if rec.AcceptWaveform(data):
            b = (eval(rec.Result()))["text"]
            if len(b) >= 2:
                print(b)
                pub.publish(b)
                if "living" in b:
                    result,pl = movebase_client(b)
                    if result:
                        rospy.loginfo("Goal execution done!")
                        text="Reached to" + pl
                        call(["espeak","-s140 -ven+18 -z",text])
                if "bedroom" in b:
                    result,pl = movebase_client(b)
                    if result:
                        rospy.loginfo("Goal execution done!")
                        text="Reached to" + pl
                        call(["espeak","-s140 -ven+18 -z",text])
                prev = now
                f=False

        diff = now - prev
        # print(diff)
        if diff == 10:
            a = False
        
