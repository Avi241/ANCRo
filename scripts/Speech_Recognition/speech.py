#! /usr/bin/env python3
import queue
import sounddevice as sd
import vosk
import rospy
from std_msgs.msg import String
import time

a = True
c = True
f = True
now = 0
prev = 0

rospy.init_node("Speech_Recognition")
pub = rospy.Publisher("phrases", String, queue_size=10)
rate = rospy.Rate(2)
msg_str = String()
msg_str = ""

q = queue.Queue()


def wakecb(hey):
    print(hey)
    global a
    global prev, now
    a = True
    prev = now


def flagcb(hi):
    print("flag called")
    global f
    f = True


def callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))


model = vosk.Model("model")
device_info = sd.query_devices(None, "input")
sub = rospy.Subscriber('/wake', String, wakecb)
fsub = rospy.Subscriber('/flag', String, flagcb)

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
        print(a)
        if c == True:
            prev = now
            c = False
        now = int(time.time())
        data = q.get()
        if rec.AcceptWaveform(data):
            b = (eval(rec.Result()))["text"]
            # print(b)
            if len(b) >= 2:
                print(b)
                pub.publish(b)
                prev = now
                f = False
