#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from subprocess import call

# This ROS Node converts String inputs into speech using espeak

# Received String message (subscribed to speech topic)
# then calls out to the espeak program to convert text to speech

def callback(text):
    call(["espeak", text.data])

# Intializes everything
def start():
    # subscribed to text inputs on topic "speech"
    rospy.Subscriber("speech", String, callback)
    # starts the node
    rospy.init_node('espeak_node')
    rospy.spin()

if __name__ == '__main__':
    start()



