#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from subprocess import call
from pipes import quote
from time import sleep

# Produce audible English speech given a text string.
#
# The Speak Node listens to the /say topic and passes the received
# text string to a speech synthesis engine to produce audible output.
# The node also publishes messages to indicate when speech is
# starting and stopping, which are intended to be used to turn off
# speech recognition while the robot is speaking to avoid feedback
# through the system.

def on_say(msg):
    publisher.publish("start_speaking")
    sleep(0.5)
    call(["espeak -v english-us -s 140 -p 80 -g 2 " + quote(msg.data) + " --stdout | aplay"], shell=True)
    publisher.publish("done_speaking")

# Intializes everything
def start():
    global publisher
    # starts the node
    rospy.init_node('speak_node')
    # subscribed to text inputs on topic "speech"
    rospy.Subscriber("say", String, on_say)
    publisher = rospy.Publisher("speech_info", String)
    rospy.spin()

if __name__ == '__main__':
    start()



