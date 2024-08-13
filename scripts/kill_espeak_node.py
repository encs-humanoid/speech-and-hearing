#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from subprocess import call
from pipes import quote
from time import sleep

# Kill all running espeak processes.
#
# The Kill Espeak Node listens to the /control topic and kills all
# espeak processes on the local host if the cancel_speaking command
# is received.

def on_control(msg):
    if msg.data == "cancel_speaking":
        call(["killall espeak"], shell=True)

# Intializes everything
def start():
    # starts the node
    rospy.init_node('kill_espeak_node')
    # subscribed to control commands on topic "control"
    rospy.Subscriber("control", String, on_control)
    rospy.spin()

if __name__ == '__main__':
    start()

