#!/usr/bin/env python
# Produce audible English speech given a text string.
#
# The Speak Node listens to the /say topic and passes the received
# text string to a speech synthesis engine to produce audible output.
# The node also publishes messages to indicate when speech is
# starting and stopping, which are intended to be used to turn off
# speech recognition while the robot is speaking to avoid feedback
# through the system.

import rospy
from std_msgs.msg import String
from subprocess import call
from pipes import quote
from time import sleep


class SpeakNode(object):
    def __init__(self):
	rospy.init_node('speak_node')

	info_topic = self.get_param('~out_speech_info', '/speech_info')
	say_topic = self.get_param('~in_say', '/say')
	self.voice = self.get_param('~voice', 'english-us')
	self.speed = int(self.get_param('~speed', '140'))
	self.pitch = int(self.get_param('~pitch', '80'))

	# the speech_info topic publishes messages when speaking starts and stops
	self.publisher = rospy.Publisher(info_topic, String) #, queue_size=2)

	# messages coming in on the say topic will be spoken aloud
	rospy.Subscriber(say_topic, String, self.on_say)


    def get_param(self, param_name, param_default):
	value = rospy.get_param(param_name, param_default)
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param_name), value)
	return value


    def run(self):
    	rospy.spin()


    def on_say(self, msg):
	self.publisher.publish("start_speaking")
	sleep(0.5)
	espeak_cmd = "espeak -v %s -s %d -p %d -g 2 " % (self.voice, self.speed, self.pitch)
	call([espeak_cmd + quote(msg.data) + " --stdout | aplay"], shell=True)
	self.publisher.publish("done_speaking")


if __name__ == '__main__':
    try:
	node = SpeakNode()
	node.run()
    except rospy.ROSInterruptException:
	pass

