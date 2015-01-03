#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

# Monitor the audio stream coming from the audio_capture node and segment
# it for recognition analysis.
#
# The Listen node has two responsibilities.  It's primary responsibility
# is to detect a sustained sound signal exceeding a configured intensity
# threshold and record the sound waveform until it detects a sustained
# silence below the intensity threshold.  It then publishes the captured
# waveform for processing by other nodes, such as for speech recognition.
# It's second responsibility is to prevent the capture of sound while
# the robot is producing its own speech.  To do this, it listens on the
# /speech_info topic.

def on_speech_info(msg):
    if msg.data == "start_speaking":
	is_listening = False
        rospy.loginfo(rospy.get_caller_id() + ": stop listening")
    elif msg.data == "done_speaking":
	is_listening = True
        rospy.loginfo(rospy.get_caller_id() + ": resume listening")
    else:
        rospy.loginfo(rospy.get_caller_id() + ": unrecognised speech_info message %s", msg.data)

def on_audio(audio_data):
    if is_listening:
	# TODO identify audio segments and batch up the data into a new AudioData message
	# publish the new message using the publisher
        rospy.loginfo(rospy.get_caller_id() + ": received audio data")

# Intializes everything
def start():
    global publisher, is_listening
    is_listening = True
    rospy.Subscriber("speech_info", String, on_speech_info)
    rospy.Subscriber("audio", AudioData, on_audio)
    publisher = rospy.Publisher("audio_segment", AudioData)
    # starts the node
    rospy.init_node('listen_node')
    rospy.spin()

if __name__ == '__main__':
    start()



