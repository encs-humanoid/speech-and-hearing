#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

# Perform speech recognition on segmented audio signals producing recognized
# speech as text.  
#
# Different implementations of the Recognize Speech node may use different
# implementations, such as the CMU Sphinx library, Google Speech API, or
# other speech recognizer.  Regardless of implementation, the node takes
# an Audio Data message which is already segmented (see Listen Node) and
# processes it to produce a text string containing the recognized speech.

def on_audio_segment(audio_data):
    # This is where the speech recognition goes
    # TODO recognize the spoken language in the audio segment and
    # publish the text result using the publisher
    publisher.publish("Sorry, I didn't get that.")

# Intializes everything
def start():
    global publisher
    rospy.Subscriber("audio_segment", AudioData, on_audio_segment)
    publisher = rospy.Publisher("recognized_speech", String)
    # starts the node
    rospy.init_node('recognize_speech_node')
    rospy.spin()

if __name__ == '__main__':
    start()



