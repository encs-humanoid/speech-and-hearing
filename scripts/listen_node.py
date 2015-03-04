#!/usr/bin/python
from sys import byteorder
from array import array
from struct import pack
import rospy
from std_msgs.msg import String

import pyaudio
import wave
from subprocess import call

THRESHOLD = 10000
CHUNK_SIZE = 1024
FORMAT = pyaudio.paInt16
RATE = 16000
#RATE = 44100
#RATE =8000
is_listening = True

maxval = -9999
def is_silent(snd_data):
    global maxval
    "Returns 'True' if below the 'silent' threshold"
    m = max(snd_data)
    n = min(snd_data)
    val = m - n
    if (val > maxval):
	maxval = val 
    return val < THRESHOLD

def record():
    """
    Record a word or words from the microphone and 
    return the data as an array of signed shorts.
    """
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT, channels=1, rate=RATE,
        input=True,#output=True,
        frames_per_buffer=CHUNK_SIZE, input_device_index=2)

    num_silent = 0
    snd_started = False

    r = array('h')

    rate = rospy.Rate(44100) # 44100 Hz - not a meaningful frequency, just a small delay
    while 1:
	rate.sleep() # give ROS a chance to run
	if is_listening:
		# little endian, signed short
		snd_data = array('h', stream.read(CHUNK_SIZE))
		if byteorder == 'big':
		    snd_data.byteswap()
		r.extend(snd_data)

		silent = is_silent(snd_data)

		if silent and snd_started:
		    num_silent += 1
		elif not silent and not snd_started:
		    snd_started = True

		if snd_started and num_silent > 30:
		    break

    sample_width = p.get_sample_size(FORMAT)
    stream.stop_stream()
    stream.close()
    p.terminate()

    return sample_width, r

def record_to_file(path):
    "Records from the microphone and outputs the resulting data to 'path'"
    sample_width, data = record()
    data = pack('<' + ('h'*len(data)), *data)

    wf = wave.open(path, 'wb')
    wf.setnchannels(1)
    wf.setsampwidth(sample_width)
    wf.setframerate(RATE)
    wf.writeframes(data)
    wf.close()

def on_speech_info(msg):
    if msg.data == "start_speaking":
	is_listening = False
        rospy.loginfo(rospy.get_caller_id() + ": stop listening")
    elif msg.data == "done_speaking":
	is_listening = True
        rospy.loginfo(rospy.get_caller_id() + ": resume listening")
    else:
        rospy.loginfo(rospy.get_caller_id() + ": unrecognised speech_info message %s", msg.data)

if __name__ == '__main__':
    global is_listening
    is_listening = True
    rospy.Subscriber("speech_info", String, on_speech_info)
    publisher = rospy.Publisher("recognized_speech", String)
    rospy.init_node('listen_node')

    while True:
	print("please speak a word into the microphone")
	record_to_file('demo.wav')
	print("done - result written to demo.wav")
	call(["./speech2text.sh"])
	with open("stt.txt") as f:
	    text = f.read()
	    if text != "":
		publisher.publish(text)

