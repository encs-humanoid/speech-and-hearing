#!/usr/bin/python
from sys import byteorder
from array import array
from struct import pack
from std_msgs.msg import String
from subprocess import call

import pyaudio
import rospy
import threading
import time
import traceback
import wave

THRESHOLD = 10000
CHUNK_SIZE = 1024
FORMAT = pyaudio.paInt16
RATE = 16000


class Segment(object):
    def __init__(self, sample_width, data):
	self.sample_width = sample_width
	self.data = data


class ListenerThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, verbose=None):
	threading.Thread.__init__(self, group=group, target=target, name=name, verbose=verbose)
        self.args = args
        self.kwargs = kwargs
	self.p = None  # initialized in run()
	self.stream = None  # initialized in run()
	self.last_segment = None
        return

    def run(self):
	global shutdown

	input_device = 6
	num_retries = 10
	time.sleep(5)  # try to prevent crash when starting at boot
	self.p = pyaudio.PyAudio()
	# attempt to connect to the audio stream.  on startup, this may
	# initially fail, so retry a few times
	for num_tries in range(num_retries):
	    try:
		rospy.loginfo(rospy.get_caller_id() + ": starting audio listener for input device " + str(input_device) + " try " + str(num_tries + 1))

		self.stream = None
		self.stream = self.p.open(format=FORMAT, channels=1, rate=RATE, input=True,
					  frames_per_buffer=CHUNK_SIZE, input_device_index=input_device)
		break
	    except:
		traceback.print_exc()
		time.sleep(1)

	if self.stream is None:
	    rospy.loginfo(rospy.get_caller_id() + ": failed to start audio listener for input device " + str(input_device) + " after " + str(num_retries) + " tries")
	    exit(1)

	while not shutdown:
	    # determine speech segment from input audio
	    # store last speech segment recorded while listening
	    self.last_segment = Segment(*self.record())
	# close audio stream on exit
	rospy.loginfo(rospy.get_caller_id() + ": stopping audio listener")
	self.stream.stop_stream()
	self.stream.close()
	self.p.terminate()
	rospy.loginfo(rospy.get_caller_id() + ": audio listener terminated")
	return

    def is_silent(self, snd_data):
	"Returns 'True' if below the 'silent' threshold"
	m = max(snd_data)
	n = min(snd_data)
	val = m - n
	return val < THRESHOLD

    def record(self):
	"""
	Record a segment of speech from the microphone and 
	return the data as an array of signed shorts.
	"""
	global is_listening

	num_silent = 0
	snd_started = False

	r = array('h')

	rospy.loginfo(rospy.get_caller_id() + ": start recording")
	while 1:
	    # little endian, signed short
	    snd_data = array('h', self.stream.read(CHUNK_SIZE))
	    if is_listening:
		if byteorder == 'big':
		    snd_data.byteswap()

		silent = self.is_silent(snd_data)

		if not silent and not snd_started:
		    snd_started = True
		    rospy.loginfo(rospy.get_caller_id() + ": sound started")

		if silent and snd_started:
		    num_silent += 1

		if snd_started and num_silent > 30:
		    rospy.loginfo(rospy.get_caller_id() + ": sound ended")
		    break

		if snd_started:
		    r.extend(snd_data)

	rospy.loginfo(rospy.get_caller_id() + ": stop recording, recorded " + str(len(r)))
	sample_width = self.p.get_sample_size(FORMAT)

	return sample_width, r


def record_to_file(segment, path):
    "Write the speech segment data in WAV format to 'path'"
    max_sound_samples = 200000  # TODO make this into a node parameter
    size = len(segment.data)
    if size > max_sound_samples:
	rospy.loginfo(rospy.get_caller_id() + ": truncating sound length " + str(size) + " to " + str(max_sound_samples) + " samples")
	segment.data = segment.data[:max_sound_samples + 1]
	size = len(segment.data)
	
    rospy.loginfo(rospy.get_caller_id() + ": writing sound length " + str(size) + " width " + str(segment.sample_width) + " to " + path)
    data = pack('<' + ('h'*size), *segment.data)

    wf = wave.open(path, 'wb')
    wf.setnchannels(1)
    wf.setsampwidth(segment.sample_width)
    wf.setframerate(RATE)
    wf.writeframes(data)
    wf.close()
    rospy.loginfo(rospy.get_caller_id() + ": wrote " + str(len(data)) + " bytes to " + path)


def on_speech_info(msg):
    global is_listening, listening_enabled
    if msg.data == "start_speaking":
	is_listening = False
        rospy.loginfo(rospy.get_caller_id() + ": start speaking")
    elif msg.data == "done_speaking":
	is_listening = listening_enabled
        rospy.loginfo(rospy.get_caller_id() + ": done speaking")
    else:
        rospy.loginfo(rospy.get_caller_id() + ": unrecognised speech_info message %s", msg.data)


# listen control is separate from speech info handling to allow a mechanism
# to turn off listening without interfering with other speech processes
def on_listen_control(msg):
    global is_listening, listening_enabled, publisher
    if msg.data == "stop_listening":
	listening_enabled = False
	publisher.publish("KEN STOP LISTENING")
	rospy.loginfo(rospy.get_caller_id() + ": stop listening")
    elif msg.data == "resume_listening":
	listening_enabled = True
	publisher.publish("KEN RESUME LISTENING")
	rospy.loginfo(rospy.get_caller_id() + ": resume listening")
    else:
	rospy.loginfo(rospy.get_caller_id() + ": unrecognised listen_control message %s", msg.data)
    is_listening = listening_enabled


if __name__ == '__main__':
    global is_listening, listening_enabled, publisher, shutdown
    is_listening = True
    listening_enabled = True
    shutdown = False
    rospy.init_node('listen_node')
    rospy.Subscriber("speech_info", String, on_speech_info)
    rospy.Subscriber("listen_control", String, on_listen_control)
    publisher = rospy.Publisher("recognized_speech", String, queue_size=1)

    # initialize a thread to read and segment audio from the mic
    # the thread should read continuously to avoid input buffer overflow on the audio stream
    t = ListenerThread()
    t.setDaemon(True)
    t.start()

    try:
	print("in main thread")
	rate = rospy.Rate(10)
	while True:
	    if t.last_segment is not None:
		segment = t.last_segment
		t.last_segment = None
		record_to_file(segment, 'demo.wav')
		call(["./speech2text.sh"])
		with open("stt.txt") as f:
		    text = str(f.read()).strip()
		    if text != "":
			publisher.publish(text)
	    rate.sleep() # give ROS a chance to run
    except KeyboardInterrupt:
	pass
    except:
	traceback.print_exc()
    print("set shutdown=True")
    shutdown = True

    t.join(1)

