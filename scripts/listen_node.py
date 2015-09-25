#!/usr/bin/python
#===================================================================
# This is the Listen Node.
#
# Subscribes To:
# - speech_info
# - listen_control
#
# Publishes To:
# - recognized_speech
#
# Node Parameters:
# - in_device      - (default 6) the input device number to listen to
# - out            - (default /recognized_speech) the topic to publish to
# - in_speech_info - (default /speech_info) the speech info topic used to stop
#                    listening when the robot is speeking to avoid feedback
# - in_control     - (default /listen_control) name of topic to receive
#                    control messages to enable/disable listening
#                    TODO upgrade this to work with standard /control topic
# - tmp_wav_file   - (default demo.wav) temporary WAV file name
# - speech_to_text_program - (default ./speech2text.sh) default program to
#                    convert speech to text
# - speech_to_text_output  - (default stt.txt) output file from speech to text
#                    program containing the text
# - max_samples    - (default 200000) maximum number of sound samples to attempt
#                    to convert to text
#
# The listen node opens an audio stream from the specified input device.
# It performs voice activity detection to segment the audio into speech
# and silence segments.  Speech segments are written to a WAV file and
# the speech2text.sh shell script is called to convert the WAV file to
# text.  The resulting text is read from stt.text and published to the
# recognized_speech topic.
#
# Copyright 2015, IEEE ENCS Humanoid Robot Project
#===================================================================

from __future__ import division
from sys import byteorder
from array import array
from struct import pack
from std_msgs.msg import String
from subprocess import call

import collections
import pyaudio
import rospy
import threading
import time
import traceback
import wave

MIN_THRESHOLD = 10000
THRESHOLD = MIN_THRESHOLD
CHUNK_SIZE = 1024  # 64ms
FORMAT = pyaudio.paInt16
RATE = 16000


class Segment(object):
    def __init__(self, sample_width, data):
	self.sample_width = sample_width
	self.data = data


class ListenerThread(threading.Thread):
    def __init__(self, input_device, max_sound_samples, group=None, target=None, name=None, args=(), kwargs=None, verbose=None):
	threading.Thread.__init__(self, group=group, target=target, name=name, verbose=verbose)
        self.args = args
        self.kwargs = kwargs
	self.input_device = input_device
	self.max_sound_samples = max_sound_samples
	self.p = None  # initialized in run()
	self.stream = None  # initialized in run()
	self.last_segment = None
	self.is_listening = True
	self.is_shutdown = False
	self.is_reset_requested = False
	self.is_reset_in_progress = False
	self.reset_start_time_in_sec = 0
	self.reset_duration_in_sec = 3
        return


    def run(self):
    	global FORMAT, CHUNK_SIZE, RATE
	num_retries = 30
	time.sleep(5)  # try to prevent crash when starting at boot
	self.p = pyaudio.PyAudio()
	# attempt to connect to the audio stream.  on startup, this may
	# initially fail, so retry a few times
	for num_tries in range(num_retries):
	    try:
		rospy.loginfo(rospy.get_caller_id() + ": starting audio listener for input device " + str(self.input_device) + " try " + str(num_tries + 1))

		self.stream = None
		self.stream = self.p.open(format=FORMAT, channels=1, rate=RATE, input=True,
					  frames_per_buffer=CHUNK_SIZE, input_device_index=self.input_device)
		break
	    except:
		traceback.print_exc()
		time.sleep(1)

	if self.stream is None:
	    rospy.loginfo(rospy.get_caller_id() + ": failed to start audio listener for input device " + str(self.input_device) + " after " + str(num_retries) + " tries")
	    exit(1)

	while not self.is_shutdown:
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


    def pop_last_segment(self):
    	segment = self.last_segment
	self.last_segment = None
	return segment


    def is_silent(self, snd_data):
	"Returns 'True' if below the 'silent' threshold"
	global THRESHOLD
	m = max(snd_data)
	n = min(snd_data)
	val = m - n
	return val < THRESHOLD


    def set_silence_threshold(self, threshold):
    	global THRESHOLD
	THRESHOLD = threshold


    def reset_silence_threshold(self):
	"""
	Request the listener thread to reset the silence threshold
	"""
	self.is_reset_requested = True  # flag the background thread to start the reset
	while self.is_reset_requested:
	    # sleep while the reset happens in the background thread
	    time.sleep(0.1)


    def _update_silence_threshold(self, snd_data):
        """
	Monitor the sound stream for a few seconds and reset the threshold
	to the maximum amplitude encountered.
	"""
	global THRESHOLD, MIN_THRESHOLD
	if not self.is_reset_in_progress:
	    # capture start time - first time only
	    self.reset_start_time_in_sec = time.time()
	    self.is_reset_in_progress = True
	    # initialize new threshold
	    THRESHOLD = max(MIN_THRESHOLD, max(snd_data) - min(snd_data))
	    rospy.loginfo(rospy.get_caller_id() + ": set threshold to " + str(THRESHOLD))
	else:
	    # update threshold
	    THRESHOLD = max(THRESHOLD, max(snd_data) - min(snd_data))
	    rospy.loginfo(rospy.get_caller_id() + ": reset threshold to " + str(THRESHOLD))
	    # test if current time - start time exceeds reset duration
	    if time.time() - self.reset_start_time_in_sec >= self.reset_duration_in_sec:
		self.is_reset_requested = False
		self.is_reset_in_progress = False
		self.reset_start_time_in_sec = 0


    def record(self):
	"""
	Record a segment of speech from the microphone and 
	return the data as an array of signed shorts.

	Speech is distinguished from silence by:
	1) max - min amplitude exceeds a threshold in a frame
	2) at least min_voiced_frames consecutive nonsilent frames
	"""
	global THRESHOLD, MIN_THRESHOLD, CHUNK_SIZE, FORMAT
	rospy.loginfo(rospy.get_caller_id() + ": start recording")
	end_num_silent_frames = 15  # 15 => 960ms; 30 => 1920ms
	start_num_silent_frames = 5  # 5 => 320ms
	min_voiced_frames = 2  # 2 => 128ms
	has_min_voiced_frames = False

	while not has_min_voiced_frames and not self.is_shutdown:
	    num_silent = 0
	    num_voiced = 0
	    snd_started = False

	    d = collections.deque(maxlen=start_num_silent_frames)
	    r = array('h')

	    while not self.is_shutdown and len(r) < self.max_sound_samples:
		# little endian, signed short
		snd_data = array('h', self.stream.read(CHUNK_SIZE))
		if self.is_listening:
		    if byteorder == 'big':
			snd_data.byteswap()

		    if self.is_reset_requested:
			self._update_silence_threshold(snd_data)

		    silent = self.is_silent(snd_data)

		    if not snd_started:
			if silent:
			    d.append(snd_data)  # cache initial silent frames
			else:
			    snd_started = True
			    for s in d:
				r.extend(s)  # copy initial silent frames to avoid clipping speech
			    d.clear()
			    r.extend(snd_data)
			    rospy.loginfo(rospy.get_caller_id() + ": sound started")
		    else:
			r.extend(snd_data)

			if silent:
			    num_silent += 1  # count consecutive silent frames
			    num_voiced = 0   # clear consecutive voiced count for silent frame
			else:
			    num_silent = 0   # clear consecutive silent count for voiced frame
			    num_voiced += 1  # count consecutive voiced frames

			if not has_min_voiced_frames and num_voiced >= min_voiced_frames:
			    has_min_voiced_frames = True
			    rospy.loginfo(rospy.get_caller_id() + ": voice detected")

			if num_silent >= end_num_silent_frames:
			    rospy.loginfo(rospy.get_caller_id() + ": sound ended")
			    break

	rospy.loginfo(rospy.get_caller_id() + ": stop recording, recorded " + str(len(r)))
	sample_width = self.p.get_sample_size(FORMAT)

	return sample_width, r


class ListenNode(object):
    def __init__(self):
	rospy.init_node('listen_node')

	self.is_listening_enabled = True

	device = int(self.get_param('~device', '6'))
	speech_topic = self.get_param('~out', '/recognized_speech')
	speech_info_topic = self.get_param('~in_speech_info', '/speech_info')
	# TODO change /listen_control to /control and update AI and Humanoid Control app
	control_topic = self.get_param('~in_control', '/listen_control')
	self.tmp_wav_file = self.get_param('~tmp_wav_file', 'demo.wav')
	self.speech_to_text_program = self.get_param('~speech_to_text_program', './speech2text.sh')
	self.speech_to_text_output = self.get_param('~speech_to_text_output', 'stt.txt')
	self.max_sound_samples = int(self.get_param('~max_samples', '200000'))

	rospy.Subscriber(speech_info_topic, String, self.on_speech_info)
	rospy.Subscriber(control_topic, String, self.on_listen_control)
	self.publisher = rospy.Publisher(speech_topic, String, queue_size=1)
	self.listener = ListenerThread(device, self.max_sound_samples)


    def get_param(self, param_name, param_default):
	value = rospy.get_param(param_name, param_default)
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param_name), value)
	return value


    def run(self):
	# initialize a thread to read and segment audio from the mic
	# the thread should read continuously to avoid input buffer overflow on the audio stream
	self.listener.setDaemon(True)
	self.listener.start()

	try:
	    rospy.loginfo('starting main loop')
	    rate = rospy.Rate(10)
	    while not rospy.is_shutdown():
	    	segment = self.listener.pop_last_segment()
		if segment is not None:
		    self.publish_speech_segment(segment)
		rate.sleep() # give ROS a chance to run
	except:
	    traceback.print_exc()

	rospy.loginfo('shutting down')
	self.listener.is_shutdown = True
	self.listener.join(1)


    def publish_speech_segment(self, segment):
	self.write_wav_file(segment, self.tmp_wav_file)
	call([self.speech_to_text_program])
	with open(self.speech_to_text_output) as f:
	    text = str(f.read()).strip()
	    if text != "":
		self.publisher.publish(text)


    def write_wav_file(self, segment, path):
	"Write the speech segment data in WAV format to 'path'"
	global RATE
	size = len(segment.data)
	if size > self.max_sound_samples:
	    rospy.loginfo(rospy.get_caller_id() + ": truncating sound length " + str(size) + " to " + str(self.max_sound_samples) + " samples")
	    # keep the most recent samples (at the end)
	    segment.data = segment.data[-(self.max_sound_samples + 1):]
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


    def on_speech_info(self, msg):
	if msg.data == "start_speaking":
	    self.listener.is_listening = False
	    rospy.loginfo(rospy.get_caller_id() + ": start speaking")
	elif msg.data == "done_speaking":
	    self.listener.is_listening = self.is_listening_enabled
	    # remove any segment that might have been stored between the time of the last
	    # segment processed and the start of speaking.
	    self.listener.pop_last_segment()
	    rospy.loginfo(rospy.get_caller_id() + ": done speaking")
	else:
	    rospy.loginfo(rospy.get_caller_id() + ": unrecognised speech_info message %s", msg.data)


    # listen control is separate from speech info handling to allow a mechanism
    # to turn off listening without interfering with other speech processes
    def on_listen_control(self, msg):
    	global THRESHOLD
	if msg.data == "stop_listening":
	    self.is_listening_enabled = self.listener.is_listening = False
	    self.publisher.publish("KEN STOP LISTENING")
	    rospy.loginfo(rospy.get_caller_id() + ": stop listening")
	elif msg.data == "resume_listening":
	    self.is_listening_enabled = self.listener.is_listening = True
	    self.publisher.publish("KEN RESUME LISTENING")
	    rospy.loginfo(rospy.get_caller_id() + ": resume listening")
	elif msg.data == "reset_silence_threshold":
	    self.listener.reset_silence_threshold()
	    self.publisher.publish("SILENCE THRESHOLD SET TO " + str(THRESHOLD))
	    rospy.loginfo(rospy.get_caller_id() + ": silence threshold set to " + str(THRESHOLD))
	elif len(msg.data.split()) > 1:
	    cmd = msg.data.split()
	    if cmd[0] == "set_silence_threshold":
	    	self.listener.set_silence_threshold(int(cmd[1]))
		self.publisher.publish("SILENCE THRESHOLD SET TO " + str(THRESHOLD))
		rospy.loginfo(rospy.get_caller_id() + ": silence threshold set to " + str(THRESHOLD))
	    else:
		rospy.loginfo(rospy.get_caller_id() + ": unrecognised listen_control message %s", cmd[0])
	else:
	    rospy.loginfo(rospy.get_caller_id() + ": unrecognised listen_control message %s", msg.data)


if __name__ == '__main__':
    try:
	node = ListenNode()
	node.run()
    except rospy.ROSInterruptException:
	pass

