#!/usr/bin/env python
###
### NOTE: THIS SCRIPT IS CURRENTLY NONFUNCTIONAL - it contains some
### experimental code for understanding how to work with AudioData.
###
#===================================================================
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
#
# Copyright 2015, IEEE ENCS Humanoid Robot Project
#===================================================================
import math
import pygst
pygst.require('0.10')
import gst, gobject
gobject.threads_init()
import numpy as np
import rospy
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData

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
	size = len(audio_data.data)
	buffer = gst.buffer_new_and_alloc(size)
	buffer[0:size] = audio_data.data
	src.emit("push-buffer", buffer)
        #rospy.loginfo(rospy.get_caller_id() + ": received audio data")

def enqueue_audio_buffer(appsink):
    b = appsink.emit('pull-buffer')
    # buffer caps=audio/x-raw-int, endianness=(int)1234, signed=(boolean)true, width=(int)32, depth=(int)32, rate=(int)44100, channels=(int)2
    #buffers.append(str(b))
    if b.data != '':
	npb = np.fromstring(b.data, np.dtype('<i4'))
	ch0 = npb[0::2]
	#ch1 = npb[1::2]
	#print "ch0.size=" + str(ch0.size) + ", ch1.size=" + str(ch1.size) + ", mean(ch0)=" + str(np.mean(ch0)) + ", mean(ch1)=" + str(np.mean(ch1))
	std = np.std(ch0)
	if std > 0:
	    print "log10(std(ch0))=" + str(math.log10(std))
	#print "buffer size=" + str(len(b.data))
	#print "buffer caps=" + str(b.get_caps())

def on_new_decoded_pad(dbin, pad, is_last, userdata):
    '''Callback for decodebin linking
    '''
    print "on_new_decoded_pad:"
    print dbin
    print pad
    print is_last
    print userdata
    if not "audio" in pad.get_caps().to_string():
	return
    # see https://github.com/mopidy/mopidy/issues/144
    if not pad.is_linked():
	target_pad = audioconvert.get_pad("sink")
	if target_pad.is_linked():
	    target_pad.get_peer().unlink(target_pad)
	pad.link(target_pad)
	print "link sink to decodebin"
    else:
	print str(pad) + " is linked to " + str(pad.get_peer())

# Intializes everything
def start():
    global publisher, is_listening, audioconvert, src, sink
    is_listening = True
    rospy.Subscriber("speech_info", String, on_speech_info)
    rospy.Subscriber("audio", AudioData, on_audio)
    publisher = rospy.Publisher("audio_segment", AudioData)

    pipeline = gst.parse_launch('appsrc name=src emit-signals=True ! '
				'decodebin name=decode ! '
				#'mad ! '
    				#'audioconvert name=ac ! '
    				#'audio/x-raw-int,channels=2,rate=16000,endianness=1234,width=16,depth=16,signed=(bool)False ! '
				'appsink name=sink emit-signals=True')
    src = pipeline.get_by_name('src')
    decode = pipeline.get_by_name('decode')
    decode.connect("new-decoded-pad", on_new_decoded_pad, None)
    #audioconvert = pipeline.get_by_name('ac')
    sink = pipeline.get_by_name('sink')
    sink.connect('new-buffer', enqueue_audio_buffer)

    pipeline.set_state(gst.STATE_PLAYING)
    print pipeline

    # starts the node
    rospy.init_node('listen_node')
    rospy.spin()

if __name__ == '__main__':
    start()



