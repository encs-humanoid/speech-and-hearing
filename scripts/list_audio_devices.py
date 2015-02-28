#!/usr/bin/python
import pyaudio

p = pyaudio.PyAudio()
print "device count " + str(p.get_device_count())

for i in xrange(p.get_device_count()):
	info = p.get_device_info_by_index(i)
	print "Device " + str(i) + ": " + str(info)

p.terminate()
