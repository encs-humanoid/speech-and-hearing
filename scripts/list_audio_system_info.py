"""
http://people.csail.mit.edu/hubert/git/?p=pyaudio.git;a=blob;f=test/system_info.py;h=3c81432d7ffa3feaae06fa9804f41ffa03d4865d;hb=HEAD#l49
"""

import pyaudio

standard_sample_rates = [8000.0, 9600.0, 11025.0, 12000.0,
                         16000.0, 22050.0, 24000.0, 32000.0,
                         44100.0, 48000.0, 88200.0, 96000.0,
                         192000.0]

p = pyaudio.PyAudio()
max_apis = p.get_host_api_count()
max_devs = p.get_device_count()

print "\nPortAudio System Info:\n======================"
print "Version: %d" % pyaudio.get_portaudio_version()
print "Version Text: %s" % pyaudio.get_portaudio_version_text()
print "Number of Host APIs: %d" % max_apis
print "Number of Devices  : %d" % max_devs

print "\nHost APIs:\n=========="
for i in range(max_apis):
	apiinfo = p.get_host_api_info_by_index(i)
	for k in apiinfo.items():
		print "%s: %s" % k


print "\nDevices:\n========"

for i in range(max_devs):
    devinfo = p.get_device_info_by_index(i)
    print "i=", i
    # print out device parameters
    for k in devinfo.items():
        name, value = k

        # if host API, then get friendly name
        
        if name == 'hostApi':
            value = str(value) + \
                    " (%s)" % p.get_host_api_info_by_index(k[1])['name']
        print "\t%s: %s" % (name, value)
        
    # print out supported format rates
    
    input_supported_rates = []
    output_supported_rates = []
    full_duplex_rates = []
    
    for f in standard_sample_rates:

        if devinfo['maxInputChannels'] > 0:
            try:
                if p.is_format_supported(
                    f,
                    input_device = devinfo['index'],
                    input_channels = devinfo['maxInputChannels'],
                    input_format = pyaudio.paInt16):
                    input_supported_rates.append(f)
            except ValueError:
                pass

        if devinfo['maxOutputChannels'] > 0:
            try:
                if p.is_format_supported(
                    f,
                    output_device = devinfo['index'],
                    output_channels = devinfo['maxOutputChannels'],
                    output_format = pyaudio.paInt16):
                    output_supported_rates.append(f)
            except ValueError:
                pass

        if (devinfo['maxInputChannels'] > 0) and \
           (devinfo['maxOutputChannels'] > 0):
            try:
                if p.is_format_supported(
                    f,
                    input_device = devinfo['index'],
                    input_channels = devinfo['maxInputChannels'],
                    input_format = pyaudio.paInt16,
                    output_device = devinfo['index'],
                    output_channels = devinfo['maxOutputChannels'],
                    output_format = pyaudio.paInt16):
                    full_duplex_rates.append(f)
            except ValueError:
                pass

    if len(input_supported_rates):
        print "\tInput rates:", input_supported_rates
    if len(output_supported_rates):
        print "\tOutput rates:", output_supported_rates
    if len(full_duplex_rates):
        print "\tFull duplex: ", full_duplex_rates

	print "\t--------------------------------"

print "\nDefault Devices:\n================"
try:
    def_index = p.get_default_input_device_info()['index']
    print "Default Input Device :", def_index    
    devinfo = p.get_device_info_by_index(def_index)
    for k in devinfo.items():
        name, value = k
        if name == 'hostApi':
            value = str(value) + \
                    " (%s)" % p.get_host_api_info_by_index(k[1])['name']
        print "\t%s: %s" % (name, value)
    print "\t--------------------------------"
except IOError, e:
    print "No Input devices: %s" % e[0]

p.terminate()

