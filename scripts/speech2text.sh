#!/bin/bash

#echo "Recording your Speech (Ctrl+C to Transcribe)"
#arecord -D plughw:1,0 -q -f cd -t wav -r 16000 | 

# convert WAV file to FLAC file
flac demo.wav -f --best --sample-rate 16000 -s -o file.flac

echo "Converting Speech to Text..."

# Get the Google Speech API key from the file where it lives
apikey=`cat apikey.txt | xargs`

# call Google Speech API to recognize what was spoken
wget -q -U "Mozilla/5.0" --post-file file.flac --header "Content-Type: audio/x-flac; rate=16000" -O - "http://www.google.com/speech-api/v2/recognize?lang=en-us&client=chromium&key=${apikey}" | cut -d\" -f8 > stt.txt 
#aplay demo.wav
value=`cat stt.txt`
echo "You Said:"
echo "$value"
mv demo.wav demo_old.wav
#if [ -n "$value" ]; then
#    rostopic pub recognized_speech std_msgs/String -1 "$value"
#fi
