
# PulseAudio Record and Playback

This application demonstrates the audio record and playback with pulse audio.

## Audio Record


Audio_record application records audio from a pulse audio source. Two formats are supported: WAV and MP3.

**Note:**
-   On board, DIP_SW_0 SW1 must be switched to ON.  
-   If there is Alsa service enabled, please remove Alsa service to make PulseAudio source work

### Start Recording:
Steps for running the audio recording python application in wav and mp3 format.

-   Go to the downloaded directory on RB5 /gst-python-samples/Gstreamer-Applications/gst_audio
```bash
$ cd <path to directory in Git repository>/gst_audio
```
-   Recording audio in wav and mp3 format. Provide the filename along with the format as an argument for the command.
```bash
$ python3 audio_record.py file.wav
$ python3 audio_record.py file.mp3
```

### Stop Automatically:
+ The recording stops automatically in 10 seconds.

## Audio Playback

This application plays audio from a file to pulseaudio sink. Three formats are supported: WAV, AAC and MP3.

**Note:** Speaker must be connected to the board

### Start Playback:
Steps for running the audio playback python application in wav, mp3 and aac format

-   Go to the downloaded directory on RB5 
```bash
$ cd <path to directory in Git repository>/gst_audio
```
    
-   Download the audio files of the wav, mp3 and aac formats in the same folder.
-   Audio playback supports three formats wav, mp3 and aac.
Provide the filename along with the format as an argument for the command.
```bash
$ python3 audio_playback.py file.wav
$ python3 audio_playback.py file.mp3
$ python3 audio_playback.py file.aac
```

