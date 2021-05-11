# PulseAudio Record and Playback

This sample demostrates audio recording and playback with PulseAudio.

## Audio Record

```audio_record``` records audio from PulseAudio source. Three formats are supported: WAV, AAC, and MP3.

Usage: audio_record /FILE/PATH/FILENAME.[wav|aac|mp3]

### Start Recording:
```bash
$ cd /data/sample-apps-for-Qualcomm-Robotics-RB5-platform/Gstreamer-Applications/gst_audio
$ ./audio_record record.wav
$ ./audio_record record.aac
$ ./audio_record record.mp3
```

### Stop Automatically:

+ The recording stops in 10 seconds.

### Stop Manually:

+ Press Ctrl-C to stop recording

### Note:

+ On board, DIP_SW_0 SW1 must be switched to ON. Please refer to related hardware document
+ If there is Alsa service enabled, please remove Alsa service to make PulseAudio source work

## Audio Playback

```audio_playback``` plays audio from a file to PulseAudio sink. Three formats are supported: WAV, AAC, and MP3.

Usage: audio_playback /FILE/PATH/FILENAME.[wav|aac|mp3]

### Example:
```bash
$ cd /data/Gstreamer-Applications/gst_audio
$ ./audio_playback record.wav
$ ./audio_playback record.aac
$ ./audio_playback record.mp3
```

### Note:

+ A speaker must be connected to the board. Please refer to related hardware document 

## License
This is licensed under the BSD 3-Clause-Clear “New” or “Revised” License. Check out the [LICENSE](../LICENSE) for more details.
