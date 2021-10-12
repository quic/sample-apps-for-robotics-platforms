# Gst_ALSA
The purpose of this application is to implement the playback and recording functions of gstreamer+alsa on the QTI platform.

## Note:

-   If pulseaudio service is enabled, disable it and start the alsa-restore service.
```bash
$ systemctl stop pulseaudio
$ apt-get install alsa-utils
$ systemctl start alsa-restore.service
```
BTW, if you want to disable this service, please run:
```
$ systemctl disable qc-alsa-restore
```
### Steps to run alsa playback and recording application
-   Go to the downloaded directory on RB5 
```bash
cd /gst-python-samples/Gstreamer-Applications/gst-alsa
```
- Run the alsa playback and record application

-**Alsa audio playback:** Provide playback as the first argument for audio playback. 
```bash
python3 gst-alsa playback audiofile.wav
```
-**Alsa audio recording:** Provide capture as the first argument for alsa audio recording.
```bash
python3 gst-alsa capture audiorecord.wav