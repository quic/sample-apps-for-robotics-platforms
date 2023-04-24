# GstAlsaPlugin-Application
```
A sample app based on gstreamer
The purpose is helping users to learn how to implement the playback and recording functions of gstreamer+alsa
on the QTI platform through this sample app.
```

# Get Start

### 1. Set up the environment on PC
Install RB5 LU SDK and source the environment
Download sample code to PC

### 2. Compile
After execute the make command, the binary gst_audio_sample will be created.
```
# cd GstAlsaPlugin-Application/src
# make
```

### 3. Push binary to device
```
$ adb disable-verity
$ adb reboot
$ adb wait-for-device root
### The above three steps only need to be operated once and will always be valid.

adb push gst_audio_sample /data
cd ../HWConfig
adb push install.sh /data
adb push qc-alsa-restore.service /data
adb push qc_alsa_restore.sh /data
adb shell mount -o remount,rw /
chmod 777 /data/gst_audio_sample
chmod 777 /data/install.sh
```

## QC ALSA restore service (Once need to execute once)
We provided a ALSA restore service, to enable speaker and dmic as default.
Please follow the steps to enable it
```
adb shell
cd /data
./install.sh
```

![Image text](image/05_init_service.png)

After enable this HWConfig service, please reboot the device:
```
# exit
$ adb reboot
```

BTW, if you want to disable this service, please run:
```
systemctl disable qc-alsa-restore
```

### 4. Test
After compile the sample code, and enable service.
We can use the app to test with playback and capture.
###  Playback
For playback, please use below command to playback.
You need to choose your demo.wav audio file.
```
# cd /data
# ./gst_audio_sample playback /data/16bit_48kHz.wav
```
![Image text](image/01_playback.png)

### Capture
Below command will record about 10 seconds
```
# ./gst_audio_sample capture /data/capture.wav
```
After record, we can verified it by playback
```
# ./gst_audio_sample playback /data/capture.wav
```
![Image text](image/03_capture.png)
![Image text](image/04_capture_playback.png)

## License
This is licensed under the BSD 3-Clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.
