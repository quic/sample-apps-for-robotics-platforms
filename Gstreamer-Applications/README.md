# Gstreamer Applications

The purpose of these samples is helping users to learn how to implement the functions of gstreamer on the Qualcomm platform.  These samples include 

+ Camera display
+ Video encoding and decoding
+ Audio recording and playback 
+ Video streaming
+ Video transform
+ GUI application
+ Gst AlsaPlugin
+ Gst TFlite

## Get Started

### Install Qt

GUI application is a Qt application. Some Qt packages are needed.

```bash
$  adb shell
$  apt install qt5-default qtwayland5 qtbase5-private-dev
```

### Install Tool

To display USB camera, v4l2-ctl is a useful tool to list the device files of the camera. It can be installed from v4l-utils pacakge.

```bash
$ adb shell
$ apt install v4l-utils
```

### Download

Download and extract source code to /data/gstreamer-applications/ on board.
```
$ adb shell
$ cd /data
$ git clone https://github.com/quic/sample-apps-for-Qualcomm-Robotics-RB5-platform.git
```
Please connect RJ45(DHCP) of RB5 with network cable or config WiFi by refering to the "Network Setup" section in https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide/qualcomm_robotics_rb5_development_kit_bring_up/set-up-network 

### Compile

GUI application is a Qt application. It needs to run ```qmake``` to generate Makefile before ```make```. 

Other samples can use ```make``` to compile directly.

```bash
$ cd Gstreamer-Applications
$ cd gst_gui_app/
$ qmake
$ cd ..
$ make
```


## Run Samples

Please see detail in each folder.

gst_audio/

+ audio_record
+ audio_playback

gst_camera/

+ ispcam_display
+ usbcam_display

gst_encode_decode/

+ ispcam_encode_mp4
+ usbcam_encode_mp4
+ mp4_decode_playback

gst_gui_app/

+ gui_app

gst_streaming/

+ tcp_server
+ RTSP instruction

gst_transform/

+ transform_display

gst_alsa/

+ gst_audio_sample

gst_tflite/

+ tflite_object_detection

## License
This is licensed under the BSD 3-Clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.
