# Gstreamer Applications

The purpose of these samples is helping users to learn how to implement the functions of gstreamer on the Qualcomm platform.  These samples include 

+ Camera display
+ Video encoding and decoding
+ Audio recording and playback 
+ Video streaming
+ Video transform

### 1. Set up the environment on PC
Install RB5 LU SDK and source the environment
Download sample code to PC

### 2. Compile
```
$ cd Gstreamer-Applications/
$ make
```

### 3. Push binaries to the device
```
$ adb push [binary] /data
```

### 4. Run Samples

Please see detail in each folder.

audio/
 
+ ./audio_record
+ ./audio_playback

camera/
+ export XDG_RUNTIME_DIR=/run/user/root
+ ./ispcam_display 1
+ ./usbcam_display 

encode_decode/

+ ./ispcam_encode_mp4 1 file.mp4
+ ./mp4_decode_playback file.mp4

streaming/
+ export XDG_RUNTIME_DIR=/run/user/root
+ connect wifi first
+ ./tcp_server
+ RTSP instruction

transform/
+ export XDG_RUNTIME_DIR=/run/user/roo
+ ./transform_display 0

## License
This is licensed under the BSD 3-Clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.
