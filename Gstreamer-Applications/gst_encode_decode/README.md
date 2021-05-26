
# Encode and Deocde

These samples show how to encode camera video into a MP4 file and playback from the file. 

There are 3 samples. One is encoding from ISP camera (main onboard camera). One is encoding from USB camera. The last one is playback of MP4 file.

## Install Tool

To display USB camera, v4l2-ctl is a useful tool to list the device files of the camera. It can be installed from v4l-utils pacakge.

```bash
$ adb shell
$ apt install v4l-utils
```

## Encode from ISP Camera

```ispcam_encode_mp4``` encodes ISP camera liveview to a mp4 file.

Usage: ispcam_encode_mp4 [camera id: 0|1|2|3] [mp4 filename]

### Start Encoding Camera 0 to MP4:
```bash
$ cd /data/sample-apps-for-Qualcomm-Robotics-RB5-platform/Gstreamer-Applications/gst_encode_decode
$ ./ispcam_encode_mp4 0 isp_record.mp4
```

### Stop:

+ Press Ctrl-C to stop encoding

## Encode from USB Camera

```usbcam_encode_mp4``` encodes USB camera liveview to a mp4 file.

Usage: usbcam_encode_mp4 [/dev/videoX] [mp4 filename]

### Start Encoding /dev/video0 to MP4:
```bash
$ cd /data/sample-apps-for-Qualcomm-Robotics-RB5-platform/Gstreamer-Applications/gst_encode_decode
$ ./usbcam_encode_mp4 /dev/video0 usb_record.mp4
```

### Stop Automatically:

+ The encoding stops in 10 seconds automatically

### Stop Manually:

+ Press Ctrl-C to stop encoding

### No Encode Problem:

It depends on correct setting of resolution and framerate for the camera. If the encoding does not work for the camera, try to apply lower resolution and framerate for the camera.

Run ```usbcam_display``` first to get display work then try encoding with the same setting.

```C
/* usbcam_encode_mp4.c */

/* we set the filter parameter for videorate */
g_object_set(G_OBJECT(framefilter), "caps", 
        gst_caps_from_string(
            "video/x-raw,framerate=30/1,width=1280,height=720"), 
            NULL);
```

## Decode MP4 Playback to Display

```mp4_decode_playback``` decodes a mp4 file to weston display.

Usage: mp4_decode_playback [mp4 filename]

### Start Playback:

+ Connect the board to screen through HDMI output
+ Connect keyboard and mouse to the board
+ Login on the screen
+ Turn on weston display with ```weston.sh```
``` bash
$ /data/sample-apps-for-Qualcomm-Robotics-RB5-platform/Gstreamer-Applications/weston.sh
```
+ Click left-top button on weston display to open weston-terminal
+ Run ```mp4_decode_playback``` in weston-terminal to playback a mp4 file
```bash
$ cd /data/sample-apps-for-Qualcomm-Robotics-RB5-platform/Gstreamer-Applications/dst_encode_decode
$ ./mp4_decode_playback ./isp_record.mp4
```

### Stop Automatically:

+ The playback stops when getting the end of file

### Stop Manually:

+ Move mouse to weston-terminal and click it to activate it
+ Press Ctrl-C to stop display

## License
This is licensed under the BSD 3-Clause-Clear “New” or “Revised” License. Check out the [LICENSE](../LICENSE) for more details.
