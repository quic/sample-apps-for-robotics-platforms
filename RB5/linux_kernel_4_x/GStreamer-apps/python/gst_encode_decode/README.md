# Encode and Deocde

These samples show how to encode camera video into a MP4 file and playback from the file.

There are 3 samples. One is encoding from ISP camera (main onboard camera). One is encoding from USB camera. The last one is playback of MP4 file.

## Install Tool

To display USB camera, v4l2-ctl is a useful tool to list the device files of the camera. It can be installed from v4l-utils pacakge.

```
$ adb shell
$ apt install v4l-utils
```

##  Encode from ISP Camera

`ispcam_encode_mpeg`  encodes ISP camera liveview to a mp4 file.

Usage: ispcam_encode_mpeg [camera id: 0|1|2|3] [mp4 filename]

###  Start Encoding Camera 0 to MP4:
```
$ cd <path to directory in Git repository>/gst_encode_decode
$ python3 ispcam_encode_mpeg.py 0 isp_encode_mpeg.mp4
```

### Stop:

-   Press Ctrl-C to stop encoding

## Encode from USB Camera

`usbcam_encode_mpeg`  encodes USB camera liveview to a mp4 file.

Usage: usbcam_encode_mp4 [/dev/videoX] [mp4 filename]

### Start Encoding /dev/video0 to MP4:
```
$ cd <path to directory in Git repository>/gst_encode_decode
$ python3 usbcam_encode_mpeg.py /dev/video0 usb_record.mp4
```
### Stop Manually:

-   Press Ctrl-C to stop encoding

### No Encode Problem:

It depends on correct setting of resolution and framerate for the camera. If the encoding does not work for the camera, try to apply lower resolution and framerate for the camera.

Run  `usbcam_display`  first to get display work then try encoding with the same setting.
```C
# usbcam_encode_mpeg.py

/* we set the filter parameter for videorate */
framefilter.set_property("caps", Gst.caps_from_string(

"video/x-raw,framerate=30/1,width=1280,height=720"))
```

## Decode MP4 Playback to Display

`mp4_decode_playback`  decodes a mp4 file to weston display.

Usage: mp4_decode_playback [mp4 filename]

### Start Playback:

-   Connect the board to screen through HDMI output
-   Turn on weston display with  `weston.sh`
	```
	<path to directory in Git repository>/weston.sh
	```
-   Run  `mp4_decode_playback`  in RB5-terminal to playback a mp4 file

	```
	$ cd <path to directory in Git repository>/gst_encode_decode
	$ export XDG_RUNTIME_DIR=/dev/socket/weston
	$ mp4_decode_playback sample_video.mp4
	```
### Stop Manually:

-   Press Ctrl-C on RB5-terminal to stop display