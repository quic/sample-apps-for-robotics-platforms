# Camera Liveview Display

These samples show camera liveview on weston display. 

There are two samples. One is display from ISP camera (main onboard camera). The other is display from USB camera.

## Install Tool

To display USB camera, v4l2-ctl is a useful tool to list the device files of the camera. It can be installed from v4l-utils pacakge.

```bash
$ adb shell
$ apt install v4l-utils
```

## Display from ISP Camera

```ispcam_display``` displays ISP camera liveview on weston display.

Usage: ispcam_display [camera id: 0|1|2|3]

### Start Camera Display:

+ Connect the board to screen through HDMI output
+ Connect keyboard and mouse to the board
+ Login on the screen
+ Turn on weston display with ```weston.sh```
``` bash
$ /data/sample-apps-for-Qualcomm-Robotics-RB5-platform/Gstreamer-Applications/weston.sh
```
+ Click left-top button on weston display to open weston-terminal
+ Run ```ispcam_display``` in weston-terminal to display camera 0
```bash
$ cd /data/sample-apps-for-Qualcomm-Robotics-RB5-platform/Gstreamer-Applications/gst_camera
$ ./ispcam_display 0
```

![Image text](image/ispCamera_display_weston.png)

### Stop Camera Display:

+ Move mouse to weston-terminal and click it to activate it
+ Press Ctrl-C to stop display
  
## Display from USB Camera

```usbcam_display``` displays USB camera liveview on weston display.

Usage: usbcam_display </dev/videoX>

### Find device file of the camera:

+ Use **v4l2-ctl --list-devices** to list device files of all camera
+ Find the device file of the USB camera. Not all video device files can be displayed
```bash
$ v4l2-ctl --list-devices
<Some camera information> (<some usb infomation>):
	/dev/video0
	/dev/video1
```

### Start Camera Display:

+ Connect the board to screen through HDMI output
+ Connect keyboard and mouse to the board
+ Login on the screen
+ Turn on weston display with ```weston.sh```
``` bash
$ /data/sample-apps-for-Qualcomm-Robotics-RB5-platform/Gstreamer-Applications/weston.sh
```
+ Click left-top button on weston display to open weston-terminal
+ Run ```usbcam_display``` in weston-terminal to display camera /dev/video0
```bash
$ cd /data/sample-apps-for-Qualcomm-Robotics-RB5-platform/Gstreamer-Applications/gst_camera
$ ./usbcam_display /dev/video0
```

![Image text](image/usbCamera_display_weston.png)

### Stop Camera Display:

+ Move mouse to weston-terminal and click it to activate it
+ Press Ctrl-C to stop display

### No Display Problem:

A successful display depends on correct setting of resolution and framerate for the camera. If the display not shown for the camera, try to apply lower resolution and framerate for the camera.

```C
/* usbcam_display.c */

/* we set the filter parameter for videorate */
g_object_set(G_OBJECT(framefilter), "caps", 
        gst_caps_from_string(
            "video/x-raw,framerate=30/1,width=1280,height=720"), 
            NULL);
```

## License
This is licensed under the BSD 3-Clause-Clear “New” or “Revised” License. Check out the [LICENSE](../LICENSE) for more details.
