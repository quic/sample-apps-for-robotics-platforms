# Camera Liveview Display

These samples show camera liveview on display. 

There are two samples. One is display from ISP camera (main onboard camera). The other is display from USB camera.


## Display from ISP Camera

```ispcam_display``` displays ISP camera liveview on display.

Usage: ispcam_display [camera id: 0|1|2|3]

### Start Camera Display:

$ adb disable-verity
$ adb reboot
$ adb wait-for-device root
### The above three steps only need to be operated once and will always be valid.

+ Connect the board to screen through HDMI output
+ Run ```ispcam_display``` to display camera 0
```bash
$ cd /data/gstreamer-applications/camera
$ export XDG_RUNTIME_DIR=/run/user/root
$ ./ispcam_display 0
```
### Stop Camera Display:

+ Press Ctrl-C to stop display
  
## Display from USB Camera

```usbcam_display``` displays USB camera liveview on display.

Usage: usbcam_display </dev/videoX>


### Start Camera Display:

+ Connect the board to screen through HDMI output

+ Run ```usbcam_display``` to display camera /dev/video0
```bash
$ cd /data/gstreamer-applications/camera
$ export XDG_RUNTIME_DIR=/run/user/root
$ ./usbcam_display /dev/video2
```
### Stop Camera Display:

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
