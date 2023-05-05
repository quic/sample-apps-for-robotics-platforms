# Video Transform

This sample shows how to transform ISP camera video to weston display.

## Run Sample

```transform_video``` transforms ISP camera video to weston display.

Usage: transform_display [camera id: 0|1|2|3] [Operation]

Operation:

    flip v|h (v:vertical, h:horizontal)
    rotate 1|2|3  (1:90CW, 2:90CCW, 3:180)
    crop x y width height

Apply one operation at a time.

### Start Transform Display:

+ Connect the board to screen through HDMI output
+ Run ```transform_display``` to transform camera video

Example to transform video from camera 0:
``` bash
$ adb disable-verity
$ adb reboot
$ adb wait-for-device root
### The above three steps only need to be operated once and will always be valid.

$ cd /data/gstreamer-applications/transform
$ export XDG_RUNTIME_DIR=/run/user/root
$ ./transform_display 0 flip v
$ ./transform_display 0 flip h
$ ./transform_display 0 rotate 2
$ ./transform_display 0 crop 10 10 200 100
```

### Stop Transform Display:

+ Press Ctrl-C to stop display

## License
This is licensed under the BSD 3-Clause-Clear “New” or “Revised” License. Check out the [LICENSE](../LICENSE) for more details.
