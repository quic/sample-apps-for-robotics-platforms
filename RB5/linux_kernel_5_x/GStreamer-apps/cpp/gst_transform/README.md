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
$cd <path to directory in Git repository>/gst_transform
$ ./transform_display 0 flip v
$ ./transform_display 0 flip h
$ ./transform_display 0 rotate 2
$ ./transform_display 0 crop 10 10 200 100
```

The output screenshots of examples in the follow:
![Image text](image/gst-transform-flip-v.png)
![Image text](image/gst-transform-flip-h.png)
![Image text](image/gst-transform-flip-rotate2.png)
![Image text](image/gst-transform-flip-crop.png)


### Stop Transform Display:

+ Press Ctrl-C to stop display

## License
This is licensed under the BSD 3-Clause-Clear “New” or “Revised” License. Check out the [LICENSE](../LICENSE) for more details.
