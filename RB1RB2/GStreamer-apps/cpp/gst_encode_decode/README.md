
# Encode and Decode

These samples show how to encode camera video into a MP4 file and playback from the file. 

There are 2 samples. One is encoding from ISP camera (main onboard camera). The other is playback of MP4 file.


## Encode from ISP Camera

```ispcam_encode_mp4``` encodes ISP camera liveview to a mp4 file.

Usage: ispcam_encode_mp4 [camera id: 0|1|2|3] [mp4 filename]

### Start Encoding Camera 0 to MP4:
```bash
$ adb disable-verity
$ adb reboot
$ adb wait-for-device root
### The above three steps only need to be operated once and will always be valid.

$ cd /data/gstreamer-applications/encode_decode
$ export XDG_RUNTIME_DIR=/run/user/root
$ ./ispcam_encode_mp4 0 isp_record.mp4
```

### Stop:

+ Press Ctrl-C to stop encoding

### No Encode Problem:

It depends on correct setting of resolution and framerate for the camera. If the encoding does not work for the camera, try to apply lower resolution and framerate for the camera.

Run ```ispcam_display``` first to get display work then try encoding with the same setting.

```
/* ispcam_encode_mp4.c */

/* we set the filter parameter for videorate */
g_object_set(G_OBJECT(framefilter), "caps", 
        gst_caps_from_string(
            "video/x-raw,framerate=30/1,width=1920,height=1080"), 
            NULL);
```

## Decode MP4 Playback to Display

```mp4_decode_playback``` decodes a mp4 file to display.

Usage: mp4_decode_playback [mp4 filename]

### Start Playback:

+ Connect the board to screen through HDMI output

```bash
$ cd /data/gstreamer-applications/encode_decode
$ ./mp4_decode_playback isp_record.mp4
```

### Stop Automatically:

+ The playback stops when getting the end of file

### Stop Manually:

+ Press Ctrl-C to stop display

## License
This is licensed under the BSD 3-Clause-Clear “New” or “Revised” License. Check out the [LICENSE](../LICENSE) for more details.
