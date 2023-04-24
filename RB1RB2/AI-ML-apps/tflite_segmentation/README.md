## 1. Set up the environment
Install RB1/2 LE SDK and source the environment
Download sample code on PC

## 2. Modify & Compile the tflite demo app
```
Gstreamer App demo provides two methods, it can
be used either Gstreamer command line via
gst_parse_launch or GST API to build pipeline.
It sets to API method by default. Switch to command line methods:
$ vi tflite_segmentation.c

+#define PARSER

To Compile:
$ cd tflite_segmentation/
$ make
```

## 3. Push resources to the device
```
$ adb push tflite_segmentation /data
$ adb push dv3_argmax_int32.tflite /data
$ adb push dv3-argmax.labels /data
```

## 4. Run the demo app
```
$ adb shell
$ export XDG_RUNTIME_DIR=/run/user/root
$ ./data/tflite_segmentation
```

## License
This is licensed under the BSD 3-clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.
