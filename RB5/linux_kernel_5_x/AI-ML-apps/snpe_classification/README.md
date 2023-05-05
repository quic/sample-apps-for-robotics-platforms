## 1. Set up the environment
Install RB5 LU SDK and source the environment
Download sample code on PC

## 2. Modify & Compile the snpe demo app
```
Gstreamer App demo provides two methods, it can
be used either Gstreamer command line via
gst_parse_launch or GST API to build pipeline.
It sets to API method by default. Switch to command line methods:
$ vi snpe_classification.c

+#define PARSER

To Compile:
$ cd snpe_classification
$ make
```

## 3. Push resources to the device
```
$ adb push snpe_classification /data
$ adb push mobilenet_v1_quantaware_quantized.dlc /data
$ adb push mobilenet.labels /data/mobilenet2.labels
```

## 4. Run the demo app
```
$ adb shell
$ export XDG_RUNTIME_DIR=/run/user/root
$ ./data/snpe_classification
```

## License
This is licensed under the BSD 3-clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.
