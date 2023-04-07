## 1. Set up the environment
Install RB1/2 LE SDK and source the environment
Download sample code on PC

## 2. Compile the snpe demo app
```
$ cd snpe_detection/
$ make
```

## 3. Push resources to the device
```
$ adb push snpe_detection /data
$ adb push mobilenet_v1_ssd_2017_quantized.dlc /data
$ adb push ssd-mobilenet.labels /data
```

## 4. Run the demo app
```
$ adb shell
$ export XDG_RUNTIME_DIR=/run/user/root
$ ./data/snpe_detection
```

## License
This is licensed under the BSD 3-clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.
