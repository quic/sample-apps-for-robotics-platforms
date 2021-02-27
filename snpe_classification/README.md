# RB5 SNPE_Classification

## 1. Init and set up the environment
1. Download the source code
You may need to first download the mobilenet model from tflite official website and then convert it to '.dlc' using the snpe conversion tool from https://developer.qualcomm.com/docs/snpe/model_conv_tensorflow.html. (The output, 'tensorflow_mobilenet.dlc' will be used for testing in the 'install.sh'
```
$ adb shell
$ cd /data
$ git clone https://github.qualcomm.com/Robotics/sample-apps-for-Qualcomm-Robotics-RB5-platform.git
```
2. Run the script: install.sh
```
$ cd snpe_classification
$ sh install.sh
```
3. Start the weston display server:
```
$ source /data/weston.sh &
```
4. Set up weston path if it is not already set by weston.sh:
```
$ export XDG_RUNTIME_DIR=/usr/bin/weston_socket
```
## 2. Compile the snpe demo app
```
$ make
```
## 3. Run the built demo app:
```
$ ./snpe_classification
```
## License
This is licensed under the BSD 3-clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.
