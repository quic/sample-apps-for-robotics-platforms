#!/bin/bash

adb shell apt-get install git
adb shell pip3 install git+https://github.com/jackersson/gstreamer-python.git

source make_package.sh

adb push $DEB_NAME .
adb shell dpkg -i $DEB_NAME

adb shell "apt-get install cmake"
adb shell "cd /data/rb5-scs/libsee_sensor/ && ./prepare-target.sh rb5"
adb shell "cd /data/rb5-scs/libsee_sensor/ && ./build.sh"

adb shell gcc -o /usr/local/bin/tdk-thermistor-app /usr/local/bin/tdk-thermistor-app.c -lm -Wno-format
adb shell gcc -o /usr/local/bin/read_magnetometer /usr/local/bin/read_magnetometer.c -lm -Wno-format -Wno-implicit-function-declaration

adb shell rm $DEB_NAME

