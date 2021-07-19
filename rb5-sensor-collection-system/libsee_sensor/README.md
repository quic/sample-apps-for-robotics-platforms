# libsee_sensor

Library to access sensors via the Qualcomm Sensors Execution Environment (SEE) framework

## Build instructions

### Prepare host environment

```bash
$ ./prepare-host.sh
```

### Copy everything to the target

Suggested location: /home/development/libsee_sensor

```bash
$ cd ..
$ adb push libsee_sensor/ /home/development
$ cd libsee_sensor
```

### On target setup

This only needs to be done once

```bash
# cd /home/development/libsee_sensor
# ./prepare-target.sh <rb5, m0051>
```

### Build and install on target

```bash
# ./build.sh
```
