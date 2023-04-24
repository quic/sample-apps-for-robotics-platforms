# Camera HAL3 Sample Code

## Get Started

1. Install RB5 LU SDK and source the environment
2. Download sample code to PC

### Compile
```
$ cd camera-hal3-sample/src
$ make
$ adb push camera_hal3 /data
```

## Run Sample

### Start Sample

```camera_hal3 ``` is the sample program.

There are 2 streams created by the sample program: preview and snapshot.

The resolution of each stream must be set seperately.

Usage: ./camera_hal3 [camera id] [preview width] [preview height] [snapshot width] [snapshot height]

For example:
```bash
$ adb disable-verity
$ adb reboot
$ adb wait-for-device root
### The above three steps only need to be operated once and will always be valid.

$ cd /data/camera-hal3-sample/src
$ ./camera_hal3 0 1920 1080 1920 1080
Camera HAL library loaded.
Initialize cameras...
Cameras list:
 camera 0
 camera 1
 camera 2
 camera 3
open Camera: 0
Press h for help.
>
```
There are some functions demonstrating camera functionality.

+ save preview frame (yuv 420 8bit)
+ take a snapshot (jpeg)
+ snapshot rotation
    + 0 degree
    + 90 degree
    + 180 degree
    + 270 degree
+ exposure
    + auto exposure
    + manual exposure time in Ms
    + manual exposure ISO
+ white balance
    + auto white balance
    + color temperature
    + rgb gain
+ antibanding
    + off
    + 50Hz
    + 60Hz
    + auto
+ color correction
    + matrix
    + fast
    + high quality


### Function List  

Input 'h' for function list.

``` bash
> h
Press p: save preview frame.
Press s: take a snapshot.
Press r: set snapshot rotation.
Press e: set exposure.
Press w: set white balance.
Press a: set antibanding.
Press c: set color correction.
Press q: quit.
```

### Save Preview Frame

Input 'p' to save a preview frame. Saved file will be in the folder CAPTURE/.

```bash
> p
Save preview frame 35 to:
 CAPTURE/preview_420_888_1920x1080_20200101_112233.yuv
```

### Snapshot

Input 's' to take a snapshot. Saved file will be in the folder CAPTURE/.

```bash
> s
Save snapshot frame 40 to:
 CAPTURE/snapshot_20200101_113344.jpg
```

### Snapshot Rotation

Input 'r' to set snapshot rotation.

```bash
> r
Select degree: 0: 0, 1: 90, 2: 180, 3: 270, q: quit
>>
```

### Exposure

Input 'e' to set exposure.

```bash
> e
Press 1: auto, 2: exposure time, 3: ISO, q: quit
>> 
```

Input '1' to enable auto exposure.

Input '2' to set exposure time (Ms).

```bash
Press 1: auto, 2: exposure time, 3: ISO, q: quit
>> 2
exposure time(Ms):
```

Input '3' to set ISO. Before set ISO value, exposure time must be set first.

```bash
> e
Press 1: auto, 2: exposure time, 3: ISO, q: quit
>> 3
exposure time(Ms): 
ISO (100 200 400 800 1600): 
```

### White Balance

Input 'w' to set white balance.

```bash
> w
Press 1: auto, 2: color temperature, 3: gain, q: quit
>> 
```

Input '1' for auto white balance.

Input '2' for color temperature mode.

```bash
Press 1: auto, 2: color temperature, 3: gain, q: quit
>> 2
Color temperature (1: high, 0: mid, -1: low):
>
```

Input '3' for RGB gain.

```bash 
> w
Press 1: auto, 2: color temperature, 3: gain, q: quit
>> 3
Input R G B gain (for ex: 2.0, 1.0, 0.9)
r gain:
g gain:
b gain:
```

### Antibanding

Input 'a' to set antibanding.

```bash
> a
Press 0: off, 1: 50hz, 2: 60hz, 3: auto, q: quit
>>
```

### Color Correction

Input 'c' to set color correction.

```bash
> c
Press 0: Matrix, 1: Fast, 2: Hight Quality, q: quit
```

## License
This is licensed under the BSD 3-Clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.
