# Video Streaming

These samples show two different ways to stream video on network.

The first sample creates a TCP streaming server with gstreamer.

The secode sample demonstrates how to run a RTSP server on the board.


## TCP Streaming Server

```tcp_server``` streams video on network through TCP.

Usage: tcp_server [camera id: 0|1|2|3] IP port

 IP: [x.x.x.x] : IP address of the board. It maybe through eht0 or wlan0.

### Start Streaming:

Example of streaming video from camera 0 on port 5000, IP of the board is 192.168.3.5:
``` bash
$cd /data/Gstreamer-Applications/gst_streaming
$ ./tcp_server 0 192.168.3.5 5000
```

#### Note:

+ The board may have different network interfaces enabled, e.g. eth0 and wlan0. Therefore, please use ```ifconfig``` to get IP address of the board and specify the IP address as a parameter manually. 

### Client Player:

In client computer:

#### Command Line Example:

```bash
vlc tcp://192.168.3.5:5000/
```

#### VLC Application

+ File -> Open Network Stream
+ Input URL: ```tcp://192.168.3.5:5000/``` for example.
+ Click 'Play'

## RTSP Server

### Download

Download https://gstreamer.freedesktop.org/src/gst-rtsp-server/gst-rtsp-server-1.4.5.tar.xz

```bash
$ adb shell
$ cd /data
$ wget https://gstreamer.freedesktop.org/src/gst-rtsp-server/gst-rtsp-server-1.4.5.tar.xz
$ tar xf gst-rtsp-server-1.4.5.tar.xz
```

### Compile

```bash
$ adb shell
$ cd /data/gst-rtsp-server-1.4.5
$ ./autogen.sh --disable-gtk-doc
$ make -j4
```

### Start RTSP Server

Stream camera video on RTSP:

```bash
$ adb shell
$ cd /data/gst-rtsp-server-1.4.5/examples
$ ./test-launch "(qtiqmmfsrc name=qmmf camera=0 ! video/x-h264,format=NV12,width=1920,height=1080,framerate=30/1 ! h264parse ! rtph264pay name=pay0 pt=96 )"
```

### Client Player:

In client computer:

#### Command Line Example:

```bash
vlc rtsp://<ip>:8554/test
```

#### VLC Application

+ File -> Open Network Stream
+ Input URL: ```rtsp://<ip>:8554/test```
+ Click 'Play'

#### Note:

+ Please use ```ifconfig``` on the board to get IP address of the board.

## License
This is licensed under the BSD 3-Clause-Clear “New” or “Revised” License. Check out the [LICENSE](../LICENSE) for more details.
