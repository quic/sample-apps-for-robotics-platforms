
# Video Streaming
This application is used for streaming video on the network. This sample creates a TCP streaming server with gstreamer. tcp_server streams video on the network through tcp.

## Start Streaming

Steps for running tcp_server steaming application:

### TCP Streaming Server:
-   Go to the downloaded directory on RB5 /gst-python-samples/Gstreamer-Applications/gst_streaming
```bash
$ cd /gst-python-samples/Gstreamer-Applications/gst_streaming
```    
-   Example of streaming video from camera 0, on port 5 and ip address of the board: 192.168.3.5
    
```bash
$ python3 tcp_server.py 0 5000 192.168.3.5
```

### Client player:

-   Command line example:
```bash
vlc tcp://192.168.3.5:5000/
