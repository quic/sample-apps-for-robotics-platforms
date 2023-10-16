# How to setup the environment

# 1 Prerequisite

- One x86 Host Workstation. Operating System Ubuntu 20.04， Python 3.6.9
- Qualcomm Robotics RB5 Development kit: https://developer.qualcomm.com/qualcomm-robotics-rb5-kit

# 2 x86 Host Installation
The model used in the application needs to be trained with the dataset. Then convert the model to dlc to run in the application.

## 2.1  Qualcomm® Neural Processing Software Development Kit (SDK) Installation
Download Software Development Kit (SDK) from https://developer.qualcomm.com/software/qualcomm-neural-processing-sdk/tools-archive

The Qualcomm® Neural Processing Software Development Kit (SDK) provides tools for model conversion(onnx to dlc/caffe to dlc), model quantization and execution. Refer the steps given in the detailed documentation in the SDK for installation.
https://developer.qualcomm.com/sites/default/files/docs/snpe/index.html

# 3. X86 Host RTSP Stream Installation
In this section, show how to prepare test video and setup an live555 RTSP streaming server to simulate an IP camera. The installation steps run in the X86 host.

## 3.1 Prepare test video. 
Prepare a test video, if test video is mp4 ,mkv or other formats. Need to convert to H264 raw video.
The following steps demonstrate how to convert MP4 to H264 raw video.

```console

wget https://test-videos.co.uk/vids/bigbuckbunny/mp4/h264/1080/Big_Buck_Bunny_1080_10s_1MB.mp4

sudo apt install ffmpeg

ffmpeg -i Big_Buck_Bunny_1080_10s_1MB.mp4 -f h264 -vcodec libx264 Big_Buck_Bunny_1080_10s_1MB.264
```

## 3.2 Live555 server installation. 

```console
wget http://www.live555.com/liveMedia/public/live555-latest.tar.gz
tar -zxvf live555-latest.tar.gz
cd live/
./genMakefiles linux-64bit
make -j4
cd ..
```

copy test video to mediaServer folder

```console
cp Big_Buck_Bunny_1080_10s_1MB.264 ./live/mediaServer
cd ./live/mediaServer
./live555MediaServer
```

rtsp://192.168.4.111:8554/<filename> is the rtsp url. 
 - "192.168.4.111" is RTSP server IP address
 - 8554 is the default port
 - <filename> is the video file name under mediaServer folder
 In this case, url "rtsp://192.168.4.111:8554/Big_Buck_Bunny_1080_10s_1MB.264" is the video address. 


## 3.3 Verify live555 server
 Download and install VLC media player on a Windows desktop  https://www.videolan.org/

Launch VLC player, choose "Media->Open Network Stream"
input RTSP url rtsp://192.168.4.111:8554/Big_Buck_Bunny_1080_10s_1MB.264

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/c5cebc6f-df2b-4618-9d61-bcd684b23103)

 Click play to test if live555 server work.<mark>Note: Make sure the network address is reachable </mark>


 # 4 Steps to deploy Qualcomm® Neural Processing SDK libraries on target device

Download Qualcomm® Neural Processing Software Development Kit (SDK) from https://developer.qualcomm.com/software/qualcomm-neural-processing-sdk/tools-archive.

**Windows**

```console
cd snpe-1.68.0\snpe-1.68.0.3932
adb push lib\aarch64-ubuntu-gcc7.5\. /usr/lib/
adb push lib\aarch64-ubuntu-gcc7.5\libsnpe_dsp_domains_v2.so /usr/lib/rfsa/adsp/
adb push lib\dsp\. /usr/lib/rfsa/adsp/
adb push bin\aarch64-ubuntu-gcc7.5\snpe-net-run /usr/bin/
```

**Linux**

```console
cd snpe-1.68.0/snpe-1.68.0.3932/
adb push lib/aarch64-ubuntu-gcc7.5/* /usr/lib/
adb push lib/aarch64-ubuntu-gcc7.5/libsnpe_dsp_domains_v2.so /usr/lib/rfsa/adsp/
adb push lib/dsp/* /usr/lib/rfsa/adsp/
adb push bin/aarch64-ubuntu-gcc7.5/snpe-net-run /usr/bin/
```

**Verify SNPE version**

```console
adb shell
chmod +x /usr/bin/snpe-net-run
snpe-net-run --version
```

## 5 Installing OpenCV 4.5.5

Download OpenCV 4.5.5 source code from
https://codeload.github.com/opencv/opencv/tar.gz/refs/tags/4.5.5

```console
adb shell
wget https://codeload.github.com/opencv/opencv/tar.gz/refs/tags/4.5.5 -O opencv-4.5.5.tar.gz
tar  -zxvf opencv-4.5.5.tar.gz
cd ./opencv-4.5.5
```

### Install dependencies

```console
apt install build-essential cmake unzip git pkg-config
apt install libjpeg-dev libpng-dev libtiff-dev
apt-get install libjsoncpp-dev libjson-glib-dev libgflags-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
apt install libjasper-dev
apt-get install libeigen3-dev
```
if you receive an error about libjasper-dev being missing then follow the following instructions:
```console
wget http://ports.ubuntu.com/ubuntu-ports/pool/main/j/jasper/libjasper-dev_1.900.1-debian1-2.4ubuntu1.3_arm64.deb
dpkg -i libjasper-dev_1.900.1-debian1-2.4ubuntu1.3_arm64.deb

wget http://ports.ubuntu.com/ubuntu-ports/pool/main/j/jasper/libjasper1_1.900.1-debian1-2.4ubuntu1.3_arm64.deb
dpkg -i libjasper1_1.900.1-debian1-2.4ubuntu1.3_arm64.deb
```

Otherwise (or once libjasper-dev is installed), keep going.
```console
apt install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
apt install libxvidcore-dev libx264-dev
```

OpenCV’s highgui module relies on the GTK library for GUI operations. Install GTK command:
```console
apt install libgtk-3-dev
```
install Python 3 headers and libraries
```console
apt install libatlas-base-dev gfortran
```
Build and install
```console

mkdir build && cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local/opencv4.5 \
      -D OPENCV_ENABLE_NONFREE=ON \
      -D OPENCV_GENERATE_PKGCONFIG=YES \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -D BUILD_EXAMPLES=OFF \
      -D INSTALL_PYTHON_EXAMPLES=OFF \
      ..
make -j8
make install      

```
