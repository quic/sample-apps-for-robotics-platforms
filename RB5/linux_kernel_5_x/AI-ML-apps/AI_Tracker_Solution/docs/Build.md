# How to Configure, build and run the application

## 1. Clone the repository

```console
adb shell
cd /home/
https://github.com/quic/sample-apps-for-robotics-platforms.git
cd sample-apps-for-robotics-platforms/RB5/linux_kernel_5_x/AI-ML-apps/AI_Tracker_Solution/
```

## 2. Update application configuration

This application include multiple solution like People Intrusion detection, Helmet Detection, Fire detection and crack detection. Configuration of all the solutions are described in data/config.json file. This configuration file should be updated to select the desired solution, model configuration and input/output stream.
Application can take rtsp/camera stream as input and it can dump the output to mp4 or hdmi monitor.

Table 1-1 show all the configuration items:

**Input Configuration**

| key                      | Value  | Description                                    |
| ------------------------ | -----  | ---------------------------------------------- |
| input-config-name        | string | Name of the input config                    |
| stream-type              | string | Input stream type camera or rtsp               |
| stream-width             | int    | Width of the input stream                      |
| stream-height            | int    | Height of the input stream                     |
| SkipFrame                | int    | model input height                             |
| camera-url               | string | rtsp stream path if the input stream is rtsp   |

**Model Configuration**

| key                      | Value  | Description                                    |
| ------------------------ | -----  | ---------------------------------------------- |
| model-name               | string | Name of the model                              |
| model-path               | string | Path of the dlc file                           |
| label-path               | string | Path of the label file                         |
| runtime                  | string | SNPE Runtime (GPU, CPU, DSP)                   |
| nms-threshold            | float  | NMS threshold                                  |
| conf-threshold           | float  | Confidence threshold                           |
| labels                   | int    | Number of labels + 5                           |
| input-layers             | string | Name of the input layers                       |
| output-layers            | string | Name of the output layers                      |
| output-tensors           | string | Name of the output tensors                     |

**Solution Configuration**

| key                      | Value  | Description                                    |
| ------------------------ | -----  | ---------------------------------------------- |
| solution-name            | string | Name of the Solution                           |
| model-name               | string | Name of the model configuration to be used     |
| input-config-name        | string | Name of the Input configuration to be used  |
| Enable                   | bool   | 1 to Enable and 0 to Disable the solution      |
| output-type              | string | Filesink to save the output in mp4             |
|                          |        | Wayland if display the output on hdmi monitor  |
| output-path              | string | Path of the output, Enabled if output type     |
|                          |        | is filesink                                    |

### Example 1: Configuration for taking the input stream from camera and output on hdmi monitor
```console
{
    "input-configs":[
        {
            "input-config-name":"camera",
            "stream-type":"camera",
            "stream-width":1280,
            "stream-height":720,
            "SkipFrame":1,
            "fps-n":30,
            "fps-d":1
        },
 ],
     "model-configs":[
        {
            "model-name":"ByteTrack",
            "model-path":"../models/bytetrack_s_quant.dlc",
            "label-path":"../data/coco_label.txt",
            "runtime":"DSP",
            "labels":85,
            "grids":25200,
            "nms-threshold":0.5,
            "conf-threshold":0.5,
            "input-layers":[
                "images"
            ],
            "output-layers":[
            "/head/Transpose"
            ],
            "output-tensors":[
                "output"
            ],
            "global-threshold":0.2
        },
 ],
     "solution-configs":[
        {
            "solution-name":"People-Tracking-1",
            "model-name":"ByteTrack",
            "input-config-name":"camera",
            "Enable":1,
            "output-type":"wayland",
        },
        {
            "solution-name":"People-Tracking-2",
            "model-name":"ByteTrack",
            "input-config-name":"rtsp4",
            "Enable":0,
            "output-type":"filesink",
            "output-path":"/root/tracking.mp4"
        }
 ]
 }
 
```

## 3. Model Integration
Push model into the model directory in the application and update the config.json file.

Update the output-layers and output tensors. 
To check the output-layers and output-tensors nodes, Open the model in the Netron app and click on output layer as mentioned in the image.

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/d05f0098-0b2c-4974-be9a-7abfc6bf6fd3)


```console
  "model-configs":[
      {
          "model-name":"ByteTrack", --> Add model name here. It should match with the model name in solution config
          "model-type":"bytetrack", --> It should be yolov5 for YoloV5 model.
          "model-path":"../models/bytetrack_s_quant1.dlc", --> Path of the quantized model
          "label-path":"../data/label.txt", --> Path to the label file
          "runtime":"DSP", 
          "labels":6, --> Update label here.
          "grids":25200,
          "nms-threshold":0.5,
          "conf-threshold":0.4,
          "input-layers":[
              "images" --> Open the model in netron.app and get the input-layers names.
          ],
          "output-layers":[ --> Refer the steps given above to know the output-layers and output-tensors
              "/head/Transpose",
          ],
          "output-tensors":[
              "output",
          ],
          "global-threshold":0.2
      },
```

## 4. Steps to download and deploy Qualcomm® Neural Processing SDK libraries on target device

Download SDK from https://developer.qualcomm.com/software/qualcomm-neural-processing-sdk/tools-archive.

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

**Verify SDK version**

```console
adb shell
chmod +x /usr/bin/snpe-net-run
snpe-net-run --version
```

## 5. Steps to execute the application

### 5.1 Installing OpenCV 4.5.5

Download OpenCV 4.5.5 source code from
https://codeload.github.com/opencv/opencv/tar.gz/refs/tags/4.5.5

```console
adb shell
wget https://codeload.github.com/opencv/opencv/tar.gz/refs/tags/4.5.5 -O opencv-4.5.5.tar.gz
tar  -zxvf opencv-4.5.5.tar.gz
cd ./opencv-4.5.5
```

Install dependencies

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
apt install python3.6-dev
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

### 5.2 Building the application

```console
adb shell
cd /home/sample-apps-for-robotics-platforms/RB5/linux_kernel_5_x/AI-ML-apps/AI_Tracker_Solution/
mkdir build 
cd build
cmake -DSNPE_SDK_BASE_DIR=<SDK Directory Path>/snpe-1.68.0.3932 ..
make
```
### 5.3 Running the application

To display the output on monitor. Please connect the monitor to the device via HDMI cable. Follow below instructions to enable weston:
```console
export XDG_RUNTIME_DIR="/run/user/root"
```
```console
cd build
./out/main -c ../data/config.json
```

## Verify results

Check if the directory of "output-path" if output-type is filesink.
Or please check the monitor connected with the HDMI for the output.
