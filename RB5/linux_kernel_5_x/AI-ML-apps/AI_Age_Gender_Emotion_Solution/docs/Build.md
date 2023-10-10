# How to Configure, build and run the application

# 1 Clone the repository

```console
adb shell
cd /home/
git clone https://github.com/quic/sample-apps-for-robotics-platforms.git
cd sample-apps-for-robotics-platforms/RB5/linux_kernel_5_x/AI-ML-apps/AI-Age_Gender_Emotion-Solutions/
```

# 2 Update application configuration

Configuration of all the solutions is described in data/config.json file. This configuration file should be updated to select the desired solution, model configuration and input/output stream.
Application can take rtsp/camera stream as input, and it can dump the output to mp4 or hdmi monitor.

Table 1-1 show all the configuration items:

**Input Configuration**

| key                      | Value  | Description                                    |
| ------------------------ | -----  | ---------------------------------------------- |
| input-config-name        | string | Name of the input config                       |
| stream-type              | string | Input stream type camera or rtsp               |
| stream-width             | int    | Width of the input stream                      |
| stream-height            | int    | Height of the input stream                     |
| SkipFrame                | int    | Number of frames to skip                       |
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
| labels                   | int    | Number of labels                               |
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

## Example 1: Configuration for taking the input stream from camera and output on hdmi monitor
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
            "model-name":"face-detect",
            "model-type":"centerface",
            "model-path":"../models/centerface_quantized.dlc",
            "runtime":"DSP",
            "nms-threshold":0.3,
            "conf-threshold":0.5,
            "grids":25200,
            "input-layers":[
                "input.1"
            ],
            "output-layers":[
                "Neuron_42",
                "Conv2d_40",
                "Conv2d_41",
                "Conv2d_42"
            ],
            "output-tensors":[
                "537",
                "538",
                "539",
                "540"
            ],
            "global-threshold":0.2
        },

        {
            "model-name":"age",
            "model-type":"googlenet",
            "model-path":"../models/age_caffe_quant.dlc",
            "runtime":"DSP",
            "nms-threshold":0.5,
            "conf-threshold":0.5,
            "grids":25200,
            "input-layers":[
                "data"
            ],
            "output-layers":[
                "prob"
            ],
            "output-tensors":[
                "prob"
            ],
            "global-threshold":0.2
        },
        {
            "model-name":"gender",
            "model-type":"gendernet",
            "model-path":"../models/gender_googlenet_quant.dlc",
            "runtime":"DSP",
            "nms-threshold":0.5,
            "conf-threshold":0.5,
            "grids":25200,
            "input-layers":[
                "input"
            ],
            "output-layers":[
                "loss3/loss3"
            ],
            "output-tensors":[
                "loss3/loss3_Y"
            ],
            "global-threshold":0.2
        },
        {
            "model-name":"emotion",
            "model-type":"FERplus",
            "model-path":"../models/Emotion1.dlc",
            "runtime":"DSP",
            "nms-threshold":0.5,
            "conf-threshold":0.5,
            "grids":25200,
            "input-layers":[
                "input.1"
            ],
            "output-layers":[
                "Gemm_26"
            ],
            "output-tensors":[
                "94"
            ],
            "global-threshold":0.2
        }

    ],
    "solution-configs":[
        {
            "solution-name":"face-detection",
            "model-name":["face-detect","age","gender","emotion"],
            "input-config-name":"camera",
            "Enable":1,
            "output-type":"wayland",
            "output-path":"/root/video.mp4"
        }
    ]
}
 
```

## Example 2: Configuration of rtsp input stream and output on the device

```console
"input-configs":[
    {
        "input-config-name":"rtsp3",
        "stream-type":"rtsp",
        "camera-url":"rtsp://10.147.243.253:8554/crack_video.264",
        "SkipFrame":1
    },
        
"model-configs":[
        {
            "model-name":"face-detect",
            "model-type":"centerface",
            "model-path":"../models/centerface_quantized.dlc",
            "runtime":"DSP",
            "nms-threshold":0.3,
            "conf-threshold":0.5,
            "grids":25200,
            "input-layers":[
                "input.1"
            ],
            "output-layers":[
                "Neuron_42",
                "Conv2d_40",
                "Conv2d_41",
                "Conv2d_42"
            ],
            "output-tensors":[
                "537",
                "538",
                "539",
                "540"
            ],
            "global-threshold":0.2
        },

        {
            "model-name":"age",
            "model-type":"googlenet",
            "model-path":"../models/age_caffe_quant.dlc",
            "runtime":"DSP",
            "nms-threshold":0.5,
            "conf-threshold":0.5,
            "grids":25200,
            "input-layers":[
                "data"
            ],
            "output-layers":[
                "prob"
            ],
            "output-tensors":[
                "prob"
            ],
            "global-threshold":0.2
        },
        {
            "model-name":"gender",
            "model-type":"gendernet",
            "model-path":"../models/gender_googlenet_quant.dlc",
            "runtime":"DSP",
            "nms-threshold":0.5,
            "conf-threshold":0.5,
            "grids":25200,
            "input-layers":[
                "input"
            ],
            "output-layers":[
                "loss3/loss3"
            ],
            "output-tensors":[
                "loss3/loss3_Y"
            ],
            "global-threshold":0.2
        },
        {
            "model-name":"emotion",
            "model-type":"FERplus",
            "model-path":"../models/Emotion1.dlc",
            "runtime":"DSP",
            "nms-threshold":0.5,
            "conf-threshold":0.5,
            "grids":25200,
            "input-layers":[
                "input.1"
            ],
            "output-layers":[
                "Gemm_26"
            ],
            "output-tensors":[
                "94"
            ],
            "global-threshold":0.2
        }

    ],
    "solution-configs":[
        {
            "solution-name":"face-detection",
            "model-name":["face-detect","age","gender","emotion"],
            "input-config-name":"camera",
            "Enable":1,
            "output-type":"wayland",
            "output-path":"/root/video.mp4"
        }
    ]

Use model-name and input-config-name to select model and input stream respectively.

```

# 3 Model Integration
Push model into the model directory in the application and update the config.json file.

Update the output-layers and output tensors. 
To check the output-layers and output-tensors nodes, Open the model in the Netron app and click on Conv layer as mentioned in the image.
In centerface.onnx, the output-nodes are onnx::536, 538, 539 and 540.
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/7ee4e9f5-4357-4402-9ede-814c98ed9ea5)

In centerface.dlc the output-layers and output-tensors are 536(Conv2d_39), 538(Conv2d_40), 539(Conv2d_41) and 540(Conv2d_42)
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/723cb29b-2593-47a7-8c78-6a096a89ed75)


```console
  "model-configs":[
      {
          "model-name":"model-name", --> Add model name here. It should match with the model name in solution config
          "model-type":"model type", --> Select type of the model.
          "model-path":"../models/model.dlc", --> Path of the quantized model
          "label-path":"../data/label.txt", --> Path to the label file
          "runtime":"DSP", 
          "labels":85, --> Update label here.
          "grids":25200,
          "nms-threshold":0.5,
          "conf-threshold":0.4,
          "input-layers":[
              "images" --> Open the model in netron.app and get the input-layers names.
          ],
          "output-layers":[ --> Refer the steps given above to know the output-layers and output-tensors
              "Conv_271",
              "Conv_305",
              "Conv_339"
          ],
          "output-tensors":[
              "443",
              "496",
              "549"
          ],
          "global-threshold":0.2
      },
```

# 4 Steps to download and deploy SNPE libraries on target device

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

# 5 Steps to execute the application

## 5.1 Installing OpenCV 4.5.5

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

## 5.2 Building the application

```console
adb shell
cd /home/CE2.0_SI_Age_Gender_Emotion_Solution
mkdir build 
cd build
cmake -DSNPE_SDK_BASE_DIR=<SDK Directory Path>/snpe-1.68.0.3932 ..
make -j8
```
# 5.3 Running the application

To display the output on monitor. Please connect the monitor to the device via HDMI cable. Follow below instructions to enable weston:
```console
export XDG_RUNTIME_DIR=/run/user/root
```
```console
cd build
./out/main -c ../data/config.json
```

# Verify results

Check if the directory of "output-path" if output-type is filesink.
Or please check the monitor connected with the HDMI for the output.
