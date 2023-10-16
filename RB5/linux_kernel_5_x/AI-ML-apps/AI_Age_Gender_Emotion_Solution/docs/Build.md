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

# Executing application 
## 4.1 Building the application

```console
adb shell
cd sample-apps-for-robotics-platforms/RB5/linux_kernel_5_x/AI-ML-apps/AI-Age_Gender_Emotion-Solutions/
mkdir build 
cd build
cmake -DSNPE_SDK_BASE_DIR=<SDK Directory Path>/snpe-1.68.0.3932 ..
make -j8
```
## 4.2 Running the application

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
