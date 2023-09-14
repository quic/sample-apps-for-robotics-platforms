/* 
Copyright (c) 2021 Yifu Zhang
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
Copyright (C) 2019 THL A29 Limited, a Tencent company. All rights reserved.
Licensed under the BSD 3-Clause License (the "License"); you may not use this file except
in compliance with the License. You may obtain a copy of the License at
https://opensource.org/licenses/BSD-3-Clause
Unless required by applicable law or agreed to in writing, software distributed
under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied. See the License for the
specific language governing permissions and limitations under the License. 
*/

#include <math.h>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include "Configuration.h"

#include "ByteTrackSnpe.h"

namespace bytetracksnpe {

ByteTrackSnpe::ByteTrackSnpe() : m_snperuntime(nullptr) {
    std::vector<int> strides(stride_arr, stride_arr + sizeof(stride_arr) / sizeof(stride_arr[0]));
    generate_grids_and_stride(strides);
    app_start =  chrono::system_clock::now();

}

ByteTrackSnpe::~ByteTrackSnpe() {
    DeInitialize();
}

bool ByteTrackSnpe::Initialize(const ObjectDetectionSnpeConfig& config)
{
    m_snperuntime = std::move(std::unique_ptr<snperuntime::SNPERuntime>(new snperuntime::SNPERuntime()));

    m_inputLayers = config.inputLayers;
    m_outputLayers = config.outputLayers;
    m_outputTensors = config.outputTensors;
    m_labels = config.label_count;
    m_grids = config.grids;
    m_nmsThresh = config.nmsThresh;
    m_confThresh = config.confThresh;

    LOG_INFO("Runtime= %d \n", config.runtime);

    m_snperuntime->SetOutputLayers(m_outputLayers);
    if (!m_snperuntime->Initialize(config.model_path, config.runtime)) {
        LOG_ERROR("Can't init snpetask instance.");
        return false;
    }

    m_output = new float[m_grids * m_labels];
    m_isInit = true;
    return true;
}

bool ByteTrackSnpe::DeInitialize()
{
    if (m_isInit) {
        m_snperuntime->Deinitialize();
        m_snperuntime.reset(nullptr);
    }

    if (m_output) {
        delete[] m_output;
        m_output = nullptr;
    }

    m_isInit = false;
    return true;
}

bool ByteTrackSnpe::PreProcessInput(const cv::Mat& input_image)
{
    if (input_image.empty()) {
        LOG_ERROR("Invalid image!");
        return false;
    }

    // Convert BGR Image to RGB Image
    cv::Mat image;
    cv::cvtColor(input_image, image, cv::COLOR_BGR2RGB);

    auto inputShape = m_snperuntime->GetInputShape(m_inputLayers[0]);

    size_t batch = inputShape[0];
    size_t inputHeight = inputShape[1];
    size_t inputWidth = inputShape[2];
    size_t channel = inputShape[3];

    if (m_snperuntime->GetInputTensor(m_inputLayers[0]) == nullptr) {
        LOG_ERROR("Empty input tensor");
        return false;
    }

    cv::Mat input(inputHeight, inputWidth, CV_32FC3, m_snperuntime->GetInputTensor(m_inputLayers[0]));

    int imgWidth = image.cols;
    int imgHeight = image.rows;

    m_scale = std::min(inputHeight /(float)imgHeight, inputWidth / (float)imgWidth);
    int scaledWidth = imgWidth * m_scale;
    int scaledHeight = imgHeight * m_scale;
    m_xOffset = (inputWidth - scaledWidth) / 2;
    m_yOffset = (inputHeight - scaledHeight) / 2;

    cv::Mat inputMat(inputHeight, inputWidth, CV_8UC3, cv::Scalar(128, 128, 128));
    cv::Mat roiMat(inputMat, cv::Rect(m_xOffset, m_yOffset, scaledWidth, scaledHeight));
    cv::resize(image, roiMat, cv::Size(scaledWidth, scaledHeight), cv::INTER_LINEAR);

    inputMat.convertTo(input, CV_32FC3);

    const float mean_vals[3] = {0.485, 0.456, 0.406};
    const float norm_vals[3] = {0.229, 0.224, 0.225};

    int i = 0, j = 0; float x,y,z;
    for (i = 0; i < input.rows; i++)
    {
      float* pdata = (float*)(input.data + i * input.step);
      for (j = 0; j < input.cols; j++)
      { x = pdata[2]; y=pdata[1]; z = pdata[0];
        pdata[0] = (x / 255.0 - mean_vals[0]) / norm_vals[0];
        pdata[1] = (y / 255.0 - mean_vals[1]) / norm_vals[1];
        pdata[2] = (z / 255.0 - mean_vals[2]) / norm_vals[2];
        pdata += 3;
      }
    }
    
    input = cv::dnn::blobFromImage(input);
    LOG_DEBUG("End of preprocess \n");

    return true;
}

bool ByteTrackSnpe::Detect(shared_ptr<DetectionItem> &item)
{
    uint32_t imageWidth = item->Width; 
    uint32_t imageHeight = item->Height;
    uint8_t *img = item->ImageBuffer.get();

    cv::Mat image(cv::Size(imageWidth, imageHeight), CV_8UC3, img, cv::Mat::AUTO_STEP);
    
    auto t0 = high_resolution_clock::now();
    PreProcessInput(image);
    auto t1 = high_resolution_clock::now();

    if (!m_snperuntime->execute()) {
        LOG_ERROR("SNPERuntime execute failed.");
        return false;
    }

    auto t2 = high_resolution_clock::now();
    PostProcess(item);
    auto t3 = high_resolution_clock::now();

    duration<double, std::milli> preprocesscost = t1 - t0;
    duration<double, std::milli> snpe_execut = t2 - t1;
    duration<double, std::milli> postprocesscost = t3 - t2;
    duration<double, std::milli> Total = t3 - t0;

    LOG_DEBUG("Preprocess cost: %f ms \n", preprocesscost);
    LOG_DEBUG("snpe_execut cost: %f ms \n", snpe_execut);
    LOG_DEBUG("Postprocess cost: %f ms \n", postprocesscost);
    LOG_DEBUG("Detect cost: %f ms \n", Total);

    return true;
}

static inline float intersection_area(const Object& a, const Object& b)
{
    cv::Rect_<float> inter = a.rect & b.rect;
    return inter.area();
}

static void qsort_descent_inplace(std::vector<Object>& faceobjects, int left, int right)
{
    int i = left;
    int j = right;
    float p = faceobjects[(left + right) / 2].prob;

    while (i <= j)
    {
        while (faceobjects[i].prob > p)
            i++;

        while (faceobjects[j].prob < p)
            j--;

        if (i <= j)
        {
            // swap
            std::swap(faceobjects[i], faceobjects[j]);

            i++;
            j--;
        }
    }

    #pragma omp parallel sections
    {
        #pragma omp section
        {
            if (left < j) qsort_descent_inplace(faceobjects, left, j);
        }
        #pragma omp section
        {
            if (i < right) qsort_descent_inplace(faceobjects, i, right);
        }
    }
}

static void qsort_descent_inplace(std::vector<Object>& objects)
{
    if (objects.empty())
        return;

    qsort_descent_inplace(objects, 0, objects.size() - 1);
}

void ByteTrackSnpe::generate_grids_and_stride(std::vector<int>& strides)
{
    for (int i = 0; i < (int)strides.size(); i++)
    {
        int stride = strides[i];
        int num_grid_w = INPUT_W / stride;
        int num_grid_h = INPUT_H / stride;
        for (int g1 = 0; g1 < num_grid_h; g1++)
        {
            for (int g0 = 0; g0 < num_grid_w; g0++)
            {
                GridAndStride gs;
                gs.grid0 = g0;
                gs.grid1 = g1;
                gs.stride = stride;
                grid_strides.push_back(gs);
            }
        }
    }
}

void ByteTrackSnpe::generate_yolox_proposals(const float* feat_ptr, std::vector<Object>& objects)
{
    const int num_anchors = grid_strides.size();
    for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++)
    {
        const int grid0 = grid_strides[anchor_idx].grid0;
        const int grid1 = grid_strides[anchor_idx].grid1;
        const int stride = grid_strides[anchor_idx].stride;

        // yolox/models/yolo_head.py decode logic
        //  outputs[..., :2] = (outputs[..., :2] + grids) * strides
        //  outputs[..., 2:4] = torch.exp(outputs[..., 2:4]) * strides
        float x_center = (feat_ptr[0] + grid0) * stride;
        float y_center = (feat_ptr[1] + grid1) * stride;
        float w = exp(feat_ptr[2]) * stride;
        float h = exp(feat_ptr[3]) * stride;
        float x0 = x_center - w * 0.5f;
        float y0 = y_center - h * 0.5f;

        float box_objectness = feat_ptr[4];
        for (int class_idx = 0; class_idx < num_class; class_idx++)
        {
            float box_cls_score = feat_ptr[5 + class_idx];
            float box_prob = box_objectness * box_cls_score;
            if (box_prob > m_confThresh)
            {
                Object obj;
                obj.rect.x = x0;
                obj.rect.y = y0;
                obj.rect.width = w;
                obj.rect.height = h;
                obj.label = class_idx;
                obj.prob = box_prob;

                objects.push_back(obj);
            }

        } // class loop
        feat_ptr += (num_class + 5);

    } // point anchor loop
}

void ByteTrackSnpe::nms_sorted_bboxes(const std::vector<Object>& faceobjects, std::vector<int>& picked)
{
    picked.clear();

    const int n = faceobjects.size();

    std::vector<float> areas(n);
    for (int i = 0; i < n; i++)
    {
        areas[i] = faceobjects[i].rect.area();
    }

    for (int i = 0; i < n; i++)
    {
        const Object& a = faceobjects[i];

        int keep = 1;
        for (int j = 0; j < (int)picked.size(); j++)
        {
            const Object& b = faceobjects[picked[j]];

            // intersection over union
            float inter_area = intersection_area(a, b);
            float union_area = areas[i] + areas[picked[j]] - inter_area;
            // float IoU = inter_area / union_area
            if (inter_area / union_area > m_nmsThresh)
                keep = 0;
        }

        if (keep)
            picked.push_back(i);
    }
}

bool ByteTrackSnpe::PostProcess(shared_ptr<DetectionItem> &item)
{
    std::vector<Object> objects = item->Results;  
    auto outputShape = m_snperuntime->GetOutputShape(m_outputTensors[0]);
    const float *predOutput = m_snperuntime->GetOutputTensor(m_outputTensors[0]);

    int batch = outputShape[0];
    int height = outputShape[1];
    int width = outputShape[2];

    num_grid= height;
    num_class = width-5;

    std::vector<Object> proposals;
    generate_yolox_proposals(predOutput, proposals);

    qsort_descent_inplace(proposals);

    // apply nms with m_nmsThresh
    std::vector<int> picked;
    nms_sorted_bboxes(proposals, picked);

    int count = picked.size();

    float scale = min(INPUT_W / (1280.0f), INPUT_H / (720.0f));

    objects.resize(count);
    for (int i = 0; i < count; i++)
    {
        objects[i] = proposals[picked[i]];

        // adjust offset to original unpadded
        float x0 = (objects[i].rect.x) / scale;
        float y0 = (objects[i].rect.y) / scale;
        float x1 = (objects[i].rect.x + objects[i].rect.width) / scale;
        float y1 = (objects[i].rect.y + objects[i].rect.height) / scale;

        objects[i].rect.x = x0;
        objects[i].rect.y = y0;
        objects[i].rect.width = x1 - x0;
        objects[i].rect.height = y1 - y0;
    }

    cv::Mat img(item->Height, item->Width, CV_8UC3, item->ImageBuffer.get());

    vector<STrack> output_stracks = tracker.update(objects);
    auto end = chrono::system_clock::now();
    app_time = chrono::duration_cast<chrono::milliseconds>(end - app_start).count();

    for (int i = 0; i < output_stracks.size(); i++)
    {
        vector<float> tlwh = output_stracks[i].tlwh;
        bool vertical = tlwh[2] / tlwh[3] > 1.6;
        if (tlwh[2] * tlwh[3] > 20 && !vertical)
        {
            Scalar s = tracker.get_color(output_stracks[i].track_id);
            putText(img, format("%d", output_stracks[i].track_id), Point(tlwh[0], tlwh[1] - 5), 
                    0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
            rectangle(img, Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), s, 2);
        }
    }

    ++frame_count;
    putText(img, format("frame: %d fps: %ld num: %d", frame_count, frame_count * 1000 / app_time, (int)output_stracks.size()),
            Point(0, 30), 0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);

    int size = img.rows * img.cols * 3;
    uint8_t *imgBuf = new uint8_t[size];
    memcpy(static_cast<uint8_t *>(imgBuf), img.data, size);
    item->ImageBuffer.reset((uint8_t *)imgBuf, [](uint8_t *p)
                                { delete[] (p); });

    return true;
}

} // namespace bytetracksnpe
