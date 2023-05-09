#include <math.h>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include "Configuration.h"

#include "YOLOv5Snpe.h"

namespace yolov5snpe {

YOLOV5Snpe::YOLOV5Snpe() : m_snperuntime(nullptr) {

}

YOLOV5Snpe::~YOLOV5Snpe() {
    DeInitialize();
}

bool YOLOV5Snpe::Initialize(const ObjectDetectionSnpeConfig& config)
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

bool YOLOV5Snpe::DeInitialize()
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

bool YOLOV5Snpe::PreProcessInput(const cv::Mat& input_image)
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

    inputMat.convertTo(input, CV_32FC3, 1.0 / 255);

    return true;
}

bool YOLOV5Snpe::Detect(shared_ptr<DetectionItem> &item)
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
    PostProcess(item->Results);
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

static float fastPow(float p) {
  float offset = (p < 0) ? 1.0f : 0.0f;
  float clipp = (p < -126) ? -126.0f : p;
  int32_t w = (int32_t)clipp;
  float z = clipp - w + offset;
  union {
    uint32_t i;
    float f;
  } v = {(uint32_t)((1 << 23) *
                    (clipp + 121.2740575f + 27.7280233f / (4.84252568f - z) -
                     1.49012907f * z))};

  return v.f;
}

inline float fastExp(float p) { return fastPow(1.442695040f * p); }

inline float fastSigmoid(float x)
{
    return (1 / (1 + fastExp(-x)));
}

bool YOLOV5Snpe::PostProcess(std::vector<ObjectData> &results)
{
    float strides[3] = {8, 16, 32};
    float anchorGrid[][6] = {
        {10, 13, 16, 30, 33, 23},       // 8*8
        {30, 61, 62, 45, 59, 119},      // 16*16
        {116, 90, 156, 198, 373, 326},  // 32*32
    };

    int label_count_ = m_labels - 1 - 4; // Calculate 

    uint32_t detection_size = m_labels;

    std::vector<ObjectData> winList;
    for (size_t i = 0; i < 3; i++)
    {
        int anchorBoxIdx=i;
        uint32_t cnt = 0;
        auto outputShape = m_snperuntime->GetOutputShape(m_outputTensors[i]);
        float *predOutput = m_snperuntime->GetOutputTensor(m_outputTensors[i]);

        int batchSize = outputShape[0];
        int height = outputShape[1];
        int width = outputShape[2]; 
        int channel = outputShape[3];

        // NHWC
        for (int bs = 0; bs < batchSize; bs++)
        {
		   // anchor Box Process
		   for (int h = 0; h < height; h++)
		   { // 80/40/20
			   for (int w = 0; w < width; w++)
			   { // 80/40/20
				   for (int winIdx = 0; winIdx < 3; winIdx++)
				   { // 3
					   float *pbox = predOutput + cnt * detection_size;
					   float *pclass = pbox + NUM_COORDINATES + 1;

					   cnt++;
					   float objectConfidenceScore = fastSigmoid(pbox[4]);
					   //if (objectConfidenceScore <m_confThresh)
					   if (objectConfidenceScore <= m_confThresh)
					   {
						   continue;
					   }

					   uint32_t max_class_index = 0;
					   if (label_count_ > 1)
					   {
						   max_class_index = std::distance(pclass, std::max_element(pclass, pclass + label_count_));
					   }
					   float max_objprob = fastSigmoid(pclass[max_class_index]);
					   float score=max_objprob * objectConfidenceScore;

					   if (score <= m_confThresh)
					   {
						   continue;
					   }

					   float stride = strides[anchorBoxIdx];

					   ObjectData rect;

					   float bx=(fastSigmoid(pbox[0]) * 2 - 0.5 + w) * stride;

					   float by=(fastSigmoid(pbox[1]) * 2 - 0.5 + h) * stride;

					   float sigVal = fastSigmoid(pbox[2]);
					   
					   float bw = sigVal * sigVal * 4 * anchorGrid[i][winIdx * 2];
					  
					   sigVal = fastSigmoid(pbox[3]);						
					   float bh = sigVal * sigVal * 4 * anchorGrid[i][(winIdx * 2) + 1];

					   bx = std::max(0, static_cast<int>(bx - bw / 2)) - m_xOffset;
					   by = std::max(0, static_cast<int>(by - bh / 2)) - m_yOffset;

					   rect.bbox.width = bw/m_scale;
					   rect.bbox.height = bh/m_scale;
					   rect.bbox.x = bx/m_scale;
					   rect.bbox.y = by/m_scale;

					   rect.confidence = score;
					   rect.label =max_class_index;
					   winList.push_back(rect);
				   }
			   }
		   }
	    }
    }
    winList = doNMS(winList, m_nmsThresh);
    for (size_t i = 0; i < winList.size(); i++)
    {
        if (winList[i].bbox.width >= m_minBoxBorder || winList[i].bbox.height >= m_minBoxBorder)
        {
            results.push_back(winList[i]);
        }
    }

    return true;
}

} // namespace yolov5snpe
