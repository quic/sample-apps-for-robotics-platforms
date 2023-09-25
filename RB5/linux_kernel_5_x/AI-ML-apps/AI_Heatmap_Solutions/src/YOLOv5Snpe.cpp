#include <math.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "Configuration.h"
#include "YOLOv5Snpe.h"

namespace yolov5snpe {

/** @brief Constructor
*/
YOLOV5Snpe::YOLOV5Snpe() : m_snperuntime(nullptr) {

}

/** @brief Destructor
*/
YOLOV5Snpe::~YOLOV5Snpe() {
    DeInitialize();
}

/** @brief To read model config and set output layers
 * @param config model config parameters
 * @return true if success;false otherwise
*/
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
    /**
     * To set output layer from model config
    */
    m_snperuntime->SetOutputLayers(m_outputLayers);
    /**
     * To initialize snperuntime 
    */
    if (!m_snperuntime->Initialize(config.model_path, config.runtime)) {
        LOG_ERROR("Can't init snpetask instance.\n");
        return false;
    }

    m_output = new float[m_grids * m_labels];
    m_isInit = true;
    return true;
}


/** @brief To deallocate buffers and reset
*/
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

/** @brief To preprocess input 
 * @return true if succuess; false otherwise
*/
bool YOLOV5Snpe::PreProcessInput(const cv::Mat& input_image)
{
    if (input_image.empty()) {
        LOG_ERROR("Invalid image!\n");
        return false;
    }

    /**
     * To convert BGR Image to RGB Image
    */ 
    cv::Mat image;
    cv::cvtColor(input_image, image, cv::COLOR_BGR2RGB);
    /**
     * To get input layer dimensions
    */
    auto inputShape = m_snperuntime->GetInputShape(m_inputLayers[0]);

    size_t batch = inputShape[0];
    size_t inputHeight = inputShape[1];
    size_t inputWidth = inputShape[2];
    size_t channel = inputShape[3];

    if (m_snperuntime->GetInputTensor(m_inputLayers[0]) == nullptr) {
        LOG_ERROR("Empty input tensor\n");
        return false;
    }
    /**
     * To create Mat buffer based on input layer dimensions
    */
    cv::Mat input(inputHeight, inputWidth, CV_32FC3, m_snperuntime->GetInputTensor(m_inputLayers[0]));

    int imgWidth  = image.cols;
    int imgHeight = image.rows;
    /**
     * Scaling factor to upscale at output
    */
    m_scale = std::min(inputHeight /(float)imgHeight, inputWidth / (float)imgWidth);
    int scaledWidth = imgWidth * m_scale;
    int scaledHeight = imgHeight * m_scale;
    m_xOffset = (inputWidth - scaledWidth) / 2;
    m_yOffset = (inputHeight - scaledHeight) / 2;

    cv::Mat inputMat(inputHeight, inputWidth, CV_8UC3, cv::Scalar(128, 128, 128));
    cv::Mat roiMat(inputMat, cv::Rect(m_xOffset, m_yOffset, scaledWidth, scaledHeight));
    cv::resize(image, roiMat, cv::Size(scaledWidth, scaledHeight), cv::INTER_LINEAR);
    /**
     * Copy image buffer to input layer 
    */
    inputMat.convertTo(input, CV_32FC3, 1.0 / 255);

    return true;
}

/** @brief To read image buffer and call preprocess,execute and postprocess
 * @param item contains imagebuffer and results to store detected objects
 * @return true if success; false otherwise
*/
bool YOLOV5Snpe::Detect(shared_ptr<DetectionItem> &item)
{
    uint32_t imageWidth = item->Width; 
    uint32_t imageHeight = item->Height;
    uint8_t *img = item->ImageBuffer.get();
    /**
     * Create Mat image from gstreamer buffer
    */
    cv::Mat image(cv::Size(imageWidth, imageHeight), CV_8UC3, img, cv::Mat::AUTO_STEP);
    /**
     * Preprocessing image
    */
    PreProcessInput(image);
    /**
     * Inferencing model on target
    */
    if (!m_snperuntime->execute()) {
        LOG_ERROR("SNPERuntime execute failed.");
        return false;
    }
    /**
     * Postprocessing to extract bounding boxes
    */
    PostProcess(item->Results);
    return true;
}

/** @brief To calculate power function
 * @param p input
 * @return f exponential value
*/
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

/** @brief To calculate exponential value
 * @param p input
 * @return exponential power
*/
inline float fastExp(float p) { return fastPow(1.442695040f * p); }

/** @brief custom sigmoid function
 * @param x input
 * @return sigmoid values
*/
inline float fastSigmoid(float x)
{
    return (1 / (1 + fastExp(-x)));
}


/** @brief Yolo postprocess to extract bounding boxes
 * @param results detected object results
*/
bool YOLOV5Snpe::PostProcess(std::vector<ObjectData> &results)
{
    float strides[3] = {8, 16, 32};
    float anchorGrid[][6] = {
        {10, 13, 16, 30, 33, 23},       // 8*8
        {30, 61, 62, 45, 59, 119},      // 16*16
        {116, 90, 156, 198, 373, 326},  // 32*32
    };

    int label_count_ = m_labels - 5; // Calculate 

    uint32_t detection_size = m_labels;

    std::vector<ObjectData> winList;
    for (size_t i = 0; i < 3; i++)
    {
        int anchorBoxIdx=i;
        uint32_t cnt = 0;
        /**
         * Output layer dimensions
        */
        auto outputShape = m_snperuntime->GetOutputShape(m_outputTensors[i]);
        /**
         * pointer to output tensor buffer to read data
        */
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
    /**
     * Apply NMS on detected bounding boxes
    */
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
