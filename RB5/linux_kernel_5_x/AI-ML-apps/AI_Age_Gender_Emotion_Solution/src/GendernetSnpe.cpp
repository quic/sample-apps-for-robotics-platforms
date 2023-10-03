#include <math.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "Configuration.h"
#include "GendernetSnpe.h"

namespace gendernetsnpe {

GENDERNETSnpe::GENDERNETSnpe() : m_snperuntime(nullptr) {

}

GENDERNETSnpe::~GENDERNETSnpe() {
    DeInitialize();
}

bool GENDERNETSnpe::Initialize(const ObjectDetectionSnpeConfig& config)
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
                LOG_ERROR("Can't init snpetask instance.\n");
                return false;
        }

        m_output = new float[m_grids * m_labels];
        m_isInit = true;

        return true;
}

bool GENDERNETSnpe::DeInitialize()
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

bool GENDERNETSnpe::PreProcessInput(const cv::Mat& input_image)
{
        if (input_image.empty()) {
                LOG_ERROR("Invalid image!\n");
                return false;
        }

        // Convert BGR Image to RGB Image
        cv::Mat image;
        cv::cvtColor(input_image, image, cv::COLOR_BGR2RGB);
        auto inputShape = m_snperuntime->GetInputShape(m_inputLayers[0]);

        size_t inputHeight = inputShape[1];
        size_t inputWidth = inputShape[2];

        float *tensor = NULL;
        if ( (tensor =  m_snperuntime->GetInputTensor(m_inputLayers[0])) == nullptr) {
                LOG_ERROR("Empty input tensor\n");
                return false;
        }

        cv::resize(image,image, cv::Size(inputWidth,inputHeight));
        cv::subtract(image,cv::Scalar(104,117,123),image);
        cv::Mat input(inputHeight, inputWidth, CV_32FC3, tensor);
        image.convertTo(input,CV_32FC3,1.0);
               
        return true;
}

bool GENDERNETSnpe::Detect(shared_ptr<DetectionItem> &item)
{
        uint8_t *img = item->ImageBuffer.get();
        uint32_t imageWidth = item->Width;
        uint32_t imageHeight = item->Height;
                
        cv::Mat image(cv::Size(imageWidth, imageHeight), CV_8UC3, img, cv::Mat::AUTO_STEP);
        for (size_t j = 0; j < item->Results.size(); j++) 
        {
                ObjectData result = item->Results[j];
                if(result.face_info.x1 - 30 > 0 && result.face_info.y1 - 30 > 0 && result.face_info.x2 - 30 > 0 && result.face_info.y2 - 30 > 0)
                {
                        cv::Rect myROI( (int)result.face_info.x1 - 30, (int)result.face_info.y1 -30, (int)result.face_info.x2 - (int)result.face_info.x1 + 50, (int)result.face_info.y2 - (int)result.face_info.y1 +50);
                        cv::Mat croppedImage = image(myROI);
                        PreProcessInput(croppedImage);
                        if (!m_snperuntime->execute()) {
                                LOG_ERROR("SNPERuntime execute failed.\n");
                                return false;
                        }
                        PostProcess(&item->Results[j].gender);
                }
        }

        return true;
}

bool GENDERNETSnpe::PostProcess(string* gender)
{
        string genderList[2] = { "Male","Female" };
        auto outputShape = m_snperuntime->GetOutputShape(m_outputTensors[0]);
        float *gender_predOutput = m_snperuntime->GetOutputTensor(m_outputTensors[0]);
        int batchSize = outputShape[0];
        int elements = outputShape[1];
	float max_value = 0;
	int max_id = 0;
	for(int j = 0; j< batchSize * elements; j++)
	{
		if(gender_predOutput[j] > max_value)
		{
				max_value = gender_predOutput[j];
				max_id = j;
		}
	}	

	*gender = genderList[max_id];
       
	return true;
}

} // namespace gendernet
