#include <math.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "Configuration.h"
#include "GooglenetSnpe.h"

namespace googlenetsnpe {

GOOGLENETSnpe::GOOGLENETSnpe() : m_snperuntime(nullptr) {

}

GOOGLENETSnpe::~GOOGLENETSnpe() {
    DeInitialize();
}

bool GOOGLENETSnpe::Initialize(const ObjectDetectionSnpeConfig& config)
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

bool GOOGLENETSnpe::DeInitialize()
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

bool GOOGLENETSnpe::PreProcessInput(const cv::Mat& input_image)
{
        if (input_image.empty()) {
                LOG_ERROR("Invalid image!\n");
                return false;
        }
        auto inputShape = m_snperuntime->GetInputShape(m_inputLayers[0]);

        size_t inputHeight = inputShape[1];
        size_t inputWidth = inputShape[2];
        float *tensor = NULL;
        if ( (tensor =  m_snperuntime->GetInputTensor(m_inputLayers[0])) == nullptr) {
                LOG_ERROR("Empty input tensor\n");
                return false;
        }
	cv::Mat image;
        //Resize the input image
        cv::resize(input_image,image, cv::Size(inputWidth,inputHeight));

        cv::subtract(image,cv::Scalar(78.4263377603, 87.7689143744, 114.895847746),image);
        cv::Mat input(inputHeight, inputWidth, CV_32FC3, tensor);
        image.convertTo(input,CV_32FC3,1.0);
        return true;
    
}

bool GOOGLENETSnpe::Detect(shared_ptr<DetectionItem> &item)
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

                cv::Rect myROI( (int)result.face_info.x1, (int)result.face_info.y1, (int)result.face_info.x2 - (int)result.face_info.x1, (int)result.face_info.y2 - (int)result.face_info.y1);
	           
                        cv::Mat croppedImage = image(myROI);
                        PreProcessInput(croppedImage);
                        if (!m_snperuntime->execute()) {
                                LOG_ERROR("SNPERuntime execute failed.\n");
                                return false;
                        }
                        PostProcess(&item->Results[j].age);
                }
        }
		
        return true;

}

bool GOOGLENETSnpe::PostProcess(string* age)
{
        string ageList[8] = { "(0-2)", "(4-6)", "(8-12)", "(15-20)", "(25-32)", "(38-43)", "(48-53)", "(60-100)" };
        auto outputShape = m_snperuntime->GetOutputShape(m_outputTensors[0]);
        float *age_predOutput_box = m_snperuntime->GetOutputTensor(m_outputTensors[0]);
       
        int batchSize = outputShape[0];
        int elements = outputShape[1];
	float max_value = 0;
	int max_id = 0;
	for(int j = 0; j< batchSize * elements; j++)
	{
		if(age_predOutput_box[j] > max_value)
		{
				max_value = age_predOutput_box[j];
				max_id = j;
		}
	}	

	*age = ageList[max_id];
	return true;
}

} // namespace googlenet
