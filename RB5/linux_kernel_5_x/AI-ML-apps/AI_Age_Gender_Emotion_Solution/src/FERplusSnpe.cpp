#include <math.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "Configuration.h"
#include "FERplusSnpe.h"

namespace FERplussnpe {

FERPLUSSnpe::FERPLUSSnpe() : m_snperuntime(nullptr) {

}

FERPLUSSnpe::~FERPLUSSnpe() {
    DeInitialize();
}

bool FERPLUSSnpe::Initialize(const ObjectDetectionSnpeConfig& config)
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

bool FERPLUSSnpe::DeInitialize()
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

bool FERPLUSSnpe::PreProcessInput(const cv::Mat& input_image)
{
       if (input_image.empty()) {
                LOG_ERROR("Invalid image!\n");
                return false;
        }

        // Convert BGR Image to GRAY Image
        cv::Mat image;
        cv::cvtColor(input_image, image, cv::COLOR_BGR2GRAY);
        auto inputShape = m_snperuntime->GetInputShape(m_inputLayers[0]);

        size_t inputHeight = inputShape[1];
        size_t inputWidth = inputShape[2];

        float *tensor = NULL;
        if ( (tensor =  m_snperuntime->GetInputTensor(m_inputLayers[0])) == nullptr) {
                LOG_ERROR("Empty input tensor\n");
                return false;
        }
        //Resize the image
        cv::resize(image,image, cv::Size(inputWidth,inputHeight));
        cv::Mat input(inputHeight, inputWidth, CV_32FC1, tensor);
        image.convertTo(input,CV_32FC1,1.0/256);	
        return true;
    
}

bool FERPLUSSnpe::Detect(shared_ptr<DetectionItem> &item)
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
                        cv::Rect myROI( (int)result.face_info.x1 - 30, (int)result.face_info.y1 - 30, (int)result.face_info.x2 - (int)result.face_info.x1 + 60, (int)result.face_info.y2 - (int)result.face_info.y1 +60);
                        cv::Mat croppedImage = image(myROI);
                        PreProcessInput(croppedImage);
                        if (!m_snperuntime->execute()) {
                                LOG_ERROR("SNPERuntime execute failed.\n");
                                return false;
                        }
                        PostProcess(&item->Results[j].emotion);
                }
        }

        return true;
}

bool FERPLUSSnpe::PostProcess(string* emotion)
{
        string emotionList[EMOTIONS] = { "neutral", "happiness", "surprise",  "sadness","anger", "disguest", "fear" };
        auto outputShape = m_snperuntime->GetOutputShape(m_outputTensors[0]);
        float *emotion_predOutput_box = m_snperuntime->GetOutputTensor(m_outputTensors[0]);

        int batchSize = outputShape[0];
        int elements = outputShape[1];
	float max_value = 0;
	int max_id = 0;
	float sum=0;
	float pred[EMOTIONS];
        //Apply Softmax operation to the output layer
	for(int i=0;i<EMOTIONS;i++){
                sum+=exp(emotion_predOutput_box[i]);
        }
       
        for(int i=0;i<EMOTIONS;i++){
                pred[i]=(exp(emotion_predOutput_box[i])/sum);
        }

	for(int j = 0; j< batchSize * elements; j++)
	{
		if(pred[j] > max_value)
		{
				max_value = pred[j];
				max_id = j;
		}
	}	

	*emotion = emotionList[max_id];
       
	return true;
}

} // namespace FERplus
