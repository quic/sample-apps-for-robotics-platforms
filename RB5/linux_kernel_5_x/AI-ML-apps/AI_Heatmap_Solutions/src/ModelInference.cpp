#include "ModelInference.h"
#include "Configuration.h"
#include <YOLOv5Snpe.h>

using namespace std;
using namespace cv;
using namespace dnn;
using namespace yolov5snpe;

/** @brief contructor
*/
ModelInference::ModelInference()
{
        Yolov5Impl = new YOLOV5Snpe();
}

/** @brief Parameter constructor
 * @param model_type To check model type from config file
*/
ModelInference::ModelInference(const string model_type)
{
    if (0 == model_type.compare("yolov5")) {            
        Yolov5Impl = new YOLOV5Snpe();
    }
    else
        LOG_ERROR("Model implementation not found\n");  

    LOG_INFO("Initialized model = %s \n", model_type.c_str());

}

/** @brief destructor
*/
ModelInference::~ModelInference()
{
    if (nullptr != Yolov5Impl) {
        delete static_cast<YOLOV5Snpe*>(Yolov5Impl);
        Yolov5Impl = nullptr;
    }
}

/** @brief For model inference 
 * @param item contains image buffer and results object to store results
 * @return true if success
*/
int ModelInference::Inference(shared_ptr<DetectionItem> &item)
{
    if (nullptr != Yolov5Impl && IsInitialized()) {
        return static_cast<YOLOV5Snpe*>(Yolov5Impl)->Detect(item);
    } else {
        LOG_ERROR("ObjectDetection::Detect failed caused by incompleted initialization!\n");
        return false;
    }
}

/** @brief To intialize SNPE
 * @param contains SNPE configuration
 * @return true if success 
*/
int ModelInference::Initialization(const ObjectDetectionSnpeConfig& config)
{
    if (IsInitialized()) {
        return static_cast<YOLOV5Snpe*>(Yolov5Impl)->DeInitialize() &&
               static_cast<YOLOV5Snpe*>(Yolov5Impl)->Initialize(config);
    } else {
        return static_cast<YOLOV5Snpe*>(Yolov5Impl)->Initialize(config);
    }
}

/** @brief To uninitialize SNPE
 * @return true if success
*/
bool ModelInference::UnInitialization()
{
    if (nullptr != Yolov5Impl && IsInitialized()) {
        return static_cast<YOLOV5Snpe*>(Yolov5Impl)->DeInitialize();
    } else {
        LOG_ERROR("ObjectDetection: deinit failed!\n");
        return false;
    }
}

/** @brief To check if SNPE is initialized
 * @return true if already inititalized
*/
bool ModelInference::IsInitialized()
{
    return static_cast<YOLOV5Snpe*>(Yolov5Impl)->IsInitialized();
}

void ModelInference::SetNotifyResultCallBack(NotifyDetectionResult callback)
{
  if (callback != nullptr)
  {
    notifyDetectionHandler = callback;
  }
}
