#include "ModelInference.h"
#include "Configuration.h"
#include <YOLOv5Snpe.h>

using namespace std;
using namespace cv;
using namespace dnn;
using namespace yolov5snpe;

ModelInference::ModelInference()
{
        Yolov5Impl = new YOLOV5Snpe();
}

ModelInference::ModelInference(const string model_type)
{
    if (0 == model_type.compare("yolov5")) {            
        Yolov5Impl = new YOLOV5Snpe();
    }
    else
        LOG_ERROR("Model implementation not found");  

    LOG_INFO("Initialized model = %s \n", model_type.c_str());

}

ModelInference::~ModelInference()
{
    if (nullptr != Yolov5Impl) {
        delete static_cast<YOLOV5Snpe*>(Yolov5Impl);
        Yolov5Impl = nullptr;
    }
}

int ModelInference::Inference(shared_ptr<DetectionItem> &item)
{
    if (nullptr != Yolov5Impl && IsInitialized()) {
        auto ret = static_cast<YOLOV5Snpe*>(Yolov5Impl)->Detect(item);
        return ret;
    } else {
        LOG_ERROR("ObjectDetection::Detect failed caused by incompleted initialization!");
        return false;
    }
}

int ModelInference::Initialization(const ObjectDetectionSnpeConfig& config)
{
    if (IsInitialized()) {
        return static_cast<YOLOV5Snpe*>(Yolov5Impl)->DeInitialize() &&
               static_cast<YOLOV5Snpe*>(Yolov5Impl)->Initialize(config);
    } else {
        return static_cast<YOLOV5Snpe*>(Yolov5Impl)->Initialize(config);
    }
}

bool ModelInference::UnInitialization()
{
    if (nullptr != Yolov5Impl && IsInitialized()) {
        return static_cast<YOLOV5Snpe*>(Yolov5Impl)->DeInitialize();
    } else {
        LOG_ERROR("ObjectDetection: deinit failed!");
        return false;
    }
}

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
