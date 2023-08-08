#include "ModelInference.h"
#include "Configuration.h"
#include <ByteTrackSnpe.h>

using namespace std;
using namespace cv;
using namespace dnn;
using namespace bytetracksnpe;

ModelInference::ModelInference()
{
        ByteTrackImpl = new ByteTrackSnpe();
}

ModelInference::ModelInference(const string model_type)
{
    if (0 == model_type.compare("bytetrack")) {            
        ByteTrackImpl = new ByteTrackSnpe();
    }
    else
        LOG_ERROR("Model implementation not found");  

    LOG_INFO("Initialized model = %s \n", model_type.c_str());

}

ModelInference::~ModelInference()
{
    if (nullptr != ByteTrackImpl) {
        delete static_cast<ByteTrackSnpe*>(ByteTrackImpl);
        ByteTrackImpl = nullptr;
    }
}

int ModelInference::Inference(shared_ptr<DetectionItem> &item)
{
    if (nullptr != ByteTrackImpl && IsInitialized()) {
        auto ret = static_cast<ByteTrackSnpe*>(ByteTrackImpl)->Detect(item);
        return ret;
    } else {
        LOG_ERROR("ObjectDetection::Detect failed caused by incompleted initialization!");
        return false;
    }
}

int ModelInference::Initialization(const ObjectDetectionSnpeConfig& config)
{
    if (IsInitialized()) {
        return static_cast<ByteTrackSnpe*>(ByteTrackImpl)->DeInitialize() &&
               static_cast<ByteTrackSnpe*>(ByteTrackImpl)->Initialize(config);
    } else {
        return static_cast<ByteTrackSnpe*>(ByteTrackImpl)->Initialize(config);
    }
}

bool ModelInference::UnInitialization()
{
    if (nullptr != ByteTrackImpl && IsInitialized()) {
        return static_cast<ByteTrackSnpe*>(ByteTrackImpl)->DeInitialize();
    } else {
        LOG_ERROR("ObjectDetection: deinit failed!");
        return false;
    }
}

bool ModelInference::IsInitialized()
{
    return static_cast<ByteTrackSnpe*>(ByteTrackImpl)->IsInitialized();
}

void ModelInference::SetNotifyResultCallBack(NotifyDetectionResult callback)
{
  if (callback != nullptr)
  {
    notifyDetectionHandler = callback;
  }
}
