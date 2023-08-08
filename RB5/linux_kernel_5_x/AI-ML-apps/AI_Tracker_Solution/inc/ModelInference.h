#ifndef MODEL_INFERENCE_H_
#define MODEL_INFERENCE_H_
#include "DecodeQueue.h"
#include <vector>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/dnn.hpp>
#include "Configuration.h"

typedef void (*NotifyDetectionResult)(shared_ptr<DetectionItem> &result, SolutionConfiguration *solution_config);

class ModelInference{
public:
    ModelInference();
    ModelInference(const string model_name);
    int Initialization(const ObjectDetectionSnpeConfig& config);
    bool IsInitialized();
    bool UnInitialization();
    ~ModelInference();
    void SetNotifyResultCallBack(NotifyDetectionResult callback);
    int Inference(shared_ptr<DetectionItem> &item);
private:
    NotifyDetectionResult notifyDetectionHandler=nullptr;
    // Handler for all the methods of object detection
    void *ByteTrackImpl  = nullptr;
};

#endif