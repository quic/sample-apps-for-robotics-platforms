#ifndef DETECTION_H
#define DETECTION_H

#include <vector>
#include <string>
#include <opencv2/imgproc.hpp>
#include <memory>
#include "BYTETracker.h"

using namespace std;
using namespace cv;

struct ObjectData {
    // Bounding box information: top-left coordinate and width, height
    cv::Rect bbox;
    // Confidence of this bounding box
    float confidence = -1.0f;
    // The label of this Bounding box
    int label = -1;
    // Time cost of detecting this frame
    size_t time_cost = 0;
};

struct Detection
{
    cv::Rect bbox;
    float score;
    int label;
};

struct DetectionDetail
{
    vector<Detection> Result;
    string ModelName;
};

struct DetectionItem
{
    uint32_t Width;
    uint32_t Height;
    uint32_t FrameId;
    size_t Size; 
    string StreamName;
    int StreamId;
    shared_ptr<uint8_t> ImageBuffer;   
//    vector<DetectionDetail> Results;
    vector<Object> Results;
};

#endif