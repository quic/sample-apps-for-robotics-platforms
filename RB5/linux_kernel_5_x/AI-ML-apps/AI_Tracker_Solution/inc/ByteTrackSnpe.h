#ifndef __BYTETRACK_IMPL_H__
#define __BYTETRACK_IMPL_H__

#include <vector>
#include <string>
#include <unistd.h>
#include <memory>

#include "SNPERuntime.h"
#include "ModelInference.h"
#include "Configuration.h"
#include "BYTETracker.h"

namespace bytetracksnpe {

struct GridAndStride
{
    int grid0;
    int grid1;
    int stride;
};

class ByteTrackSnpe {
public:
    ByteTrackSnpe();
    ~ByteTrackSnpe();
    bool Initialize(const ObjectDetectionSnpeConfig& config);
    bool DeInitialize();
    //bool Detect(const cv::Mat& image, std::vector<ObjectData>& results);
    bool Detect(shared_ptr<DetectionItem> &item);
    bool SetScoreThresh(const float& conf_thresh, const float& nms_thresh = 0.5) noexcept {
        this->m_nmsThresh  = nms_thresh;
        this->m_confThresh = conf_thresh;
        return true;
    }

    bool IsInitialized() const {
        return m_isInit;
    }

private:
    bool m_isInit = false;

    bool PreProcessInput(const cv::Mat& frame);
    bool PostProcess(shared_ptr<DetectionItem> &item);

    std::unique_ptr<snperuntime::SNPERuntime> m_snperuntime;
    std::vector<std::string> m_inputLayers;
    std::vector<std::string> m_outputLayers;
    std::vector<std::string> m_outputTensors;

    int m_labels;
    int m_grids;
    float* m_output;

    int m_minBoxBorder = 16;
    float m_nmsThresh;
    float m_confThresh;
    float m_scale;
    int m_xOffset, m_yOffset;

    const int stride_arr[3] = {8, 16, 32}; // might have stride=64 in YOLOX
    std::vector<GridAndStride> grid_strides;
    void generate_grids_and_stride(std::vector<int>& strides);
    void generate_yolox_proposals(const float* feat_ptr, std::vector<Object>& objects);
    void nms_sorted_bboxes(const std::vector<Object>& faceobjects, std::vector<int>& picked);
    const int INPUT_W = 1088;
    const int INPUT_H = 608;
    int num_grid;
    int num_class;
    int frame_count = 0;
    unsigned long app_time;
    std::chrono::time_point<std::chrono::system_clock> app_start;

    BYTETracker tracker;

};

} // namespace by bytetracksnpe

#endif // __BYTETRACK_IMPL_H__
