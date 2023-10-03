#ifndef __GOOGLENET_IMPL_H__
#define __GOOGLENET_IMPL_H__

#include <vector>
#include <string>
#include <unistd.h>
#include <memory>

#include "SNPERuntime.h"
#include "ModelInference.h"
#include "Configuration.h"

namespace googlenetsnpe {

class GOOGLENETSnpe {
public:
    GOOGLENETSnpe();
    ~GOOGLENETSnpe();
    bool Initialize(const ObjectDetectionSnpeConfig& config);
    bool DeInitialize();
    
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
    bool PostProcess(string* age);

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
};

} // namespace googlenet

#endif // __GOOGLENET_IMPL_H__
