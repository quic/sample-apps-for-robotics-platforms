#ifndef __YOLOV5S_IMPL_H__
#define __YOLOV5S_IMPL_H__

#include <vector>
#include <string>
#include <unistd.h>
#include <memory>

#include "SNPERuntime.h"
#include "ModelInference.h"
#include "Configuration.h"

namespace yolov5snpe {

#define NUM_COORDINATES 4

static float computeIoU(const cv::Rect& a, const cv::Rect& b) {
    float xOverlap = std::max(
        0.,
        std::min(a.x + a.width, b.x + b.width) - std::max(a.x, b.x) + 1.);
    float yOverlap = std::max(
        0.,
        std::min(a.y + a.height, b.y + b.height) - std::max(a.y, b.y) + 1.);
    float intersection = xOverlap * yOverlap;
    float unio =
        (a.width + 1.) * (a.height + 1.) +
        (b.width + 1.) * (b.height + 1.) - intersection;
    return intersection / unio;
}

class YOLOV5Snpe {
public:
    YOLOV5Snpe();
    ~YOLOV5Snpe();
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

    static std::vector<ObjectData> doNMS(std::vector<ObjectData> winList, const float& nms_thresh) {
        if (winList.empty()) {
            return winList;
        }

        std::sort(winList.begin(), winList.end(), [] (const ObjectData& left, const ObjectData& right) {
            if (left.confidence > right.confidence) {
                return true;
            } else {
                return false;
            }
        });

        std::vector<bool> flag(winList.size(), false);
        for (unsigned int i = 0; i < winList.size(); i++) {
            if (flag[i]) {
                continue;
            }

            for (unsigned int j = i + 1; j < winList.size(); j++) {
                if (computeIoU(winList[i].bbox, winList[j].bbox) > nms_thresh) {
                    flag[j] = true;
                }
            }
        }

        std::vector<ObjectData> ret;
        for (unsigned int i = 0; i < winList.size(); i++) {
            if (!flag[i])
                ret.push_back(winList[i]);
        }

        return ret;
    }

private:
    bool m_isInit = false;

    bool PreProcessInput(const cv::Mat& frame);
    bool PostProcess(std::vector<ObjectData>& results);

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

} // namespace yolov5

#endif // __YOLOV5S_IMPL_H__
