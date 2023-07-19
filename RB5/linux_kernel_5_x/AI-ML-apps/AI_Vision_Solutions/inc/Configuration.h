#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <string>
#include <vector>
#include <opencv2/imgproc.hpp>
#include "Utils.h"

using namespace cv;
using namespace std;

const string input_configs = "input-configs";
const string model_configs = "model-configs";
const string solution_configs = "solution-configs";

// Input Configs;
const string pipeline_input_config = "input-config-name";
const string stream_type = "stream-type";
const string camera_url = "camera-url";
const string skipframe = "SkipFrame";
const string fps_n = "fps-n";

// Model Configs
const string model_config_name = "model-name";
const string model_type = "model-type";
const string model_path = "model-path";
const string runtime = "runtime";
const string nms_threshold = "nms-threshold";
const string conf_threshold = "conf-threshold";
const string labels = "labels";
const string label_path = "label-path";
const string model_config_grids = "grids";
const string input_layers = "input-layers";
const string output_layers = "output-layers";
const string output_tensors = "output-tensors";

// Solution Configs
const string solution_name = "solution-name";
const string model_name = "model-name";
const string Enable = "Enable";
const string solution_input_config = "input-config-name";
const string strict_area_x = "strict_area_x";
const string strict_area_y = "strict_area_y";
const string strict_area_w = "strict_area_w";
const string strict_area_h = "strict_area_h";
const string output_type = "output-type";
const string output_path = "output-path";

// Solution Names
const string intrusion_detection = "people-intrusion-detection";
const string helmet_detection = "helmet-detection";
const string fire_detection = "fire-detection";
const string crack_detection = "crack-detection";
const string trash_detection = "trash-detection";

typedef struct _DetectionCordinates {
    int x;
    int y;
    int w;
    int h;
} DetectionCordinates;

class ObjectDetectionSnpeConfig {
    public:
    string model_name;
    string model_type;
    std::string model_path;
    runtime_t runtime;
    int grids;
    float nmsThresh;
    float confThresh;
    int label_count;
    std::string label_path;
    std::vector<std::string> labels;
    std::vector<std::string> inputLayers;
    std::vector<std::string> outputLayers;
    std::vector<std::string> outputTensors;
};

class InputConfiguration{
    public:
    int SkipFrame;
    int StreamNumber=0;
    string StreamType;
    string Url;
    string ConfigName;
    int camera_fps;
};

class SolutionConfiguration {
    public:
        string solution_name;
        string model_name;
        string input_config_name;
        bool Enable;
        string output_type;
        string output_path;
        DetectionCordinates detect_coordinate;
        std::shared_ptr<InputConfiguration> input_config;
        std::shared_ptr<ObjectDetectionSnpeConfig> model_config;
};

class RtspConfiguration
{
    public:
    int SkipFrame;
    int StreamNumber=0;
    vector<string> Urls;
};

class DebugConfiguration
{
    public:
    bool DumpData=false;
    string Directory;
};

class Configuration
{
public:
    static Configuration &getInstance()
    {
        static Configuration instance; // Guaranteed to be destroyed.
        return instance;
    }

private:
    Configuration() {}
public:
    Configuration(Configuration const &) = delete;
    void operator=(Configuration const &) = delete;

    RtspConfiguration Rtsp;
    DebugConfiguration Debug;
    ObjectDetectionSnpeConfig Config;
    SolutionConfiguration Sol_Config;
    std::unordered_map<std::string, std::shared_ptr<InputConfiguration>> inputconfigs;
    std::unordered_map<std::string, std::shared_ptr<ObjectDetectionSnpeConfig>> modelsconfig;
    std::unordered_map<int, std::shared_ptr<SolutionConfiguration>> solutionsconfig;

    void LoadConfiguration(string file);
    int LoadInputConfig(Json::Value& input);
    int LoadModelsConfig(Json::Value& models);
    int LoadSolutionsConfig(Json::Value& solutions);
};

#endif
