#include "Configuration.h"
#include "Utils.h"
#include <map>

static runtime_t device2runtime(std::string&& device)
{
    std::transform(device.begin(), device.end(), device.begin(),
        [](unsigned char ch){ return tolower(ch); });

    if (0 == device.compare("cpu")) {
        return CPU;
    } else if (0 == device.compare("gpu")) {
        return GPU;
    } else if (0 == device.compare("gpu_float16")) {
        return GPU_FLOAT16;
    } else if (0 == device.compare("dsp")) {
        return DSP;
    } else if (0 == device.compare("aip")) {
        return AIP;
    } else { 
        return CPU;
    }
}

int Configuration::LoadInputConfig(Json::Value& input) {
     if (input.isArray()) {
        int size = input.size();
            for (int i = 0; i < size; ++i) {
                std::shared_ptr<InputConfiguration> inputconfig = std::shared_ptr<InputConfiguration>(new InputConfiguration());
                inputconfig->ConfigName = input[i][pipeline_input_config].asString();
                inputconfig->StreamType = input[i][stream_type].asString();
                inputconfig->Url = input[i][camera_url].asString();
                inputconfig->SkipFrame = input[i][skipframe].asInt();
                inputconfig->camera_fps = input[i][fps_n].asInt();
                inputconfigs[inputconfig->ConfigName] = inputconfig;
            }
     }
     LOG_INFO("Input streams size=%u \n", input.size());
     return 0;
}

int Configuration::LoadModelsConfig(Json::Value& models) {
    std::string line;
    if (models.isArray()) {
        int size = models.size();
            for (int i = 0; i < size; ++i) {
                std::shared_ptr<ObjectDetectionSnpeConfig> modelconfig = 
                    std::shared_ptr<ObjectDetectionSnpeConfig>(new ObjectDetectionSnpeConfig());
                modelconfig->model_name = models[i][model_config_name].asString();
                modelconfig->model_type = models[i][model_type].asString();
                modelconfig->model_path = models[i][model_path].asString();
                modelconfig->runtime = device2runtime(models[i][runtime].asString());
                modelconfig->nmsThresh = models[i][nms_threshold].asFloat();
                modelconfig->confThresh = models[i][conf_threshold].asFloat();
                modelconfig->label_count = models[i][labels].asInt();
                modelconfig->label_path = models[i][label_path].asString();

                std::ifstream in(modelconfig->label_path);
                while (getline(in, line)){
                    modelconfig->labels.push_back(line);
                }
                
                modelconfig->grids = models[i][model_config_grids].asInt();
                if (models[i]["input-layers"].isArray()) {
                    int num = models[i]["input-layers"].size();
                    for (int  j= 0; j < num; j++) {
                        modelconfig->inputLayers.push_back(models[i]["input-layers"][j].asString());
                    }
                }
                if (models[i][output_layers].isArray()) {
                    int num = models[i]["output-layers"].size();
                    for (int j = 0; j < num; j++) {
                        modelconfig->outputLayers.push_back(models[i]["output-layers"][j].asString());
                    }
                }
                if (models[i][output_tensors].isArray()) {
                    int num = models[i]["output-tensors"].size();
                    for (int j = 0; j < num; j++) {
                        modelconfig->outputTensors.push_back(models[i]["output-tensors"][j].asString());
                    }
                }
                
                modelsconfig[modelconfig->model_name] = modelconfig;
            }
        }
        
        LOG_INFO("modelsconfig size = %lu \n", modelsconfig.size());
        return 0;

}

int Configuration::LoadSolutionsConfig(Json::Value& solutions) {
    if (solutions.isArray()) {
        int size = solutions.size();
            for (int i = 0; i < size; ++i) {
                std::shared_ptr<SolutionConfiguration> solutionconfig = std::shared_ptr<SolutionConfiguration>(new SolutionConfiguration());
                solutionconfig->solution_name = solutions[i][solution_name].asString();
                solutionconfig->model_name = solutions[i][model_name].asString();
                solutionconfig->Enable = solutions[i][Enable].asBool();
                solutionconfig->input_config_name = solutions[i][solution_input_config].asString();
                solutionconfig->detect_coordinate.x = solutions[i][strict_area_x].asInt();
                solutionconfig->detect_coordinate.y = solutions[i][strict_area_y].asInt();  
                solutionconfig->detect_coordinate.w = solutions[i][strict_area_w].asInt();  
                solutionconfig->detect_coordinate.h = solutions[i][strict_area_h].asInt();
                solutionconfig->output_type = solutions[i][output_type].asString();
                solutionconfig->output_path = solutions[i][output_path].asString();
                solutionsconfig[i] = solutionconfig;
            }
     }
     LOG_DEBUG("Solutions size %lu", solutionsconfig.size() );
     return 0;

}

void Configuration::LoadConfiguration(string configFilePath)
{
    Json::Reader reader;
    Json::Value root;
    std::ifstream in(configFilePath, std::ios::binary);
    reader.parse(in, root);

    LoadInputConfig(root[input_configs]);
    LoadModelsConfig(root[model_configs]);
    LoadSolutionsConfig(root[solution_configs]);

}