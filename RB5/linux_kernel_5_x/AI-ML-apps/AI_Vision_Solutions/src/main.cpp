#include "ModelInference.h"
#include "StreamDecode.h"
#include "StreamEncode.h"
#include "Configuration.h"
#include "DecodeQueue.h"

#include <getopt.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <sys/stat.h>

#include <unistd.h>

using namespace std;

shared_ptr<DecodeQueue> gDecodeQueue;
bool gExit = false;

shared_ptr<EncodeController> encoderCtrl;
shared_ptr<CaptureController> captureCtrl;

std::chrono::time_point<std::chrono::steady_clock> app_start;

static void OnNotifyResult(shared_ptr<DetectionItem> &item, SolutionConfiguration* solution_config)
{
    LOG_INFO("result size: %lu \n", item->Results.size());
    unsigned int people_count = 0;
    static unsigned long frame_count= 0;

    std::vector<std::string> labels = solution_config->model_config->labels;

    auto total_ms = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - app_start).count();

    if (item->Results.size() > 0)
    {
        cv::Mat img(item->Height, item->Width, CV_8UC3, item->ImageBuffer.get());

        for (size_t j = 0; j < item->Results.size(); j++) {
            ObjectData result = item->Results[j];

            if(0 == solution_config->solution_name.compare("people-intrusion-detection"))
            {
                DetectionCordinates config_ = solution_config->detect_coordinate;

                // Create Box for restricted area
                cv::Point position = cv::Point(config_.x, config_.y - 10);
                cv::putText(img, "Restricted Area", position, cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(0, 0, 255), 2, 0.6);
                cv::rectangle(img, cv::Rect(config_.x, config_.y, config_.w, config_.h), cv::Scalar(0, 0, 255), 3);

                if(result.label != 0)
                {
                    continue;
                }

                if ((config_.x != 0) || (config_.y != 0) || (config_.w != 0) ||
                    (config_.h != 0)) 
                {
                    if (!(((result.bbox.x > config_.x) && (result.bbox.x < config_.x + config_.w)) ||
                                ((result.bbox.x + result.bbox.width > config_.x) && (result.bbox.x + result.bbox.width < config_.x + config_.w)))) {
                        continue;
                    }

                    if (!(((result.bbox.y > config_.y) && (result.bbox.y < config_.y + config_.h)) ||
                                ((result.bbox.y + result.bbox.height > config_.y) && (result.bbox.y + result.bbox.height < config_.y + config_.h)))) {
                        continue;
                    }
                }
		
            }
            else if(0 == solution_config->solution_name.compare("people-counting"))
            {
                if(result.label != 0)
                {
                    continue;
                }
                people_count++;

            }

            cv::rectangle(img, cv::Rect(result.bbox.x, result.bbox.y, result.bbox.width, result.bbox.height), cv::Scalar(255, 0, 0), 3);
            cv::Point position = cv::Point(result.bbox.x, result.bbox.y - 10);
            cv::putText(img, labels[result.label], position, cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(255, 0, 0), 2, 0.3);
        }

        frame_count++;
        if (0 == solution_config->solution_name.compare("people-counting")) {
            cv::putText(img, "People Count: " + to_string(people_count), cv::Point(30,50), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(0, 0, 255), 2, LINE_AA);
        }

        putText(img, format("Rendered frame: %lu fps: %ld", frame_count, frame_count * 1000 / total_ms),
                Point(30, 30), 0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);

        int size = img.total() * img.elemSize();
        encoderCtrl->EncodeFrame(item->StreamId, img.data, size);
    }
    else
    {
            frame_count++;
            cv::Mat img(item->Height, item->Width, CV_8UC3, item->ImageBuffer.get());
            putText(img, format("Rendered frame: %ld fps: %ld", frame_count, frame_count * 1000 / total_ms),
                Point(30, 30), 0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);
            int size = img.total() * img.elemSize();
            encoderCtrl->EncodeFrame(item->StreamId, img.data, size);
        }

}

static void InterruptHandler(int sig)
{
    if (sig == SIGINT)
    {
        captureCtrl->InterruptClose();
        LOG_DEBUG("captureCtrl exit \n");

        gDecodeQueue->Unlock();
        gExit = true;

        LOG_DEBUG("gDecodeQueue Unlock\n");
        
        usleep(200 * 1000);
        LOG_INFO("Exit sleep\n");
        encoderCtrl->InterruptClose();
        LOG_INFO("InterruptHandler exit\n");
        exit(SIGINT);
    }
    else
    {
        exit(-1);
    }
}

void InferenceThread(void *sol_conf)
{
    LOG_DEBUG("InferenceThread \n");

    SolutionConfiguration *solution_config = (SolutionConfiguration *)sol_conf;

    shared_ptr<ModelInference> shInference;
    shInference = std::make_shared<ModelInference>(solution_config->model_config->model_type);
    
    shInference->Initialization(*solution_config->model_config.get());
    shInference->SetNotifyResultCallBack(&OnNotifyResult);

    int ret = 0;
    auto start = chrono::steady_clock::now();
    app_start =  chrono::steady_clock::now();
    uint32_t frames = 0;
    do
    {
        shared_ptr<DetectionItem> item;
        ret = gDecodeQueue->Dequeue(item, 300);
        if (ret == 0)
        {
            frames += 1;
            auto start1 = chrono::steady_clock::now();

            shInference->Inference(item);
            auto end1 = chrono::steady_clock::now();
            auto costTime1 = chrono::duration_cast<chrono::milliseconds>(end1 - start1).count();
            LOG_INFO("Elapsed inference time in milliseconds: %ld ms\n",costTime1);
            OnNotifyResult(item, solution_config);
        }
        else
        {
            if (ret != 1)
            {
                LOG_ERROR("Error ret= %d", ret);
            }
            continue;
        }

    } while (!gExit);

    // Get
    auto remains = gDecodeQueue->GetRemainItems();
    LOG_INFO("Remain Items= %lu\n", remains.size());
    for (auto item : remains)
    {
        frames += 1;
        shInference->Inference(item);
        OnNotifyResult(item, solution_config);
    }

    shInference->UnInitialization();

    auto end = chrono::steady_clock::now();
    auto costTime = chrono::duration_cast<chrono::milliseconds>(end - start).count();

    LOG_INFO("Elapsed time in milliseconds: %ld ms \t Received Frames: %d \t Through rate: %ld \n", 
    costTime, frames, (frames * 1000)/costTime);
}

int main(int argc, char **argv)
{
    gExit = false;
    const char* inputFile;
    int opt = 0;
    while ((opt = getopt(argc, argv, ":hc:")) != -1)
    {
        switch (opt)
        {
            case 'h': std::cout
                        << "\nDESCRIPTION:\n"
                        << "------------\n"
                        << "Example application demonstrating how to run the use case\n"
                        << "using the SNPE C++ API.\n"
                        << "REQUIRED ARGUMENTS:\n"
                        << "-------------------\n"
                        << "  -c  <FILE>   Path to the config json file.\n"
                        << "Example: ./out/main -c ../data/config.json\n";
            case 'c':
                    inputFile = optarg;
                    LOG_INFO("Path to config file = %s \n", inputFile);
                    break;
            default:
                LOG_INFO("Invalid parameter specified. Please run sample with the -h flag to see required arguments\n");
                exit(0);
        };
    }
 
    Configuration::getInstance().LoadConfiguration(inputFile);

    const int MAX_QUEUE_DEPTH = 1;
    gDecodeQueue = make_shared<DecodeQueue>(MAX_QUEUE_DEPTH);

    encoderCtrl = make_shared<EncodeController>();
    captureCtrl = make_shared<CaptureController>();

    vector<string> selected_model;
    vector<SolutionConfiguration> solutions_config;

    for (auto i : Configuration::getInstance().solutionsconfig) {
         std::shared_ptr<SolutionConfiguration> config = i.second;
         if (config->Enable == true) {
            config->input_config = Configuration::getInstance().inputconfigs[config->input_config_name];
            if (config->input_config == NULL) {
                LOG_ERROR("NULL Input configuration for selected solution name = %s \n", config->solution_name.c_str());
                exit(1);
            }
            config->input_config->StreamNumber = i.first;
            config->model_config = Configuration::getInstance().modelsconfig[config->model_name];
            if (config->model_config == NULL) {
                LOG_ERROR("NULL Model configuration for selected solution name = %s \n", config->solution_name.c_str());
                exit(1);
            }
            solutions_config.emplace_back(*config);
            selected_model.push_back(config->model_name);
            captureCtrl->CreateCapture(config->input_config, gDecodeQueue);
            encoderCtrl->CreateEncoder(config);
        }
    }

    if (selected_model.size() == 0) {
        LOG_ERROR("Solution not enabled, Enable the desired solution in config.json file\n");
        exit(1);
    }

    std::thread inferThread[selected_model.size()];
    for (unsigned int i=0; i<selected_model.size(); i++) {
        inferThread[i] = std::thread(InferenceThread, (void *)(&solutions_config[i]));

    }

    signal(SIGINT, InterruptHandler);

    for (unsigned int i=0; i<solutions_config.size(); i++) {
       captureCtrl->StopAll();
    }

    gExit = true;
    gDecodeQueue->Unlock();

    for (unsigned int i=0; i<solutions_config.size(); i++) {
        inferThread[i].join();
        encoderCtrl->Stop();
    }

    return 0;
}
