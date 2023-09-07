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

/**
 * Bufferqueue to decode frames from camera 
*/
shared_ptr<DecodeQueue> gDecodeQueue;

/**
 * To check for gstreamer exit
*/
bool gExit = false;

/**
 * To encode frames for preview/file
*/
shared_ptr<EncodeController> encoderCtrl;

/**
 * To create object for frame capture
*/
shared_ptr<CaptureController> captureCtrl;

/** @brief  Call encoder control to save the output
 * @param solution_config Solution information parsed from config.json
 */
static void OnNotifyResult(shared_ptr<DetectionItem> &item, SolutionConfiguration* solution_config)
{
    cv::Mat img(item->Height, item->Width, CV_8UC3, item->ImageBuffer.get());
    int size = img.total() * img.elemSize();
    encoderCtrl->EncodeFrame(item->StreamId, img.data, size);
}

/** @brief To handle user interrupt
 * @param sig signal value of the interrupt
*/
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
        exit(0);
    }
    else
    {
        exit(1);
    }
}

/** @brief To intialize and configure the runtime based on the solution
 * @param sol_conf contains information about the solution 
*/
void InferenceThread(void *sol_conf)
{
    LOG_DEBUG("InferenceThread \n");

    SolutionConfiguration *solution_config = (SolutionConfiguration *)sol_conf;

    /**
     * TO initialize layers and buffers based on model type
    */
    shared_ptr<ModelInference> shInference;
    shInference = std::make_shared<ModelInference>(solution_config->model_config->model_type);
    
    /**
     * To set callback function
    */
    shInference->Initialization(*solution_config->model_config.get());
    shInference->SetNotifyResultCallBack(&OnNotifyResult);

    int ret = 0;
    auto start = chrono::steady_clock::now();
    uint32_t frames = 0;
    
    /**
     * Run the loop until stream ends or interrupt from user
    */
    do
    {
        auto start1 = chrono::steady_clock::now();
        shared_ptr<DetectionItem> item;
        ret = gDecodeQueue->Dequeue(item, 300);
        /**
         * Check if Dequeue is successful
        */
        if (ret == 0)
        {
            frames += 1;
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

    /**
     * To infer on the remaining pending items if exited before completion
    */
    auto remains = gDecodeQueue->GetRemainItems();
    LOG_INFO("Remain Items= %lu\n", remains.size());
    for (auto item : remains)
    {
        frames += 1;
        shInference->Inference(item);
        OnNotifyResult(item, solution_config);
    }

    /**
     * To deallocate the bufferes and runtime
    */
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
 
    /**
     * To parse input,model and solution config from inputFile
    */
    Configuration::getInstance().LoadConfiguration(inputFile);

    const int MAX_QUEUE_DEPTH = 1;
    gDecodeQueue = make_shared<DecodeQueue>(MAX_QUEUE_DEPTH);

    encoderCtrl = make_shared<EncodeController>();
    captureCtrl = make_shared<CaptureController>();

    vector<string> selected_model;
    vector<SolutionConfiguration> solutions_config;

    /**
     * To intialize each enabled solution
    */
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

    /**
     * To stop gstreamer pipeline
    */
    for (unsigned int i=0; i<solutions_config.size(); i++) {
       captureCtrl->StopAll();
    }

    /**
     * To stop inference
    */
    gExit = true;
    gDecodeQueue->Unlock();

    /**
     * Wait for all the threads to finish
    */
    for (unsigned int i=0; i<solutions_config.size(); i++) {
        inferThread[i].join();
        encoderCtrl->Stop();
    }

    return 0;
}
