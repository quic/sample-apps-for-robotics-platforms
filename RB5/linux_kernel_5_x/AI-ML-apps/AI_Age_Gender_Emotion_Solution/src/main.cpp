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
#include <condition_variable>

using namespace std;

shared_ptr<DecodeQueue> gDecodeQueue;
bool gExit = false;

shared_ptr<EncodeController> encoderCtrl;
shared_ptr<CaptureController> captureCtrl;

enum model_enum{face_detect, age,gender,emotion};
mutex mu;
condition_variable cond;
int current_model = face_detect;
shared_ptr<DetectionItem> item;
int ret;

static void OnNotifyResult(shared_ptr<DetectionItem> &item, SolutionConfiguration* solution_config)
{
    LOG_INFO("result size: %lu \n", item->Results.size());

    if (item->Results.size() > 0)
    {
        cv::Mat img(item->Height, item->Width, CV_8UC3, item->ImageBuffer.get());
        for (size_t j = 0; j < item->Results.size(); j++) {
            ObjectData result = item->Results[j];

            
	    if(result.face_info.x1 > 0 && result.face_info.y1 > 0 && result.face_info.x2 > 0 && result.face_info.y2 > 0)
            {

		    cv::rectangle(img, cv::Point(result.face_info.x1, result.face_info.y1), cv::Point(result.face_info.x2, result.face_info.y2), cv::Scalar(0, 255, 0), 2);
                    std::vector<std::string> texts = {result.age, result.gender, result.emotion};

              
                    std::vector<cv::Point> positions = {
                       cv::Point(result.face_info.x1-100, result.face_info.y1 - 10), 
                       cv::Point(result.face_info.x1+100, result.face_info.y1 - 10),
                       cv::Point(result.face_info.x1+250, result.face_info.y1 - 10) 
                   };

              for (size_t i = 0; i < texts.size(); i++) {
                 cv::putText(img, texts[i], positions[i], cv::FONT_HERSHEY_COMPLEX, 0.8, cv::Scalar(255, 0, 0), 2, 0.3);
                }
	                
		 
	    }
        }

            int size = img.total() * img.elemSize();
            encoderCtrl->EncodeFrame(item->StreamId, img.data, size);
    }
    else
    {
        cv::Mat img(item->Height, item->Width, CV_8UC3, item->ImageBuffer.get());
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

void InferenceThread(void *sol_conf,int model_id, int size)
{
    LOG_DEBUG("InferenceThread \n");

    SolutionConfiguration *solution_config = (SolutionConfiguration *)sol_conf;
    shared_ptr<ModelInference> shInference;
    shInference = std::make_shared<ModelInference>(solution_config->model_config[model_id]->model_type);
    shInference->Initialization(*solution_config->model_config[model_id].get());
    shInference->SetNotifyResultCallBack(&OnNotifyResult);
    
    auto start = chrono::steady_clock::now();
    uint32_t frames = 0;
    do
    {  
	unique_lock<mutex> lock(mu);
	cond.wait(lock, [&](){
		return model_id == current_model;
	}
        );
        if(current_model == face_detect)
        {
            ret = gDecodeQueue->Dequeue(item, 300);
        }
        if (ret == 0)
        {
            frames += 1;
            auto start1 = chrono::steady_clock::now();
            shInference->Inference(item);
            auto end1 = chrono::steady_clock::now();
            auto costTime1 = chrono::duration_cast<chrono::milliseconds>(end1 - start1).count();
            LOG_INFO("Elapsed inference time in milliseconds : %ld ms for %s\n",costTime1, solution_config->model_config[model_id]->model_type.c_str());
            
            if(current_model == (size -1))
                current_model = face_detect;
            else
                current_model++;
            
            if( model_id  == (size -1) )
            {
                OnNotifyResult(item, solution_config);
            }

            lock.unlock();
            cond.notify_all();
               
        }
        else
        {
            if (ret != 1)
            {
                LOG_ERROR("Error ret= %d\n", ret);
            }
            continue;
        }

    } while (!gExit);

    auto remains = gDecodeQueue->GetRemainItems();
    LOG_INFO("Remain Items= %lu\n", remains.size());
    for (auto remain_item : remains)
    {
        frames += 1;
        unique_lock<mutex> lock(mu);
        cond.wait(lock, [&](){
            return model_id == current_model;
            }
        );
        auto start1 = chrono::steady_clock::now();
        shInference->Inference(remain_item);
        auto end1 = chrono::steady_clock::now();
        auto costTime1 = chrono::duration_cast<chrono::milliseconds>(end1 - start1).count();
        LOG_INFO("Elapsed inference time in milliseconds : %ld ms for %s\n",costTime1, solution_config->model_config[model_id]->model_type.c_str());
        
        if(current_model == (size -1))
            current_model = face_detect;
        else
            current_model++;
        
        if( model_id  == (size -1) )
        {
            OnNotifyResult(remain_item, solution_config);
        }

        lock.unlock();
        cond.notify_all();
    }

    shInference->UnInitialization();

    auto end = chrono::steady_clock::now();
    auto costTime = chrono::duration_cast<chrono::milliseconds>(end - start).count();

    LOG_INFO("Elapsed time in milliseconds: %ld ms \t Received Frames: %d \t Through rate: %ld for %s\n", costTime, frames, (frames * 1000)/costTime,  solution_config->model_config[model_id]->model_type.c_str());
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

    const int MAX_QUEUE_DEPTH = 5;
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

            for(long unsigned int j=0; j<config->model_name.size(); j++)
            {    
                config->model_config.push_back( Configuration::getInstance().modelsconfig[config->model_name[j]]);
                if (config->model_config[j] == NULL) {
                    LOG_ERROR("NULL Model configuration for selected solution name = %s \n", config->solution_name.c_str());
                    exit(1);
                }
                selected_model.push_back(config->model_name[j]);
            }
            solutions_config.emplace_back(*config);
            captureCtrl->CreateCapture(config->input_config, gDecodeQueue);
            encoderCtrl->CreateEncoder(config);
        }
    }

    if (selected_model.size() == 0) {
        LOG_ERROR("Solution not enabled, Enable the desired solution in config.json file\n");
        exit(1);
    }

    signal(SIGINT, InterruptHandler);

    std::thread inferThread[selected_model.size()];
    for (unsigned int i=0; i<selected_model.size(); i++) {
        inferThread[i] = std::thread(InferenceThread, (void *)(&solutions_config[0]),i,selected_model.size());
    }

    for (unsigned int i=0; i<solutions_config.size(); i++) {
       captureCtrl->StopAll();
    }

    gExit = true;

    for (unsigned int i=0; i<selected_model.size(); i++) {
        inferThread[i].join();
    }
        
    gDecodeQueue->Unlock();

    encoderCtrl->Stop();

    return 0;
}
