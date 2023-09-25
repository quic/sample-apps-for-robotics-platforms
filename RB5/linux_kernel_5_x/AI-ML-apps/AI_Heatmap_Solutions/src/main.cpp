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
 * pixel value > MAX_PIXEL_VALUE will be treated as red zone 
*/ 
#define MAX_PIXEL_VALUE 1.9833333333333334 

/**
 *  To downscale frame rows and cols by HEATMAP_SIZE
*/
#define HEATMAP_SIZE 80  

/**
 * To decode frames from gstreamer 
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



/** @brief  To overlay bounding boxes or other predctions on the frame
 * @param item Bounding boxes stored in Results
 * @param solution_config Solution information parsed from config.json
 */
static void OnNotifyResult(shared_ptr<DetectionItem> &item, SolutionConfiguration* solution_config)
{

    LOG_INFO("result size: %lu \n", item->Results.size());
    /**
     * To check for number of detected objects
    */
    if (item->Results.size() > 0)
    {
        /**
         * Creating Mat img from gstreamer buffer
        */
        cv::Mat img(item->Height, item->Width, CV_8UC3, item->ImageBuffer.get());
        /**
         * Creating downscaled heat_matrix to map center of bounding box   
        */
        cv::Mat heat_matrix(item->Height/HEATMAP_SIZE,item->Width/HEATMAP_SIZE,CV_64FC1,Scalar(0));

        for (size_t j = 0; j < item->Results.size(); j++)
        {
            ObjectData result = item->Results[j];
            /**
             * To calculate the center of the bounding box
            */ 
            int x1 = result.bbox.x;
            int y1 = result.bbox.y;
            int x2 = result.bbox.x + result.bbox.width;
            int y2 = result.bbox.y + result.bbox.height;

            /**
             * To calculate center of the bounding box from top-left(x1,y1) and bottom-right(x2,y2)
            */
            int col = ((x1 + x2)/2) / HEATMAP_SIZE;
            int row = ((y1 + y2)/2) / HEATMAP_SIZE;

            /**
             * Adding weight to each pixel
            */ 
            heat_matrix.at<double>(row,col) += 1;
        }
        /**
         * Converting single channel to 3 channels 
        */
        heat_matrix.convertTo(heat_matrix,CV_64FC3);

        /**
         * Resize heat_matrix(downscaled by HEATMAP_SIZE) to frame dimension 
        */
        cv::Mat resize_heat_matrix;
        cv::resize(heat_matrix, resize_heat_matrix,cv::Size(item->Width,item->Height));
        
        /**
         * Normalizing with MAX_PIXEL_VALUE
        */
        resize_heat_matrix.convertTo(resize_heat_matrix,CV_64FC3,1/MAX_PIXEL_VALUE);

        /**
         * Scaling pixel values from 0 to 255
        */
        resize_heat_matrix.convertTo(resize_heat_matrix,CV_64FC3,255);

        /**
         * Converting from 64 bit to 8 bit
        */
        resize_heat_matrix.convertTo(resize_heat_matrix,CV_8UC3);

        /**
         * Generating heatmap
        */ 
        cv::Mat image_heat;
        cv::applyColorMap(resize_heat_matrix, image_heat, cv::COLORMAP_JET);

        /**
         * Applying weight to the heatmap
        */
        double alpha = 0.8;
        cv::addWeighted(image_heat, alpha, img, 1 - alpha, 0.0, img);

        int size = img.total() * img.elemSize();
        /**
         * To display or save to file
        */
        encoderCtrl->EncodeFrame(item->StreamId, img.data, size);
    }
    /**
     * If there are no detections in the frame
    */
    else
    {
        /**
         * To maintain constant overlay in the display
        */
        cv::Mat img(item->Height, item->Width, CV_8UC3, item->ImageBuffer.get());
        cv::Mat heat_matrix(item->Height,item->Width,CV_8UC3,Scalar(255*0.5,0,0));
        /**
         * Applying weight to the frame
        */
        double alpha = 0.6;
        cv::addWeighted(heat_matrix, alpha, img, 1 - alpha, 0.0, img);
        int size = img.total() * img.elemSize();
        /**
         * To display or save to file
        */
        encoderCtrl->EncodeFrame(item->StreamId, img.data, size);
    }

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
        exit(SIGINT);
    }
    else
    {
        exit(-1);
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
    
    shInference->Initialization(*solution_config->model_config.get());
    /**
     * To set callback function
    */
    shInference->SetNotifyResultCallBack(&OnNotifyResult);

    int ret = 0;
    auto start = chrono::steady_clock::now();
    uint32_t frames = 0;
    /**
     * Run the loop until stream ends or interrupt from user
    */
    do
    {
        shared_ptr<DetectionItem> item;
        /**
         * To retrieve gstreamer buffer from queue
        */
        ret = gDecodeQueue->Dequeue(item, 300);
        /**
         * Check if Dequeue is successful
        */
        if (ret == 0)
        {
            frames += 1;
            auto start1 = chrono::steady_clock::now();
            /**
             * start inferencing on the image buffer 
            */
            shInference->Inference(item);
            auto end1 = chrono::steady_clock::now();
            auto costTime1 = chrono::duration_cast<chrono::milliseconds>(end1 - start1).count();
            LOG_INFO("Elapsed inference time in milliseconds: %ld ms\n",costTime1);
            /**
             * To overlay heatmap on frame
            */
            OnNotifyResult(item, solution_config);
        }
        /**
         * If there are no items in the queue
        */
        else
        {
            if (ret != 1)
            {
                LOG_ERROR("Error ret= %d\n", ret);
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

/** @brief Execution starts from here  
 * @param argc for total argument count
 * @param argv arguments to be passed
*/

int main(int argc, char **argv)
{
    gExit = false;
    /**
     * To store config file name passed in argument
    */
    const char* inputFile; 
    int opt = 0;
    /**
     * Check if 'h' or 'c' passed in argument
    */
    while ((opt = getopt(argc, argv, ":hc:")) != EOF) 
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
                        break;
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

    /**
     * To maintain maximum number of items in the queue
    */
    const int MAX_QUEUE_DEPTH = 1;
    gDecodeQueue = make_shared<DecodeQueue>(MAX_QUEUE_DEPTH);
    encoderCtrl = make_shared<EncodeController>();
    captureCtrl = make_shared<CaptureController>();

    /**
     * To access enabled soultion model
    */
    vector<string> selected_model;
    /**
     * To access enabled solution configuration
    */
    vector<SolutionConfiguration> solutions_config;
    /**
     * To intialize each enabled solution
    */
    for (auto i : Configuration::getInstance().solutionsconfig) { 
        /**
         * To access solution configuration
        */
         std::shared_ptr<SolutionConfiguration> config = i.second;
         /**
          * To check if solution is enabled
         */
         if (config->Enable == true) {
            /**
             * To access the input configuration 
            */
            config->input_config = Configuration::getInstance().inputconfigs[config->input_config_name];
            if (config->input_config == NULL) {
                LOG_ERROR("NULL Input configuration for selected solution name = %s \n", config->solution_name.c_str());
                exit(1);
            }
            config->input_config->StreamNumber = i.first;
            /**
             * To access the model configuration 
            */
            config->model_config = Configuration::getInstance().modelsconfig[config->model_name];
            if (config->model_config == NULL) {
                LOG_ERROR("NULL Model configuration for selected solution name = %s \n", config->solution_name.c_str());
                exit(1);
            }
            /**
             * To store the enabled solution configuration
            */
            solutions_config.emplace_back(*config);
            /**
             * Append the selected models
            */
            selected_model.push_back(config->model_name);
            /**
             * Intialze gstreamer pipeline to capture
            */
            captureCtrl->CreateCapture(config->input_config, gDecodeQueue);
            /**
             * Intialze encoder to display or save frame
            */
            encoderCtrl->CreateEncoder(config);
        }
    }
    /**
     * Check if any solution is enabled
    */
    if (selected_model.size() == 0) {
        LOG_ERROR("Solution not enabled, Enable the desired solution in config.json file\n");
        exit(1);
    }
    /**
     * To handle user interrupt signal with  InterruptHandler function
    */
    signal(SIGINT, InterruptHandler); 
    /**
     * Create individual thread for each solution
    */
    std::thread inferThread[selected_model.size()]; 
    for (unsigned int i=0; i<selected_model.size(); i++) {
        inferThread[i] = std::thread(InferenceThread, (void *)(&solutions_config[i]));

    }

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
