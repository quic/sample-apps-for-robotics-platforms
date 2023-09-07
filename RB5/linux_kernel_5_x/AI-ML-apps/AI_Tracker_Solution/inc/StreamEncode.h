#ifndef INC_STREAM_ENCODE_H
#define INC_STREAM_ENCODE_H

#include "DecodeQueue.h"
#include <iostream>
#include <string>
#include <memory>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <iostream>
#include <string>
#include <map>
#include <thread>

#include "Utils.h"
#include "Configuration.h"

using namespace std;
/* Structure to contain all our information, so we can pass it to callbacks */
typedef struct _EncodePipeline
{
    shared_ptr<GstElement> pipeline;
    shared_ptr<GstElement> appsrc;
    shared_ptr<GstElement> vidconv; 
    shared_ptr<GstElement> vtransform; 
    shared_ptr<GstElement> capsfilter; 
    shared_ptr<GstElement> videoscale;
    shared_ptr<GstElement> x265enc;
    shared_ptr<GstElement> h265parse;
    shared_ptr<GstElement> qtmux;
    shared_ptr<GstElement> filesink;
    shared_ptr<GstElement> waylandsink;
    shared_ptr<GstElement> videoconvert;
} EncodePipeline;

class StreamEncode{
public:
    StreamEncode()=default;
    ~StreamEncode()=default;
    int Initialization(string output_type, string output_path);
    void UnInitialization();
    void PushData(uint8_t *data, int len);
    int Loop();
    void Stop();
private:
    EncodePipeline data;
    shared_ptr<GstBus> bus=nullptr;
    bool terminate=false;
    string outputFile;
    int gst_wayland_pipeline_init(string output_type);
    int gst_filesink_pipeline_init(string output_type, string output_path);
}; 

class EncodeController
{
    public:
        void CreateEncoder(std::shared_ptr<SolutionConfiguration> sol_conf);
        void EncodeFrame(int streamId, uint8_t *pushData, int len);
        void EndOfStream(int streamId);
        void Stop();
        void InterruptClose();
    private:
        map<int,shared_ptr<StreamEncode>> encoders;
        vector<std::thread> threads;
};
#endif