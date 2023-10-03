#include <math.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "Configuration.h"
#include "CenterfaceSnpe.h"

namespace centerfacesnpe {

CENTERFACESnpe::CENTERFACESnpe() : m_snperuntime(nullptr) {

}

CENTERFACESnpe::~CENTERFACESnpe() {
    DeInitialize();
}

bool CENTERFACESnpe::Initialize(const ObjectDetectionSnpeConfig& config)
{
    m_snperuntime = std::move(std::unique_ptr<snperuntime::SNPERuntime>(new snperuntime::SNPERuntime()));

    m_inputLayers = config.inputLayers;
    m_outputLayers = config.outputLayers;
    m_outputTensors = config.outputTensors;
    m_labels = config.label_count;
    m_grids = config.grids;
    m_nmsThresh = config.nmsThresh;
    m_confThresh = config.confThresh;

    LOG_INFO("Runtime= %d \n", config.runtime);

    m_snperuntime->SetOutputLayers(m_outputLayers);
    if (!m_snperuntime->Initialize(config.model_path, config.runtime)) {
        LOG_ERROR("Can't init snpetask instance.\n");
        return false;
    }

    m_output = new float[m_grids * m_labels];
    m_isInit = true;
    return true;
}

bool CENTERFACESnpe::DeInitialize()
{
    if (m_isInit) {
        m_snperuntime->Deinitialize();
        m_snperuntime.reset(nullptr);
    }

    if (m_output) {
        delete[] m_output;
        m_output = nullptr;
    }

    m_isInit = false;
    return true;
}

bool CENTERFACESnpe::PreProcessInput(const cv::Mat& input_image)
{
    if (input_image.empty()) {
        LOG_ERROR("Invalid image!\n");
        return false;
    }

    auto inputShape = m_snperuntime->GetInputShape(m_inputLayers[0]);
    image_h = input_image.rows;
    image_w = input_image.cols;

    model_h = inputShape[1];
    model_w = inputShape[2];

    // Convert BGR Image to RGB Image
    cv::Mat image;
    cv::cvtColor(input_image, image, cv::COLOR_BGR2RGB); 
    cv::resize(image,image,cv::Size(model_h,model_w));

    float* tensor = NULL;
    if ( (tensor =  m_snperuntime->GetInputTensor(m_inputLayers[0])) == nullptr) {
        LOG_ERROR("Empty input tensor\n");
        return false;
    }

    cv::Mat input(model_h, model_w, CV_32FC3, tensor);
    image.convertTo(input, CV_32FC3, 1.0);   
    return true;
}

bool CENTERFACESnpe::Detect(shared_ptr<DetectionItem> &item)
{
    uint32_t imageWidth = item->Width; 
    uint32_t imageHeight = item->Height;
    uint8_t *img = item->ImageBuffer.get();

   cv::Mat image(cv::Size(imageWidth, imageHeight), CV_8UC3, img, cv::Mat::AUTO_STEP);
    PreProcessInput(image);

    if (!m_snperuntime->execute()) {
        LOG_ERROR("SNPERuntime execute failed.\n");
        return false;
    }
    
    PostProcess(item->Results);

    return true;
}

bool CENTERFACESnpe::PostProcess(std::vector<ObjectData> &results)
{
    std::vector<ObjectData> winList;
    std::vector<FaceInfo> face_info;

    auto outputShape = m_snperuntime->GetOutputShape(m_outputTensors[0]);
    float *heatmap_predOutput = m_snperuntime->GetOutputTensor(m_outputTensors[0]);

    int batchSize = outputShape[0];
    int height = outputShape[1];
    int width = outputShape[2];
    int channel = outputShape[3];

    int size = height*width;


    fea_h = height;
    fea_w = width;

    cv::Mat heatmap(cv::Size(width,height), CV_32FC1, heatmap_predOutput);

    float *scale_predOutput = m_snperuntime->GetOutputTensor(m_outputTensors[1]);

    //To deinterlace SNPE output
    float scale_[2*size];
    for(int i = 0;i<size;i++)
    {
            scale_[i] = scale_predOutput[i*2];
            scale_[size + i] = scale_predOutput[i*2 + 1];

    }

    for(int i = 0;i<2*size;i++)
    {
            scale_predOutput[i] = scale_[i];
    }


    cv::Mat scale(cv::Size(width,height), CV_32FC1, scale_predOutput);

    float *offset_predOutput = m_snperuntime->GetOutputTensor(m_outputTensors[2]);
    //To deinterlace SNPE output
    float offset_[2*size];
    for(int i = 0;i<size;i++)
    {
            offset_[i] = offset_predOutput[i*2];
            offset_[size + i] = offset_predOutput[i*2 + 1];
    }
    for(int i = 0;i<2*size;i++)
    {
            offset_predOutput[i] = offset_[i];
    }

    cv::Mat offset(cv::Size(width,height), CV_32FC1, offset_predOutput);

    decode(heatmap, scale, offset, face_info, 0.5, 0.3);
    squareBox(face_info);
 
    LOG_ERROR(" faces = %ld\n",face_info.size());
    for (size_t i = 0; i < face_info.size(); i++)
    {
            ObjectData face;
            face.face_info.x1 = face_info[i].x1;
            face.face_info.y1 = face_info[i].y1;
            face.face_info.x2 = face_info[i].x2;
            face.face_info.y2 = face_info[i].y2;
            face.face_info.score = face_info[i].score;

            for(int j = 0;j<10;j++)
                face.face_info.landmarks[j] = face_info[i].landmarks[j];

            winList.push_back(face);
            results.push_back(winList[i]);
    }
    
     return true;

}

void CENTERFACESnpe::nms(std::vector<FaceInfo>& input, std::vector<FaceInfo>& output, float nmsthreshold)
{
        std::sort(input.begin(), input.end(),
                [](const FaceInfo& a, const FaceInfo& b)
        {
                return a.score > b.score;
        });

        int box_num = input.size();

        std::vector<int> merged(box_num, 0);

        for (int i = 0; i < box_num; i++)
        {
                if (merged[i])
                        continue;

                output.push_back(input[i]);

                float h0 = input[i].y2 - input[i].y1 + 1;
                float w0 = input[i].x2 - input[i].x1 + 1;
                float area0 = h0 * w0;

                for (int j = i + 1; j < box_num; j++)
                {
                        if (merged[j])
                                continue;

                        float inner_x0 = input[i].x1 > input[j].x1 ? input[i].x1 : input[j].x1;
                        float inner_y0 = input[i].y1 > input[j].y1 ? input[i].y1 : input[j].y1;

                        float inner_x1 = input[i].x2 < input[j].x2 ? input[i].x2 : input[j].x2;  
                        float inner_y1 = input[i].y2 < input[j].y2 ? input[i].y2 : input[j].y2;

                        float inner_h = inner_y1 - inner_y0 + 1;
                        float inner_w = inner_x1 - inner_x0 + 1;

                        if (inner_h <= 0 || inner_w <= 0)
                                continue;

                        float inner_area = inner_h * inner_w;

                        float h1 = input[j].y2 - input[j].y1 + 1;
                        float w1 = input[j].x2 - input[j].x1 + 1;
                        float area1 = h1 * w1;
                        float score;
                        score = inner_area / (area0 + area1 - inner_area);

                        if (score > nmsthreshold)
                                merged[j] = 1;
                }

        }
}


void CENTERFACESnpe::decode(cv::Mat & heatmap, cv::Mat & scale, cv::Mat & offset, std::vector<FaceInfo>& faces, float scoreThresh, float nmsThresh)
{
        int spacial_size = fea_w*fea_h;
        dynamic_scale(image_w, image_h);

        float *heatmap_ = (float*)(heatmap.data);
        float *scale0 = (float*)(scale.data);
        float *scale1 = scale0+spacial_size;

        float *offset0 = (float*)(offset.data);
        float *offset1 = offset0 + spacial_size;

        std::vector<int> ids = getIds(heatmap_, fea_h, fea_w, scoreThresh);
        std::vector<FaceInfo> faces_tmp;
        for (size_t i = 0; i < ids.size()/2; i++) 
        {
                int id_h = ids[2*i];
                int id_w = ids[2*i+1];
                int index = id_h*fea_w + id_w;

                float s0 = std::exp(scale0[index]) * 4;
                float s1= std::exp(scale1[index]) * 4;
                float o0 = offset0[index];
                float o1= offset1[index];

                float x1 = std::max(0., (id_w + o1 + 0.5) * 4 - s1 / 2);
                float y1 = std::max(0., (id_h + o0 + 0.5) * 4 - s0 / 2);
                float x2 = 0, y2 = 0;

                x1 = std::min(x1, d_w);
                y1 = std::min(y1, d_h);
                x2 = std::min(x1 + s1, d_w);
                y2 = std::min(y1 + s0, d_h);

                FaceInfo facebox;
                facebox.x1 = x1;
                facebox.y1 = y1;
                facebox.x2 = x2;
                facebox.y2 = y2;
                facebox.score = heatmap_[index];
                faces_tmp.push_back(facebox);
        }

        nms(faces_tmp, faces, nmsThresh);
        
        for (size_t k = 0; k < faces.size(); k++) {
                faces[k].x1 *=d_scale_w*scale_w;
                faces[k].y1 *=d_scale_h*scale_h;
                faces[k].x2 *= d_scale_w*scale_w;
                faces[k].y2 *=d_scale_h*scale_h;
        }
}

void CENTERFACESnpe::dynamic_scale(float in_w, float in_h)
{
        d_h = (int)(std::ceil(in_h / 32) * 32);
        d_w = (int)(std::ceil(in_w / 32) * 32);

        d_scale_h = in_h/d_h ;
        d_scale_w = in_w/d_w ;

        scale_w = (float)d_w / (float)model_w;
        scale_h = (float)d_h / (float)model_h;

}

std::vector<int> CENTERFACESnpe::getIds(float *heatmap, int  h, int w, float thresh)
{
        std::vector<int> ids;
        for (int i = 0; i < h; i++) {
                for (int j = 0; j < w; j++) {
                        if (heatmap[i*w + j] > thresh) {
                                ids.push_back(i);
                                ids.push_back(j);
                        }
                }
        }
        return ids;
}
void CENTERFACESnpe::squareBox(std::vector<FaceInfo>& faces)
{
        float w=0, h=0, maxSize=0;
        float cenx, ceny;
        for (size_t i = 0; i < faces.size(); i++) {
                w = faces[i].x2 - faces[i].x1;
                h = faces[i].y2 - faces[i].y1;

                maxSize = std::max(w, h);
                cenx = faces[i].x1 + w / 2;
                ceny = faces[i].y1 + h / 2;
                faces[i].x1 = std::max(cenx - maxSize / 2, 0.f);                
                faces[i].y1 = std::max(ceny - maxSize / 2, 0.f);               
                faces[i].x2 = std::min(cenx + maxSize / 2, image_w - 1.f);  
                faces[i].y2 = std::min(ceny + maxSize / 2, image_h - 1.f); 
        }
}


} // namespace centerface
