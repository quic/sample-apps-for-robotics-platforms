#include "ModelInference.h"
#include "Configuration.h"
#include "CenterfaceSnpe.h"
#include "GooglenetSnpe.h"
#include "GendernetSnpe.h"
#include "FERplusSnpe.h"

using namespace std;
using namespace cv;
using namespace dnn;
using namespace centerfacesnpe;
using namespace googlenetsnpe;
using namespace gendernetsnpe;
using namespace FERplussnpe;



ModelInference::ModelInference()
{
        Impl = new CENTERFACESnpe();
}

ModelInference::ModelInference(const string model_type)
{
    std::cout<<"model inference---------------------------  "<<model_type<<std::endl;
 
     if(0 == model_type.compare("centerface"))
    {
        Impl = new CENTERFACESnpe();
        model = CENTERFACE;
    }
    else if(0 == model_type.compare("googlenet"))
    {
        Impl = new GOOGLENETSnpe(); 
        model = GOOGLENET;
    }
    else if(0 == model_type.compare("gendernet"))
    {
        Impl = new GENDERNETSnpe(); 
        model = GENDERNET;
    }
    else if(0 == model_type.compare("FERplus"))
    {
        Impl = new FERPLUSSnpe(); 
        model = FERPLUS;
    }
    else
        LOG_ERROR("Model implementation not found\n");  

    LOG_INFO("Initialized model = %s \n", model_type.c_str());

}

ModelInference::~ModelInference()
{

    if (nullptr != Impl) {

        switch(model)
        {
            case CENTERFACE:delete static_cast<CENTERFACESnpe*>(Impl);
                    break;
            case GOOGLENET:delete static_cast<GOOGLENETSnpe*>(Impl);
                    break;
            case GENDERNET:delete static_cast<GENDERNETSnpe*>(Impl);
                    break;
            case FERPLUS:delete static_cast<FERPLUSSnpe*>(Impl);
                    break;
            
        }
        Impl = nullptr;
    }
}

int ModelInference::Inference(shared_ptr<DetectionItem> &item)
{
    if (nullptr != Impl && IsInitialized()) {
        switch(model)
        {
            
                    
            case CENTERFACE: return static_cast<CENTERFACESnpe*>(Impl)->Detect(item);
                    
            case GOOGLENET: return static_cast<GOOGLENETSnpe*>(Impl)->Detect(item);

            case GENDERNET: return static_cast<GENDERNETSnpe*>(Impl)->Detect(item);

            case FERPLUS: return static_cast<FERPLUSSnpe*>(Impl)->Detect(item);
            
        }
    }
    else {
        LOG_ERROR("ObjectDetection::Detect failed caused by incompleted initialization!\n");
        return false;
    }
}

int ModelInference::Initialization(const ObjectDetectionSnpeConfig& config)
{
    if (IsInitialized()) {
        switch(model)
        {
            case CENTERFACE:return static_cast<CENTERFACESnpe*>(Impl)->DeInitialize() && static_cast<CENTERFACESnpe*>(Impl)->Initialize(config);
            case GOOGLENET:return static_cast<GOOGLENETSnpe*>(Impl)->DeInitialize() && static_cast<GOOGLENETSnpe*>(Impl)->Initialize(config);
            case GENDERNET:return static_cast<GENDERNETSnpe*>(Impl)->DeInitialize() && static_cast<GENDERNETSnpe*>(Impl)->Initialize(config);
            case FERPLUS:return static_cast<FERPLUSSnpe*>(Impl)->DeInitialize() && static_cast<FERPLUSSnpe*>(Impl)->Initialize(config);
        }
        
    } else {

        switch(model)
        {
           
            case CENTERFACE:return static_cast<CENTERFACESnpe*>(Impl)->Initialize(config);
            case GOOGLENET:return static_cast<GOOGLENETSnpe*>(Impl)->Initialize(config);
            case GENDERNET:return static_cast<GENDERNETSnpe*>(Impl)->Initialize(config);
            case FERPLUS:return static_cast<FERPLUSSnpe*>(Impl)->Initialize(config);

        }
    }
}

bool ModelInference::UnInitialization()
{
    if (nullptr != Impl && IsInitialized()) {
        switch(model)
        {
            case CENTERFACE:return static_cast<CENTERFACESnpe*>(Impl)->DeInitialize();
            case GOOGLENET:return static_cast<GOOGLENETSnpe*>(Impl)->DeInitialize();
            case GENDERNET:return static_cast<GENDERNETSnpe*>(Impl)->DeInitialize();
            case FERPLUS:return static_cast<FERPLUSSnpe*>(Impl)->DeInitialize();
        }
    } else {
        LOG_ERROR("ObjectDetection: deinit failed!\n");
        return false;
    }
}

bool ModelInference::IsInitialized()
{
    switch(model)
    {
        case CENTERFACE:return static_cast<CENTERFACESnpe*>(Impl)->IsInitialized();
        case GOOGLENET:return static_cast<GOOGLENETSnpe*>(Impl)->IsInitialized();
        case GENDERNET:return static_cast<GENDERNETSnpe*>(Impl)->IsInitialized();
        case FERPLUS:return static_cast<FERPLUSSnpe*>(Impl)->IsInitialized();
    }
}

void ModelInference::SetNotifyResultCallBack(NotifyDetectionResult callback)
{
  if (callback != nullptr)
  {
    notifyDetectionHandler = callback;
  }
}
