#include "SNPERuntime.h"

namespace snperuntime{

SNPERuntime::SNPERuntime()
{
    static zdl::DlSystem::Version_t version = zdl::SNPE::SNPEFactory::getLibraryVersion();
    LOG_INFO("Using SNPE: '%s' \n", version.asString().c_str());
}

static size_t calcSizeFromDims(const zdl::DlSystem::Dimension* dims, size_t rank, size_t elementSize)
{
    if (rank == 0) return 0;
    size_t size = elementSize;
    while (rank--) {
        size *= *dims;
        dims++;
    }
    return size;
}

void CreateUserBuffer(zdl::DlSystem::UserBufferMap& userBufferMap,
                      std::unordered_map<std::string, std::vector<float>>& applicationBuffers,
                      std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers,
                      const zdl::DlSystem::TensorShape& bufferShape,
                      const char* name)
{
    size_t bufferElementSize = sizeof(float);

    // calculate stride based on buffer strides
    // Note: Strides = Number of bytes to advance to the next element in each dimension.
    // For example, if a float tensor of dimension 2x4x3 is tightly packed in a buffer of 96 bytes, then the strides would be (48,12,4)
    std::vector<size_t> strides(bufferShape.rank());
    strides[strides.size() - 1] = bufferElementSize;
    size_t stride = strides[strides.size() - 1];
    for (size_t i = bufferShape.rank() - 1; i > 0; i--)
    {
        stride *= bufferShape[i];
        strides[i - 1] = stride;
    }

    size_t bufSize = calcSizeFromDims(bufferShape.getDimensions(), bufferShape.rank(), bufferElementSize);

    // set the buffer encoding type
    zdl::DlSystem::UserBufferEncodingFloat userBufferEncodingFloat;
    // create user-backed storage to load input data onto it
    applicationBuffers.emplace(name, std::vector<float>(bufSize / bufferElementSize));
    // create SNPE user buffer from the user-backed buffer
    zdl::DlSystem::IUserBufferFactory& ubFactory = zdl::SNPE::SNPEFactory::getUserBufferFactory();
    snpeUserBackedBuffers.push_back(ubFactory.createUserBuffer((void*)applicationBuffers.at(name).data(),
                                                                bufSize,
                                                                strides,
                                                                &userBufferEncodingFloat));
    // add the user-backed buffer to the inputMap, which is later on fed to the network for execution
    userBufferMap.add(name, snpeUserBackedBuffers.back().get());
}

void SNPERuntime::setTargetRuntime(const runtime_t runtime) 
{
    switch (runtime) {
        case CPU:
            m_runtime = zdl::DlSystem::Runtime_t::CPU;
            break;
        case GPU:
            m_runtime = zdl::DlSystem::Runtime_t::GPU;
            break;
        case GPU_FLOAT16:
            m_runtime = zdl::DlSystem::Runtime_t::GPU_FLOAT16;
            break;
        case DSP:
            m_runtime = zdl::DlSystem::Runtime_t::DSP;
            break;
        case AIP:
            m_runtime = zdl::DlSystem::Runtime_t::AIP_FIXED8_TF;
            break;
        default:
            m_runtime = zdl::DlSystem::Runtime_t::CPU;
            break;
    }

    if (!zdl::SNPE::SNPEFactory::isRuntimeAvailable(m_runtime)) {
        LOG_ERROR("Selected runtime not present. Falling back to CPU.");
        m_runtime = zdl::DlSystem::Runtime_t::CPU;
    }
}

void SNPERuntime::setPerformanceProfile(const performance_t perfprofile)
{
    switch (perfprofile) {
        case BALANCED:
            m_profile = zdl::DlSystem::PerformanceProfile_t::BALANCED;
            break;
        case HIGH_PERFORMANCE:
            m_profile = zdl::DlSystem::PerformanceProfile_t::HIGH_PERFORMANCE;
            break;
        case POWER_SAVER:
            m_profile = zdl::DlSystem::PerformanceProfile_t::POWER_SAVER;
            break;
        case SUSTAINED_HIGH_PERFORMANCE:
            m_profile = zdl::DlSystem::PerformanceProfile_t::SUSTAINED_HIGH_PERFORMANCE;
            break;
        case BURST:
            m_profile = zdl::DlSystem::PerformanceProfile_t::BURST;
            break;
        case LOW_POWER_SAVER:
            m_profile = zdl::DlSystem::PerformanceProfile_t::LOW_POWER_SAVER;
            break;
        case HIGH_POWER_SAVER:
            m_profile = zdl::DlSystem::PerformanceProfile_t::HIGH_POWER_SAVER;
            break;
        case LOW_BALANCED:
            m_profile = zdl::DlSystem::PerformanceProfile_t::LOW_BALANCED;
            break;
        case SYSTEM_SETTINGS:
            m_profile = zdl::DlSystem::PerformanceProfile_t::SYSTEM_SETTINGS;
            break;
        default:
            m_profile = zdl::DlSystem::PerformanceProfile_t::BALANCED;
            break;
    }
    LOG_DEBUG("Choose performance: %d,  Set performance: %d \n", perfprofile, (int)m_profile);
}

bool SNPERuntime::Initialize(const std::string& dlc_path, const runtime_t runtime)
{
    setTargetRuntime(runtime);

    setPerformanceProfile(BURST);

    m_container = zdl::DlContainer::IDlContainer::open(dlc_path);

    zdl::SNPE::SNPEBuilder snpeBuilder(m_container.get());
    m_snpe = snpeBuilder.setOutputLayers(m_outputLayers)
       .setRuntimeProcessorOrder(m_runtime)
       .setPerformanceProfile(m_profile)
       .setUseUserSuppliedBuffers(true)
       .build();

    if (nullptr == m_snpe.get()) {
        const char* errStr = zdl::DlSystem::getLastErrorString();
        LOG_ERROR("SNPE build failed: {%s}", errStr);
        return false;
    }

    // get input tensor names of the network that need to be populated
    const auto& inputNamesOpt = m_snpe->getInputTensorNames();
    if (!inputNamesOpt) throw std::runtime_error("Error obtaining input tensor names");
    const zdl::DlSystem::StringList& inputNames = *inputNamesOpt;

    // create SNPE user buffers for each application storage buffer
    for (const char* name : inputNames) {
        // get attributes of buffer by name
        auto bufferAttributesOpt = m_snpe->getInputOutputBufferAttributes(name);
        if (!bufferAttributesOpt) {
            LOG_ERROR("Error obtaining attributes for input tensor: %s", name);
            return false;
        }

        const zdl::DlSystem::TensorShape& bufferShape = (*bufferAttributesOpt)->getDims();
        std::vector<size_t> tensorShape;
        for (size_t j = 0; j < bufferShape.rank(); j++) {
            tensorShape.push_back(bufferShape[j]);
        }
        m_inputShapes.emplace(name, tensorShape);

        CreateUserBuffer(m_inputUserBufferMap, m_applicationInputBuffers, m_inputUserBuffers, bufferShape, name);
    }

    // get output tensor names of the network that need to be populated
    const auto& outputNamesOpt = m_snpe->getOutputTensorNames();
    if (!outputNamesOpt) throw std::runtime_error("Error obtaining output tensor names");
    const zdl::DlSystem::StringList& outputNames = *outputNamesOpt;

    // create SNPE user buffers for each application storage buffer
    for (const char* name : outputNames) {
        // get attributes of buffer by name
        auto bufferAttributesOpt = m_snpe->getInputOutputBufferAttributes(name);
        if (!bufferAttributesOpt) {
            LOG_ERROR("Error obtaining attributes for input tensor: %s", name);
            return false;
        }

        const zdl::DlSystem::TensorShape& bufferShape = (*bufferAttributesOpt)->getDims();
        std::vector<size_t> tensorShape;
        for (size_t j = 0; j < bufferShape.rank(); j++) {
            tensorShape.push_back(bufferShape[j]);
        }
        m_outputShapes.emplace(name, tensorShape);

        CreateUserBuffer(m_outputUserBufferMap, m_applicationOutputBuffers, m_outputUserBuffers, bufferShape, name);
    }

    m_isInit = true;

    return true;
}

bool SNPERuntime::Deinitialize()
{
    if (nullptr != m_snpe) {
        m_snpe.reset(nullptr);
    }

    for (auto [k, v] : m_applicationInputBuffers) ClearVector(v);
    for (auto [k, v] : m_applicationOutputBuffers) ClearVector(v);
    return true;
}

bool SNPERuntime::SetOutputLayers(std::vector<std::string>& outputLayers)
{
    for (size_t i = 0; i < outputLayers.size(); i ++) {
        m_outputLayers.append(outputLayers[i].c_str());
        LOG_DEBUG("SetOutputLayers %s \n", outputLayers[i].c_str());
    }

    return true;
}

std::vector<size_t> SNPERuntime::GetInputShape(const std::string& name)
{
    if (IsInit()) {
        if (m_inputShapes.find(name) != m_inputShapes.end()) {
            return m_inputShapes.at(name);
        }
        LOG_ERROR("Can't find any input layer named %s", name.c_str());
        return {};
    } else {
        LOG_ERROR("GetInputShape Failed: SNPE Init Failed !!!");
        return {};
    }
}

std::vector<size_t> SNPERuntime::GetOutputShape(const std::string& name)
{
    if (IsInit()) {
        if (m_outputShapes.find(name) != m_outputShapes.end()) {
            return m_outputShapes.at(name);
        }
        LOG_ERROR("Can't find any ouput layer named %s", name.c_str());
        return {};
    } else {
        LOG_ERROR("GetOutputShape Failed: SNPE Init Failed !!!");
        return {};
    }
}

float* SNPERuntime::GetInputTensor(const std::string& name)
{
    if (IsInit()) {
        if (m_applicationInputBuffers.find(name) != m_applicationInputBuffers.end()) {
            return m_applicationInputBuffers.at(name).data();
        }
        LOG_ERROR("Can't find any input tensor named '%s' \n", name.c_str());
        return nullptr;
    } else {
        LOG_ERROR("GetInputTensor Failed: SNPE Init Failed !!!");
        return nullptr;
    }
}

float* SNPERuntime::GetOutputTensor(const std::string& name)
{
    if (IsInit()) {
        if (m_applicationOutputBuffers.find(name) != m_applicationOutputBuffers.end()) {
            return m_applicationOutputBuffers.at(name).data();
        }
        LOG_ERROR("Can't find any output tensor named '%s' \n", name.c_str());
        return nullptr;
    } else {
        LOG_ERROR("GetOutputTensor Failed: SNPE Init Failed !!!");
        return nullptr;
    }
}

bool SNPERuntime::execute() 
{
    if (!m_snpe->execute(m_inputUserBufferMap, m_outputUserBufferMap)) {
        LOG_ERROR("SNPE Task execute failed: %s", zdl::DlSystem::getLastErrorString());
        return false;
    }

    return true;
}

}   // namespace snperuntime