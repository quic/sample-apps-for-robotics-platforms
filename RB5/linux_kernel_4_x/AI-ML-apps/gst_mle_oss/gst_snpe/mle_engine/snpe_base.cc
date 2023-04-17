/*
* Copyright (c) 2020, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <vector>
#include <memory>

#include "snpe_base.h"

namespace mle {

SNPEBase::SNPEBase(MLConfig &config) : MLEngine(config) {
  ConfigureRuntime(config);
  config_.io_type = config.io_type;
  config_.output_layers = config.output_layers;
  need_labels_ = true;
}

int32_t SNPEBase::ConfigureRuntime(MLConfig &config) {
  switch (config.runtime) {
    case RuntimeType::DSP: {
      if (zdl::SNPE::SNPEFactory::isRuntimeAvailable(
          zdl::DlSystem::Runtime_t::DSP)) {
        runtime_ = zdl::DlSystem::Runtime_t::DSP;
        MLE_LOGI("DSP runtime selected");
      } else {
        runtime_ = zdl::DlSystem::Runtime_t::CPU;
        MLE_LOGI("CPU runtime selected, but DSP was configured");
      }
      break;
    }
    case RuntimeType::GPU: {
      if (zdl::SNPE::SNPEFactory::isRuntimeAvailable(
          zdl::DlSystem::Runtime_t::GPU)) {
        runtime_ = zdl::DlSystem::Runtime_t::GPU;
        MLE_LOGI("GPU runtime selected");
      } else {
        runtime_ = zdl::DlSystem::Runtime_t::CPU;
        MLE_LOGI("CPU runtime selected, but GPU was configured");
      }
      break;
    }
    case RuntimeType::AIP: {
      if (zdl::SNPE::SNPEFactory::isRuntimeAvailable(
          zdl::DlSystem::Runtime_t::AIP_FIXED8_TF)) {
        runtime_ = zdl::DlSystem::Runtime_t::AIP_FIXED8_TF;
        MLE_LOGI("AIP runtime selected");
      } else {
        runtime_ = zdl::DlSystem::Runtime_t::CPU;
        MLE_LOGI("CPU runtime selected, but AIP was configured");
      }
      break;
    }
    case RuntimeType::CPU: {
      runtime_ = zdl::DlSystem::Runtime_t::CPU;
      MLE_LOGI("CPU runtime selected");
      break;
    }
  }
  return MLE_OK;
}

int32_t SNPEBase::ConfigureDimensions() {
  zdl::DlSystem::Optional <zdl::DlSystem::StringList> names_opt;
  names_opt = snpe_params_.snpe->getInputTensorNames();
  const zdl::DlSystem::StringList& names = *names_opt;
  const char * name = names.at(0);
  auto uba_opt = snpe_params_.snpe->getInputOutputBufferAttributes(name);
  const zdl::DlSystem::TensorShape& buffer_shape = (*uba_opt)->getDims();
  const zdl::DlSystem::Dimension* dims = buffer_shape.getDimensions();

  engine_input_params_.height = dims[1];
  engine_input_params_.width = dims[2];

  // Obtain input and result layer names
  snpe_params_.input_layer = static_cast<std::string>(names.at(0));

  zdl::DlSystem::Optional <zdl::DlSystem::StringList> out_names;
  out_names = snpe_params_.snpe->getOutputTensorNames();
  const zdl::DlSystem::StringList& result_layers = *out_names;

  for (uint32_t i = 0; i < result_layers.size(); i++) {
    snpe_params_.result_layers.push_back(result_layers.at(i));
  }
  return MLE_OK;
}

std::unique_ptr<zdl::SNPE::SNPE> SNPEBase::SetBuilderOptions() {
  MLE_LOGI("%s: Enter", __func__);
  std::unique_ptr <zdl::SNPE::SNPE> snpe;
  zdl::SNPE::SNPEBuilder snpeBuilder(snpe_params_.container.get());
  zdl::DlSystem::StringList output_layers;

  for (size_t i = 0; i < config_.output_layers.size(); i++) {
    output_layers.append(config_.output_layers[i].c_str());
  }

  if (config_.io_type == NetworkIO::kUserBuffer) {
    snpe =
        snpeBuilder.setOutputLayers(output_layers).setRuntimeProcessor(runtime_)
            .setUseUserSuppliedBuffers(true).setCPUFallbackMode(true).build();
  } else if (config_.io_type == NetworkIO::kITensor) {
    snpe =
        snpeBuilder.setOutputLayers(output_layers).setRuntimeProcessor(runtime_)
            .setUseUserSuppliedBuffers(false).setCPUFallbackMode(true).build();
  } else {
    MLE_LOGE("%s: Invalid Network IO value", __func__);
    throw std::runtime_error("Invalid Network IO value");
  }

  MLE_LOGI("%s: Exit", __func__);
  return snpe;
}

int32_t SNPEBase::LoadModel(std::string& model_path) {
  int32_t res = MLE_OK;
  snpe_params_.container = LoadContainerFromFile(model_path);
  if (nullptr == snpe_params_.container) {
    PrintErrorStringAndExit();
    res = MLE_FAIL;
  }
  return res;
}

std::unique_ptr<zdl::DlContainer::IDlContainer> SNPEBase::LoadContainerFromFile(
    std::string container_path) {
  std::unique_ptr<zdl::DlContainer::IDlContainer> container;
  container = zdl::DlContainer::IDlContainer::open(container_path);
  if (nullptr == container) {
    MLE_LOGE("%s: Container loading failed", __func__);
    return nullptr;
  }

  return container;
}

int32_t SNPEBase::PopulateMap(BufferType type) {
  int32_t result = MLE_OK;
  zdl::DlSystem::Optional <zdl::DlSystem::StringList> names_opt;

  switch (type) {
    case BufferType::kInput:
      names_opt = snpe_params_.snpe->getInputTensorNames();
      break;
    case BufferType::kOutput:
      names_opt = snpe_params_.snpe->getOutputTensorNames();
      break;
    default:
      MLE_LOGE("Error obtaining tensor names");
      throw std::runtime_error("Error obtaining tensor names");
  }

  const zdl::DlSystem::StringList& names = *names_opt;
  for (const char *name : names) {
    if (config_.io_type == NetworkIO::kUserBuffer) {
      result = CreateUserBuffer(type, name);
    } else if (config_.io_type == NetworkIO::kITensor) {
      result = CreateTensor(type, name);
    } else {
      MLE_LOGE("Invalid Network IO value %d", static_cast<int32_t>(config_.io_type));
      result = MLE_FAIL;
    }

    if (MLE_OK != result) {
      break;
    }
  }
  return result;
}

int32_t SNPEBase::CreateUserBuffer(BufferType type, const char * name) {
  zdl::DlSystem::IUserBufferFactory& ub_factory =
      zdl::SNPE::SNPEFactory::getUserBufferFactory();

  auto uba_opt = snpe_params_.snpe->getInputOutputBufferAttributes(name);
  if (!uba_opt) {
    throw std::runtime_error(
        std::string("Error obtaining attributes for tensor ") + name);
  }

  auto m_encoding = (*uba_opt)->getEncoding();
  auto enc_type = (*uba_opt)->getEncodingType();
  MLE_LOGI("Encoding type is %d", (int)enc_type);
  const zdl::DlSystem::TensorShape& buffer_shape = (*uba_opt)->getDims();

  size_t elem_size = (*uba_opt)->getElementSize();
  MLE_LOGI("Bufer type %d elements size in bytes: %zu", (int)type, elem_size);

  size_t buf_size = CalculateSizeFromDims(buffer_shape.rank(),
                                          buffer_shape.getDimensions(),
                                          elem_size);

  auto *heap_map = &snpe_params_.in_heap_map;
  auto *ub_map = &snpe_params_.input_ub_map;
  if (type == BufferType::kOutput) {
    heap_map = &snpe_params_.out_heap_map;
    ub_map = &snpe_params_.output_ub_map;
  }

  heap_map->emplace(name, std::vector<float>(buf_size / elem_size));

  snpe_params_.ub_list.push_back(ub_factory.createUserBuffer(
      heap_map->at(name).data(), buf_size,
      GetStrides((*uba_opt)->getDims(), elem_size), m_encoding));
  ub_map->add(name, snpe_params_.ub_list.back().get());

  return MLE_OK;
}

int32_t SNPEBase::CreateTensor(BufferType type, const char* name) {
  zdl::DlSystem::ITensorFactory& tensor_factory =
      zdl::SNPE::SNPEFactory::getTensorFactory();

  auto tensor_opt = snpe_params_.snpe->getInputOutputBufferAttributes(name);
  if (!tensor_opt) {
    throw std::runtime_error(
        std::string("Error obtaining attributes for tensor ") + name);
  }
  const zdl::DlSystem::TensorShape& tensor_shape = (*tensor_opt)->getDims();

  size_t elem_size = (*tensor_opt)->getElementSize();
  MLE_LOGI("Bufer type %d elements size in bytes: %zu", (int)type, elem_size);

  size_t buf_size = CalculateSizeFromDims(tensor_shape.rank(),
                                          tensor_shape.getDimensions(),
                                          elem_size);
  auto *heap_map = &snpe_params_.in_heap_map;
  auto *tensor_map = &snpe_params_.input_tensor_map;
  if (type == BufferType::kOutput) {
    heap_map = &snpe_params_.out_heap_map;
    tensor_map = &snpe_params_.output_tensor_map;
  }

  heap_map->emplace(name, std::vector<float>(buf_size / elem_size));
  snpe_params_.tensor_list.push_back(tensor_factory.createTensor(tensor_shape));
  tensor_map->add(name, snpe_params_.tensor_list.back().get());

  return MLE_OK;
}

size_t SNPEBase::CalculateSizeFromDims(const size_t rank,
                                       const zdl::DlSystem::Dimension* dims,
                                       const size_t& element_size) {
  if (0 == rank) {
    return 0;
  }
  size_t size = element_size;
  for (size_t i = 0; i < rank; i++) {
    size *= dims[i];
  }
  return size;
}

std::vector<size_t> SNPEBase::GetStrides(zdl::DlSystem::TensorShape dims,
                                         const size_t& element_size) {
  std::vector<size_t> strides(dims.rank());
  strides[strides.size() - 1] = element_size;
  size_t stride = strides[strides.size() - 1];

  for (size_t i = dims.rank() - 1; i > 0; i--) {
    stride *= dims[i];
    strides[i - 1] = stride;
  }

  return strides;
}

void* SNPEBase::GetInputBuffer() {
  void* buf =
    (void*)snpe_params_.in_heap_map[snpe_params_.input_layer.c_str()].data();

  // memset buffer now to avoid it while padding/mean subtract
  memset(buf, 0, snpe_params_.in_heap_map[snpe_params_.input_layer.c_str()].size());
  return buf;
}

int32_t SNPEBase::ExecuteModel() {
    MLE_LOGI("%s: Execution begin!", __func__);
  if (config_.io_type == NetworkIO::kUserBuffer) {
    if (!snpe_params_.snpe->execute(snpe_params_.input_ub_map,
                                    snpe_params_.output_ub_map)) {
      PrintErrorStringAndExit();
      return MLE_FAIL;
    }
    MLE_LOGI("%s: Execution completed!", __func__);
  } else if (config_.io_type == NetworkIO::kITensor) {
    snpe_params_.output_tensor_map.clear();
    if (!snpe_params_.snpe->execute(snpe_params_.input_tensor_map,
                                    snpe_params_.output_tensor_map)) {
      PrintErrorStringAndExit();
      return MLE_FAIL;
    }
    MLE_LOGI("%s: Execution completed!", __func__);
  } else {
    MLE_LOGE("%s: Invalid Network IO value", __func__);
    return MLE_FAIL;
  }

  return MLE_OK;
}

int32_t SNPEBase::PostProcess(GstBuffer* buffer) {
  std::vector<float> score_buf;
  const zdl::DlSystem::StringList &output_buf_names =
      snpe_params_.output_ub_map.getUserBufferNames();
  const zdl::DlSystem::StringList &output_tensor_names =
      snpe_params_.output_tensor_map.getTensorNames();
  const zdl::DlSystem::StringList *output_names = &output_buf_names;
  if (config_.io_type == NetworkIO::kITensor) {
    output_names = &output_tensor_names;
  }
  std::for_each(
      output_names->begin(),
      output_names->end(),
      [&](const char* name)
      {
        if (0 == std::strcmp(name, snpe_params_.result_layers[0].c_str())) {
          if (config_.io_type == NetworkIO::kUserBuffer) {
            score_buf = snpe_params_.out_heap_map.at(name);
          } else if (config_.io_type == NetworkIO::kITensor) {
            auto t = snpe_params_.output_tensor_map.getTensor(name);
            for (auto it = t->begin(); it != t->end(); it++) {
              score_buf.push_back(*it);
            }
          }
        }
      });

  uint32_t top_score_idx = 0;
  float top_score = 0.0;

  for (size_t i = 0; i < score_buf.size(); i++) {
    if (score_buf[i] > top_score) {
      top_score = score_buf[i];
      top_score_idx = i;
    }
  }
  if (top_score_idx < labels_.size() &&
      top_score > config_.conf_threshold) {

    GstMLClassificationMeta *meta =
        gst_buffer_add_classification_meta(buffer);
    if (!meta) {
      MLE_LOGE("Failed to create metadata");
      return MLE_NULLPTR;
    }

    meta->result.confidence = top_score * 100;
    uint32_t label_size = labels_.at(top_score_idx).size() + 1;
    meta->result.name = (gchar *)malloc(label_size);
    snprintf(meta->result.name, label_size, "%s", labels_.at(top_score_idx).c_str());
  }

  return MLE_OK;
}

void SNPEBase::PrintErrorStringAndExit() {
  const char* const err = zdl::DlSystem::getLastErrorString();
  MLE_LOGE(" %s", err);
}

int32_t SNPEBase::InitFramework() {
  MLE_LOGI("%s Enter", __func__);
  version_ = zdl::SNPE::SNPEFactory::getLibraryVersion();
  MLE_LOGI("SNPE version: %s", version_.toString().c_str());
  int32_t res = MLE_OK;

  snpe_params_.snpe = SetBuilderOptions();
  if (nullptr == snpe_params_.snpe) {
    PrintErrorStringAndExit();
    res = MLE_FAIL;
  }

  if (MLE_OK == res) {
    ConfigureDimensions();
    res = PopulateMap(BufferType::kInput);
  }
  if (MLE_OK == res) {
    res = PopulateMap(BufferType::kOutput);
  }

  MLE_LOGI("%s Exit", __func__);
  return res;
}

}; // namespace mle
