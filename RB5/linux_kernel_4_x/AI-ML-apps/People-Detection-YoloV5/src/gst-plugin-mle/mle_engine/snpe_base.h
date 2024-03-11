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

#pragma once

#include <cstdlib>
#include <vector>
#include <string>
#include <iterator>
#include <stdlib.h>
#include <thread>
#include <memory>
#include <cstring>
#include <unordered_map>
#include <algorithm>
#include <fcntl.h>
#include <condition_variable>
#include <mutex>

#include "DlContainer/IDlContainer.hpp"
#include "SNPE/SNPE.hpp"
#include "SNPE/SNPEFactory.hpp"
#include "SNPE/SNPEBuilder.hpp"
#include "DlSystem/DlError.hpp"
#include "DlSystem/ITensorFactory.hpp"
#include "DlSystem/TensorMap.hpp"
#include "DlSystem/TensorShape.hpp"
#include "DlSystem/StringList.hpp"
#include "DlSystem/DlError.hpp"
#include "DlSystem/IUserBuffer.hpp"
#include "DlSystem/IUserBufferFactory.hpp"
#include "DlSystem/UserBufferMap.hpp"
#include "DlSystem/IBufferAttributes.hpp"
#include "ml_engine_intf.h"

namespace mle {

enum class BufferType {
  kOutput = 0,
  kInput
};

struct SNPEParams {
  std::unique_ptr<zdl::DlContainer::IDlContainer> container;
  std::unique_ptr<zdl::SNPE::SNPE> snpe;

  std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>> ub_list;
  zdl::DlSystem::UserBufferMap output_ub_map;
  zdl::DlSystem::UserBufferMap input_ub_map;

  std::vector<std::unique_ptr<zdl::DlSystem::ITensor>> tensor_list;
  zdl::DlSystem::TensorMap output_tensor_map;
  zdl::DlSystem::TensorMap input_tensor_map;

  std::unordered_map<std::string, std::vector<float>> in_heap_map;
  std::unordered_map<std::string, std::vector<float>> out_heap_map;

  std::string input_layer;
  std::vector<std::string> result_layers;
};

class SNPEBase : public MLEngine {
 public:
  SNPEBase(MLConfig &config);
  virtual ~SNPEBase(){};

 protected:
  void PrintErrorStringAndExit();
  virtual int32_t PostProcess(GstBuffer* buffer);
  SNPEParams snpe_params_;
  zdl::DlSystem::Runtime_t runtime_;
  zdl::DlSystem::Version_t version_;

 private:
  int32_t LoadModel(std::string& model_path);
  int32_t ConfigureRuntime(MLConfig &config);
  int32_t ConfigureDimensions();
  int32_t InitFramework();
  int32_t ExecuteModel();
  void* GetInputBuffer();
  std::unique_ptr<zdl::DlContainer::IDlContainer> LoadContainerFromFile(
      std::string container_path);
  std::unique_ptr<zdl::SNPE::SNPE> SetBuilderOptions();
  virtual size_t CalculateSizeFromDims(const size_t rank,
                                       const zdl::DlSystem::Dimension* dims,
                                       const size_t& element_size);
  virtual std::vector<size_t> GetStrides(zdl::DlSystem::TensorShape dims,
                                         const size_t& element_size);

  int32_t PopulateMap(BufferType type);
  int32_t CreateUserBuffer(BufferType type, const char* name);
  int32_t CreateTensor(BufferType type, const char* name);
};

}; // namespace mle
