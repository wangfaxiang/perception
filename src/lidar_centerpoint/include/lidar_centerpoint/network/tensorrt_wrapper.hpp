// Copyright 2021 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LIDAR_CENTERPOINT__NETWORK__TENSORRT_WRAPPER_HPP_
#define LIDAR_CENTERPOINT__NETWORK__TENSORRT_WRAPPER_HPP_

#include <lidar_centerpoint/centerpoint_config.hpp>

#include <NvInfer.h>

#include <iostream>
#include <memory>
#include <string>

namespace centerpoint
{

// Minimal replacement for tensorrt_common::Logger (implements nvinfer1::ILogger).
class Logger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::cerr << "[TensorRT] " << msg << std::endl;
    }
  }
};

// Minimal replacement for tensorrt_common::TrtUniquePtr.
struct InferDeleter
{
  template <typename T>
  void operator()(T * obj) const
  {
    if (obj != nullptr) {
      obj->destroy();
    }
  }
};

template <typename T>
using TrtUniquePtr = std::unique_ptr<T, InferDeleter>;

class TensorRTWrapper
{
public:
  explicit TensorRTWrapper(const CenterPointConfig & config);

  ~TensorRTWrapper();

  bool init(
    const std::string & onnx_path, const std::string & engine_path, const std::string & precision);

  TrtUniquePtr<nvinfer1::IExecutionContext> context_{nullptr};

protected:
  virtual bool setProfile(
    nvinfer1::IBuilder & builder, nvinfer1::INetworkDefinition & network,
    nvinfer1::IBuilderConfig & config) = 0;

  CenterPointConfig config_;
  Logger logger_;

private:
  bool parseONNX(
    const std::string & onnx_path, const std::string & engine_path, const std::string & precision,
    size_t workspace_size = (1ULL << 30));

  bool saveEngine(const std::string & engine_path);

  bool loadEngine(const std::string & engine_path);

  bool createContext();

  TrtUniquePtr<nvinfer1::IRuntime> runtime_{nullptr};
  TrtUniquePtr<nvinfer1::IHostMemory> plan_{nullptr};
  TrtUniquePtr<nvinfer1::ICudaEngine> engine_{nullptr};
};

}  // namespace centerpoint

#endif  // LIDAR_CENTERPOINT__NETWORK__TENSORRT_WRAPPER_HPP_
