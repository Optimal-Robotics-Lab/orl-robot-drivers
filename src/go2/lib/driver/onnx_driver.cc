#include "src/go2/lib/driver/onnx_driver.h"

#include <iostream>
#include <vector>   
#include <array>
#include <filesystem>
#include <thread>
#include <cmath>
#include <numeric>
#include <string>

#include "absl/status/status.h"
#include "absl/log/absl_check.h"

#include <onnxruntime_cxx_api.h>

#include "Eigen/Dense"


namespace {
    template <typename T>
    T vector_product(const std::vector<T>& v) {
        return std::accumulate(v.begin(), v.end(), 1, std::multiplies<T>());
    }
}


ONNXDriver::ONNXDriver(
    std::filesystem::path onnx_model_path, std::string onnx_session_id
) : 
    onnx_model_path(onnx_model_path),
    onnx_session_id(onnx_session_id) { }

ONNXDriver::~ONNXDriver() { }

absl::Status ONNXDriver::initialize() {
    absl::Status result;
    // Initialize ONNX Session:
    if (!session_initialized)
        result.Update(initialize_session());

    ABSL_CHECK(result.ok()) << result.message();

    initialized = true;

    return result;
};

absl::Status ONNXDriver::initialize_session() {
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    session_ptr = std::make_unique<Ort::Session>(*env, onnx_model_path.c_str(), session_options);
    if (!session_ptr) {
        return absl::InternalError("[ONNX Driver] [initialize_session]: Failed to create ONNX session");
    }

    // Initialize Inputs and Outputs:
    for (size_t i = 0; i < session_ptr->GetInputCount(); ++i) {
        input_nodes.push_back(session_ptr->GetInputNameAllocated(i, allocator));
        input_names.push_back(input_nodes.back().get());
        Ort::TypeInfo type_info = session_ptr->GetInputTypeInfo(i);
        auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
        input_types.push_back(tensor_info.GetElementType());
        input_shapes.push_back(tensor_info.GetShape());
    }

    // Get Output Names and Shapes
    for (size_t i = 0; i < session_ptr->GetOutputCount(); ++i) {
        output_nodes.push_back(session_ptr->GetOutputNameAllocated(i, allocator));
        output_names.push_back(output_nodes.back().get());
        Ort::TypeInfo type_info = session_ptr->GetOutputTypeInfo(i);
        auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
        output_types.push_back(tensor_info.GetElementType());
        output_shapes.push_back(tensor_info.GetShape());
    }
    
    // Initialize Input and Output Vectors: (assumes 1 input and 1 output tensor)
    if (input_shapes.size() != 1 || output_shapes.size() != 1) {
        return absl::InternalError("[ONNX Driver] [initialize_session]: Expected 1 input and 1 output tensor");
    }
    input_tensor_size = vector_product(input_shapes[0]);
    output_tensor_size = vector_product(output_shapes[0]);
    policy_input.resize(input_tensor_size);
    policy_output.resize(output_tensor_size);
    observation.resize(input_tensor_size);

    session_initialized = true;
    return absl::OkStatus();
};

absl::Status ONNXDriver::set_observation(const Eigen::Vector<float, Eigen::Dynamic>& new_observation) {
    // Update Observation:
    if (new_observation.size() != observation.size())
        return absl::InvalidArgumentError("[ONNX Driver] [set_observation]: Observation size mismatch");

    observation = new_observation;

    // Update Policy Input:
    if (observation.size() != policy_input.size())
        return absl::InternalError("[ONNX Driver] [set_observation]: Policy input size mismatch");

    std::copy(
        observation.data(), 
        observation.data() + observation.size(), 
        policy_input.data()
    );

    return absl::OkStatus();
};

absl::Status ONNXDriver::inference_policy() {
    // Initialize Input and Output Tensors:
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault
    );
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        policy_input.data(),
        policy_input.size(),
        input_shapes[0].data(),
        input_shapes[0].size()
    );
    Ort::Value output_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        policy_output.data(),
        policy_output.size(),
        output_shapes[0].data(),
        output_shapes[0].size()
    );

    // Inference:
    Ort::RunOptions run_options;
    session_ptr->Run(
        run_options,
        input_names.data(),
        &input_tensor,
        1,
        output_names.data(),
        &output_tensor,
        1
    );

    if(!output_tensor.HasValue()) {
        return absl::InternalError("[ONNX Driver] [inference_policy]: Failed to get output tensor");
    }

    return absl::OkStatus();
};