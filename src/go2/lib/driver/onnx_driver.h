#pragma once

#include <iostream>
#include <vector>   
#include <array>
#include <filesystem>
#include <thread>
#include <cmath>
#include <numeric>

#include "absl/status/status.h"
#include "absl/log/absl_check.h"

#include <onnxruntime_cxx_api.h>

#include "Eigen/Dense"


/**
 * @class ONNXDriver
 * @brief A class to control the Unitree's Go2 robot using a Policy loaded from an ONNX file.
 */
class ONNXDriver {
    public:
        /**
         * @brief Construct a new Policy Interface object
         */
        ONNXDriver(
            std::filesystem::path onnx_model_path,
            std::string onnx_session_id
        );

        /**
         * @brief Destroy the Policy Interface object, ensuring proper cleanup of resources.
         */
        ~ONNXDriver();

        // Disable copy and move semantics to prevent accidental duplication
        ONNXDriver(const ONNXDriver&) = delete;
        ONNXDriver& operator=(const ONNXDriver&) = delete;
        ONNXDriver(ONNXDriver&&) = delete;
        ONNXDriver& operator=(ONNXDriver&&) = delete;

        /**
         * @brief Initializes the Unitree Driver and ONNX Session.
         * @return absl::Status OkStatus on success, or an error status on failure.
         */
        absl::Status initialize();

        /**
         * @brief Explicitly Initializes the ONNX Session.
         * @return absl::Status OkStatus on success, or an error status on failure.
         */
        absl::Status initialize_session();

        /**
         * @brief Inference the ONNX policy model with the current observation.
         * @return absl::Status OkStatus on success, or an error status on failure.
         */
        absl::Status inference_policy();

        /**
         * @brief Updates the observation vector.
         * @param new_observation The new observation to set.
         */
        absl::Status set_observation(const Eigen::Vector<float, Eigen::Dynamic>& new_observation);

        /**
         * @brief Gets the current observation.
         * @return Eigen::Vector<float, Eigen::Dynamic> The current observation.
         */
        const Eigen::Vector<float, Eigen::Dynamic> observation() const {
            return observation;
        }

        /**
         * @brief Gets the input of the policy.
         * @return std::vector<float> The current input of the policy.
         */
        const std::vector<float> policy_input() const {
            return policy_input;
        }

        /**
         * @brief Gets the output of the policy.
         * @return std::vector<float> The current output of the policy.
         */
        const std::vector<float> policy_output() const {
            return policy_output;
        }

        /**
         * @brief Gets the size of the input tensor.
         * @return size_t The size of the input tensor.
         */
        const size_t input_tensor_size() const {
            return input_tensor_size;
        };

        /**
         * @brief Gets the size of the output tensor.
         * @return size_t The size of the output tensor.
         */
        const size_t output_tensor_size() const {
            return output_tensor_size;
        };
        
         /**
         * @brief Checks if the ONNX session has been initialized.
         * @return true if the ONNX session is initialized, false otherwise.
         */
        const bool is_initialized() const {
            return initialized;
        }
        
    private:
        // ONNX Variables
        std::filesystem::path onnx_model_path;
        std::string onnx_session_id;
        std::shared_ptr<Ort::Env> env = std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, onnx_session_id.c_str());
        std::unique_ptr<Ort::Session> session_ptr;
        Ort::AllocatorWithDefaultOptions allocator;
        std::vector<Ort::AllocatedStringPtr> input_nodes;
        std::vector<Ort::AllocatedStringPtr> output_nodes;
        std::vector<const char*> input_names;
        std::vector<const char*> output_names;
        std::vector<ONNXTensorElementDataType> input_types;
        std::vector<ONNXTensorElementDataType> output_types;
        std::vector<std::vector<int64_t>> input_shapes;
        std::vector<std::vector<int64_t>> output_shapes;
        
        // Initialization Flags
        bool session_initialized = false;
        bool initialized = false;
        
        // Thread Variables
        std::mutex mutex;
        
        // Policy Variables
        size_t input_tensor_size;
        size_t output_tensor_size;
        std::vector<float> policy_input;
        std::vector<float> policy_output;
        Eigen::Vector<float, Eigen::Dynamic> observation;

};