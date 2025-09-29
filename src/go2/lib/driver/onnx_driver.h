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

#include "rclcpp/rclcpp.hpp"

#include "src/utils/constants.h"
#include "src/go2/lib/utils/constants.h"
#include "src/go2/lib/driver/go2_driver.h"
#include "src/go2/msgs/unitree_go_msgs.h"

using Go2State = unitree_go::msg::LowState;
using Go2Command = unitree_go::msg::LowCmd;
using namespace robot;
using namespace robot::constants;


/**
 * @class ONNXDriver
 * @brief A class to control the Unitree's Go2 robot using a Policy loaded from an ONNX file.
 */
class ONNXDriver : public rclcpp::Node {
    public:
        /**
         * @brief Construct a new Policy Interface object
         */
        ONNXDriver(
            std::filesystem::path onnx_model_path,
            std::shared_ptr<Go2Driver> unitree_driver
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
         * @brief Gets the latest state of the robot.
         * @return absl::Status OkStatus on success, or an error status if the state cannot be retrieved.
         */
        absl::Status get_initial_position();

        /**
         * @brief Starts the ROS 2 executor in a separate thread to handle callbacks.
         * @return absl::Status OkStatus on success, or an error status if already spinning.
         */
        absl::Status initialize_thread();

        /**
         * @brief Stops the ROS 2 executor thread.
         * @return absl::Status OkStatus on success, or an error if the thread cannot be joined.
         */
        absl::Status stop_thread();

        /**
         * @brief Switches the control mode: Default Position, Hold Position, Damping, and Policy.
         * @param mode The new control mode to set.
         * @return absl::Status OkStatus on success, or an error if the thread cannot be joined.
         */
        absl::Status set_control_mode(go2::constants::HighLevelControlMode mode);

        /**
         * @brief Gets the current control mode.
         * @return HighLevelControlMode The current control mode.
         */
        const go2::constants::HighLevelControlMode get_control_mode();

        /**
         * @brief Sets the command to be sent to the robot using an Eigen Vector.
         * @param new_command The new command to send as a Eigen Vector.
         * @return absl::Status OkStatus on success, or an error if the command is invalid.
         */
        absl::Status set_command(const Vector3<float>& new_command);
        
        /**
         * @brief Sets the command to be sent to the robot using a std::array.
         * @param new_command The new command to send as an array of floats.
         * @return absl::Status OkStatus on success, or an error if the command is invalid.
         */
        absl::Status set_command(const std::array<float, 3>& new_command);

        /**
         * @brief Gets the current command being sent to the robot.
         * @return Vector3<float> The current command.
         */
        const Vector3<float> get_command();

        /**
         * @brief Sets the master gain that scales the policy output.
         * @param gain The new master gain value.
         * @return absl::Status OkStatus on success, or an error if the gain is invalid.
         */
        absl::Status set_master_gain(float gain);

        /**
         * @brief Gets the current master gain.
         * @return float The current master gain value.
         */
        const float get_master_gain();

        /**
         * @brief Gets the output of the policy.
         * @return std::vector<float> The current output of the policy.
         */
        const std::vector<float> get_policy_output();

        /**
         * @brief Gets the current observation.
         * @return Eigen::Vector<float, Eigen::Dynamic> The current observation.
         */
        const Eigen::Vector<float, Eigen::Dynamic> get_observation();

        /**
         * @brief Checks if the ONNX session has been initialized.
         * @return true if the ONNX session is initialized, false otherwise.
         */
        const bool is_initialized() const;

        /**
         * @brief Checks if the ROS 2 executor thread has been initialized.
         * @return true if the executor thread is initialized, false otherwise.
         */
        const bool is_thread_initialized() const;
    
    private:
        /**
         * @brief Inference the ONNX policy model with the current observation.
         * @return absl::Status OkStatus on success, or an error status on failure.
         */
        absl::Status inference_policy();

        /**
         * @brief Constructs a new observation from the Unitree driver.
         * @return absl::Status OkStatus on success, or an error status on failure.
         */
        absl::Status make_observation();

        /**
         * @brief Transforms the Policy output to a Go2 Command.
         * @return Go2Command Motor command.
         */
        Go2Command policy_command();

        /**
         * @brief Callback function for the ROS 2 executor to handle policy updates.
         */
        void policy_callback();

        // Shared Variables
        constants::Vector3<float> command = { 0.0, 0.0, 0.0 };

        // ONNX Variables
        std::filesystem::path onnx_model_path;
        std::shared_ptr<Ort::Env> env = std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "ONNXPolicy");
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
        bool thread_initialized = false;
        bool initialized = false;
        
        // Unitree Driver
        std::shared_ptr<Go2Driver> unitree_driver;
        
        // Thread Variables
        std::mutex mutex;
        
        // Policy Variables
        size_t input_tensor_size;
        size_t output_tensor_size;
        std::vector<float> policy_input;
        std::vector<float> policy_output;
        Eigen::Vector<float, Eigen::Dynamic> observation;

        // ROS2 Thread:
        rclcpp::executors::MultiThreadedExecutor executor;
        std::jthread executor_thread;
        rclcpp::TimerBase::SharedPtr timer_;

        // Default Command Values
        go2::constants::HighLevelControlMode control_mode = go2::constants::HighLevelControlMode::DAMPING;
        int control_rate_us;
        const float action_scale = 0.5f;
        float master_gain = 0.0f;
        const go2::constants::MotorVector<float> default_position = Eigen::Map<const go2::constants::MotorVector<float>>(go2::constants::default_position.data());
        const go2::constants::ActionVector<float> default_setpoints = Eigen::Map<const go2::constants::ActionVector<float>>(go2::constants::default_setpoints.data());
        static constexpr std::array<float, go2::constants::num_joints> kp = go2::constants::default_kp;
        static constexpr std::array<float, go2::constants::num_joints> kd = go2::constants::default_kd;

};