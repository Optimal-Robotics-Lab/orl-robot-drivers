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

#include "src/go2/lib/driver/onnx_driver.h"
#include "src/go2/lib/driver/go2_driver.h"

#include "src/utils/constants.h"
#include "src/go2/lib/utils/constants.h"
#include "src/go2/msgs/unitree_go_msgs.h"

#include "geometry_msgs/msg/vector3.hpp"


using Go2State = unitree_go::msg::LowState;
using Go2Command = unitree_go::msg::LowCmd;
using namespace robot;
using namespace robot::constants;


/**
 * @class PositionControlPolicy
 * @brief A class to control the Unitree's Go2 robot using a Policy loaded from an ONNX file.
 */
class PositionControlPolicy : public rclcpp::Node {
    public:
        /**
         * @brief Construct a new Policy Interface object
         */
        PositionControlPolicy(
            const rclcpp::NodeOptions& options,
            std::filesystem::path onnx_model_path,
            std::shared_ptr<Go2Driver> unitree_driver
        );

        /**
         * @brief Destroy the Policy Interface object, ensuring proper cleanup of resources.
         */
        ~PositionControlPolicy();

        // Disable copy and move semantics to prevent accidental duplication
        PositionControlPolicy(const PositionControlPolicy&) = delete;
        PositionControlPolicy& operator=(const PositionControlPolicy&) = delete;
        PositionControlPolicy(PositionControlPolicy&&) = delete;
        PositionControlPolicy& operator=(PositionControlPolicy&&) = delete;

        /**
         * @brief Initializes the Unitree Driver and ONNX Session.
         * @return absl::Status OkStatus on success, or an error status on failure.
         */
        absl::Status initialize();

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
        absl::Status set_control_mode(go2::constants::HighLevelControlMode mode) {
            std::lock_guard<std::mutex> lock(mutex);
            
            if (mode == go2::constants::HighLevelControlMode::POLICY) {
                if (!unitree_driver->get_state_estimation().has_value()) {
                    return absl::FailedPreconditionError(
                        "[Position Control Policy] Cannot switch to POLICY: EKF odometry is not yet publishing."
                    );
                }
            }

            control_mode = mode;
            return absl::OkStatus();
        };

        /**
         * @brief Gets the current control mode.
         * @return HighLevelControlMode The current control mode.
         */
        const go2::constants::HighLevelControlMode get_control_mode() {
            std::lock_guard<std::mutex> lock(mutex);
            return control_mode;
        };

        /**
         * @brief Sets the command to be sent to the robot using an Eigen Vector.
         * @param new_command The new command to send as a Eigen Vector.
         * @return absl::Status OkStatus on success, or an error if the command is invalid.
         */
        absl::Status set_command(const Vector3<float>& new_command) {
            std::lock_guard<std::mutex> lock(mutex);
            command = new_command;
            
        if (policy_command_publisher_) {
            geometry_msgs::msg::Vector3 command_msg;
            command_msg.x = command(0);
            command_msg.y = command(1);
            command_msg.z = command(2);
            policy_command_publisher_->publish(command_msg);
        }

            return absl::OkStatus();
        };
        
        /**
         * @brief Sets the command to be sent to the robot using a std::array.
         * @param new_command The new command to send as an array of floats.
         * @return absl::Status OkStatus on success, or an error if the command is invalid.
         */
        absl::Status set_command(const std::array<float, 3>& new_command) {
            std::lock_guard<std::mutex> lock(mutex);
            command = Eigen::Map<const constants::Vector3<float>>(new_command.data());

            if (policy_command_publisher_) {
                geometry_msgs::msg::Vector3 command_msg;
                command_msg.x = command(0);
                command_msg.y = command(1);
                command_msg.z = command(2);
                policy_command_publisher_->publish(command_msg);
            }

            return absl::OkStatus();
        };

        /**
         * @brief Gets the current command being sent to the robot.
         * @return Vector3<float> The current command.
         */
        const Vector3<float> get_command() {
            std::lock_guard<std::mutex> lock(mutex);
            return command;
        };

        /**
         * @brief Sets the master gain that scales the policy output.
         * @param gain The new master gain value.
         * @return absl::Status OkStatus on success, or an error if the gain is invalid.
         */
        absl::Status set_master_gain(float gain) {
            std::lock_guard<std::mutex> lock(mutex);
            master_gain = std::clamp(gain, 0.0f, 1.0f);
            if (master_gain < 0.0f || master_gain > 1.0f) {
                return absl::InvalidArgumentError("Policy Interface [set_master_gain]: Gain clamped between 0.0 and 1.0.");
            }
            return absl::OkStatus();
        };


        /**
         * @brief Gets the current master gain.
         * @return float The current master gain value.
         */
        const float get_master_gain() {
            std::lock_guard<std::mutex> lock(mutex);
            return master_gain;
        }

        /**
         * @brief Gets the output of the policy.
         * @return std::vector<float> The current output of the policy.
         */
        const std::vector<float> get_policy_output() {
            std::lock_guard<std::mutex> lock(mutex);
            return onnx_driver->get_policy_output();
        }

        /**
         * @brief Gets the current observation.
         * @return Eigen::Vector<float, Eigen::Dynamic> The current observation.
         */
        const Eigen::Vector<float, Eigen::Dynamic> get_observation() {
            std::lock_guard<std::mutex> lock(mutex);
            return onnx_driver->get_observation();
        }

        /**
         * @brief Checks if the ONNX session has been initialized.
         * @return true if the ONNX session is initialized, false otherwise.
         */
        const bool is_initialized() const {
            return initialized;
        }

        /**
         * @brief Checks if the ROS 2 executor thread has been initialized.
         * @return true if the executor thread is initialized, false otherwise.
         */
        const bool is_thread_initialized() const {
            return thread_initialized;
        }
    
    private:
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
        std::shared_ptr<ONNXDriver> onnx_driver = std::make_shared<ONNXDriver>(onnx_model_path, "WalkingPolicySession");
        
        // Initialization Flags
        bool thread_initialized = false;
        bool initialized = false;
        
        // Unitree Driver
        std::shared_ptr<Go2Driver> unitree_driver;
        
        // Thread Variables
        std::mutex mutex;

        // ROS2 Thread:
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr policy_command_publisher_;
        rclcpp::executors::MultiThreadedExecutor executor;
        std::jthread executor_thread;
        rclcpp::TimerBase::SharedPtr timer_;

        // Default Command Values
        go2::constants::HighLevelControlMode control_mode = go2::constants::HighLevelControlMode::DAMPING;
        int control_rate_us;
        const float action_scale = 0.5f;
        float master_gain = 0.0f;
        const go2::constants::MotorVector<float> default_position = Eigen::Map<const go2::constants::MotorVector<float>>(go2::constants::default_position.data());
        static constexpr std::array<float, go2::constants::num_joints> kp = go2::constants::default_kp;
        static constexpr std::array<float, go2::constants::num_joints> kd = go2::constants::default_kd;

};