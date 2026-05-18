#pragma once

#include <iostream>
#include <vector>   
#include <array>
#include <filesystem>
#include <thread>
#include <cmath>
#include <numeric>
#include <memory>
#include <mutex>
#include <optional>
#include <algorithm>
#include <string>

#include "absl/status/status.h"
#include "absl/log/absl_check.h"

#include <onnxruntime_cxx_api.h>
#include "Eigen/Dense"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "src/go2/lib/driver/onnx_driver.h"
#include "src/go2/lib/driver/go2_driver.h"
#include "src/go2/lib/driver/filter_driver.h"

#include "src/utils/constants.h"
#include "src/go2/lib/utils/constants.h"
#include "src/go2/lib/utils/utilities.h"
#include "src/go2/msgs/unitree_go_msgs.h"


using Go2State = unitree_go::msg::LowState;
using Go2Command = unitree_go::msg::LowCmd;

using namespace robot;
using namespace robot::constants;


/**
 * @class PositionControlPolicy
 * @brief A class to control the Unitree's Go2 robot using a Policy loaded from an ONNX file.
 */
template <typename FilterType = NoFilter<go2::constants::num_joints>>
class PositionControlPolicy : public rclcpp::Node {
    public:
        /**
         * @brief Construct a new Policy Interface object
         */
        PositionControlPolicy(
            const rclcpp::NodeOptions& options,
            std::filesystem::path onnx_model_path,
            std::shared_ptr<Go2Driver> unitree_driver,
            FilterType filter
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
                if (!unitree_driver->has_state_estimation()) {
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
            {
                std::lock_guard<std::mutex> lock(mutex);
                command = new_command;
            }

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
            {
                std::lock_guard<std::mutex> lock(mutex);
                command = Eigen::Map<const constants::Vector3<float>>(new_command.data());
            }

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
        std::shared_ptr<ONNXDriver> onnx_driver;
        
        // Initialization Flags
        bool thread_initialized = false;
        bool initialized = false;
        
        // Unitree Driver
        std::shared_ptr<Go2Driver> unitree_driver;

        // Action Filter:
        FilterType filter;
        
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
        const go2::constants::MotorVector<float> action_scale = Eigen::Map<const go2::constants::MotorVector<float>>(go2::constants::action_scale.data());
        float master_gain = 0.0f;
        const go2::constants::MotorVector<float> default_position = Eigen::Map<const go2::constants::MotorVector<float>>(go2::constants::default_position.data());
        static constexpr std::array<float, go2::constants::num_joints> kp = go2::constants::default_kp;
        static constexpr std::array<float, go2::constants::num_joints> kd = go2::constants::default_kd;
        
};

// Implementation:
template <typename FilterType>
PositionControlPolicy<FilterType>::PositionControlPolicy(
    const rclcpp::NodeOptions& options,
    std::filesystem::path onnx_model_path,
    std::shared_ptr<Go2Driver> unitree_driver,
    FilterType filter
) : 
    Node("walking_policy_interface", options),
    onnx_model_path(onnx_model_path),
    onnx_driver(std::make_shared<ONNXDriver>(onnx_model_path, "WalkingPolicySession")),
    unitree_driver(unitree_driver),
    filter(std::move(filter)) {
    // Set Control Rate:
    declare_parameter("control_rate_us", 20000);
    this->control_rate_us = this->get_parameter("control_rate_us").as_int();
    
    // Create default QoS Profile:
    rclcpp::QoS qos_profile(10);
    qos_profile.best_effort();
    
    // Create Publisher Node:
    policy_command_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "/policy_command",
        qos_profile
    );
}

template <typename FilterType>
PositionControlPolicy<FilterType>::~PositionControlPolicy() {
    if (executor.is_spinning())
        executor.cancel();
}

template <typename FilterType>
absl::Status PositionControlPolicy<FilterType>::initialize() {
    absl::Status result;
    // Initialize Robot Driver:
    if (!unitree_driver->is_initialized())
        result.Update(unitree_driver->initialize());

    // Initialize ONNX Session:
    if (!onnx_driver->is_initialized())
        result.Update(onnx_driver->initialize());

    // Initialize Thread:
    if (!thread_initialized)
        result.Update(initialize_thread());

    ABSL_CHECK(result.ok()) << result.message();

    initialized = true;
    timer_ = create_wall_timer(
        std::chrono::microseconds(control_rate_us),
        [this]() { this->policy_callback(); }
    );

    return result;
};

template <typename FilterType>
absl::Status PositionControlPolicy<FilterType>::initialize_thread() {
    absl::Status result;

    if (!unitree_driver->is_thread_initialized())
        result.Update(unitree_driver->initialize_thread());

    if (executor.is_spinning())
        return absl::InternalError(
            "[Position Control Policy Interface] [initialize_thread]: Node is already spinning."
        );

    thread_initialized = true;
    executor.add_node(this->shared_from_this());
    executor_thread = std::jthread([this] { executor.spin(); });

    return absl::OkStatus();
};

template <typename FilterType>
absl::Status PositionControlPolicy<FilterType>::stop_thread() {
    if (!thread_initialized)
        return absl::FailedPreconditionError(
            "[Position Control Policy Interface] [stop_thread]: Node needs to be spinning."
        );

    if (executor.is_spinning())
        executor.cancel();

    executor_thread.join();
    return absl::OkStatus();
};

template <typename FilterType>
absl::Status PositionControlPolicy<FilterType>::make_observation() {
    // Get Measurements:
    std::optional<Go2State> state = unitree_driver->get_state();
    if (!state) [[unlikely]]
        return absl::InternalError("[Position Control Policy Interface] [make_observation]: Failed to get state from Unitree driver");

    // Get Measurements from the state:
    const auto& imu_state = state->imu_state;
    const auto& motor_state = state->motor_state;
    const auto& foot_force = state->foot_force;

    constants::Vector3<float> accelerometer_measurement = Eigen::Map<const constants::Vector3<float>>(imu_state.accelerometer.data());
    constants::Vector3<float> gyroscope_measurement = Eigen::Map<const constants::Vector3<float>>(imu_state.gyroscope.data());
    constants::Vector4<float> quaternion_measurement = Eigen::Map<const constants::Vector4<float>>(imu_state.quaternion.data());
    go2::constants::MotorVector<float> joint_positions;
    go2::constants::MotorVector<float> joint_velocities;
    for (size_t i = 0; i < go2::constants::num_joints; ++i) {
        joint_positions(i) = motor_state[i].q;
        joint_velocities(i) = motor_state[i].dq;
    }

    // Contact Mask:
    Eigen::Map<const Eigen::Matrix<int16_t, 4, 1>> force_map(foot_force.data());
    Eigen::Vector4f contact_mask = (force_map.array() >= 20).cast<float>();

    // Previous Actions:
    const auto& policy_output = onnx_driver->get_policy_output();
    go2::constants::MotorVector<float> previous_actions = Eigen::Map<const go2::constants::MotorVector<float>>(policy_output.data());

    // Projected Gravity:
    Eigen::Quaternion<float> quaternion(
        quaternion_measurement(0), quaternion_measurement(1), quaternion_measurement(2), quaternion_measurement(3)
    );
    quaternion.normalize();
    Eigen::Matrix3<float> rotation = quaternion.toRotationMatrix();
    constants::Vector3<float> projected_gravity = rotation.transpose() * constants::Vector3<float>(0.0f, 0.0f, -1.0f);
    
    // Mutex Locked Getter:
    const auto cached_command = this->get_command();

    // Get Filter Observation:
    const auto filter_observation = filter.get_observation();

    // Set Observation:
    Eigen::Vector<float, Eigen::Dynamic> observation;
    observation.resize(onnx_driver->get_input_tensor_size());
    observation <<  gyroscope_measurement,
                    projected_gravity,
                    joint_positions - default_position,
                    joint_velocities,
                    previous_actions,
                    cached_command,
                    filter_observation;

    // Set Input Tensor:
    absl::Status status = onnx_driver->set_observation(observation);
    if (!status.ok()) [[unlikely]] {
        std::string message = std::string(status.message());
        return absl::InternalError("[Position Control Policy Interface] [make_observation]: Failed to set observation in ONNX driver: " + message);
    }

    return absl::OkStatus();
};

template <typename FilterType>
Go2Command PositionControlPolicy<FilterType>::policy_command() {
    const auto& policy_output = onnx_driver->get_policy_output();
    
    const auto actions = Eigen::Map<const go2::constants::MotorVector<float>>(policy_output.data());
    // Apply Go2Filter (NOTE: updated from filter-> to filter.):
    const go2::constants::MotorVector<float> filtered_actions = filter.apply(actions);
    const go2::constants::MotorVector<float> position_setpoints = default_position + master_gain * (action_scale.cwiseProduct(filtered_actions));

    Go2Command command = go2::utilities::default_position_command();

    for (size_t i = 0; i < go2::constants::num_joints; ++i) {
        auto& motor_command = command.motor_cmd[i];
        motor_command.q = position_setpoints(i);
        motor_command.kp = kp[i];
        motor_command.kd = kd[i];
    }

    return command;
};

template <typename FilterType>
void PositionControlPolicy<FilterType>::policy_callback() {
    absl::Status result;

    result.Update(this->make_observation());
    if (!result.ok()) [[unlikely]] {
        std::string message = std::string(result.message());
        RCLCPP_ERROR(this->get_logger(), "[Position Control Policy Interface] Failed to make observation: %s", message.c_str());
        std::ignore = unitree_driver->update_command(go2::utilities::damping_command());
        return;
    }

    result.Update(onnx_driver->inference_policy());
    if (!result.ok()) [[unlikely]] {
        std::string message = std::string(result.message());
        RCLCPP_ERROR(this->get_logger(), "[Position Control Policy Interface] Failed to run policy inference: %s", message.c_str());
        std::ignore = unitree_driver->update_command(go2::utilities::damping_command());
        return;
    }

    // Get Motor Command:
    Go2Command command;

    {
        std::lock_guard<std::mutex> lock(mutex);
        switch (control_mode) {
            case go2::constants::HighLevelControlMode::DEFAULT:
                command = go2::utilities::default_position_command();
                for (size_t i = 0; i < go2::constants::num_joints; ++i) {
                    auto& motor_command = command.motor_cmd[i];
                    motor_command.kp = master_gain * kp[i];
                    motor_command.kd = kd[i];
                }
                break;
            case go2::constants::HighLevelControlMode::DAMPING:
                command = go2::utilities::damping_command();
                break;
            case go2::constants::HighLevelControlMode::POLICY:
                command = this->policy_command();
                break;
            case go2::constants::HighLevelControlMode::DISABLE:
                command = go2::utilities::disable_command();
                break;
            case go2::constants::HighLevelControlMode::INACTIVE:
                return;
        }
    }

    // Send Motor Command:
    std::ignore = unitree_driver->update_command(command);

};