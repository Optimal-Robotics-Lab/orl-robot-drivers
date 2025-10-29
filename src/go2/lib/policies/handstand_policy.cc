#include "src/go2/lib/policies/handstand_policy.h"

#include <iostream>
#include <vector>   
#include <array>
#include <filesystem>
#include <thread>
#include <cmath>
#include <numeric>

#include "absl/status/status.h"
#include "absl/log/absl_check.h"

#include "Eigen/Dense"

#include "rclcpp/rclcpp.hpp"

#include "src/utils/constants.h"
#include "src/go2/lib/utils/constants.h"
#include "src/go2/lib/utils/utilities.h"
#include "src/go2/lib/driver/go2_driver.h"
#include "src/go2/msgs/unitree_go_msgs.h"


using Go2State = unitree_go::msg::LowState;
using Go2Command = unitree_go::msg::LowCmd;
using namespace robot;
using namespace robot::constants;


HandstandPolicy::HandstandPolicy(
    std::filesystem::path onnx_model_path,
    std::shared_ptr<Go2Driver> unitree_driver
) : 
    Node("handstand_policy_interface"),
    onnx_model_path(onnx_model_path),
    unitree_driver(unitree_driver) {

    // Set Control Rate:
    declare_parameter("control_rate_us", 20000);
    this->control_rate_us = this->get_parameter("control_rate_us").as_int();

}

HandstandPolicy::~HandstandPolicy() {
    if (executor.is_spinning())
        executor.cancel();
}

absl::Status HandstandPolicy::initialize() {
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

absl::Status HandstandPolicy::initialize_thread() {
    absl::Status result;

    if (!unitree_driver->is_thread_initialized())
        result.Update(unitree_driver->initialize_thread());

    if (executor.is_spinning())
        return absl::InternalError(
            "[Handstand Policy Interface] [initialize_thread]: Node is already spinning."
        );

    thread_initialized = true;
    executor.add_node(this->shared_from_this());
    executor_thread = std::jthread([this] { executor.spin(); });

    return absl::OkStatus();
};

absl::Status HandstandPolicy::stop_thread() {
    if (!thread_initialized)
        return absl::FailedPreconditionError(
            "[Handstand Policy Interface] [stop_thread]: Node needs to be spinning."
        );

    if (executor.is_spinning())
        executor.cancel();

    executor_thread.join();
    return absl::OkStatus();
};

absl::Status HandstandPolicy::make_observation() {
    // Get Measurements:
    std::optional<Go2State> state = unitree_driver->get_state();
    if (!state)
        return absl::InternalError("[Handstand Policy Interface] [make_observation]: Failed to get state from Unitree driver");

    // Get Measurements from the state:
    const auto& imu_state = state->imu_state;
    const auto& motor_state = state->motor_state;

    constants::Vector3<float> accelerometer_measurement = Eigen::Map<const constants::Vector3<float>>(imu_state.accelerometer.data());
    constants::Vector3<float> gyroscope_measurement = Eigen::Map<const constants::Vector3<float>>(imu_state.gyroscope.data());
    constants::Vector4<float> quaternion_measurement = Eigen::Map<const constants::Vector4<float>>(imu_state.quaternion.data());
    go2::constants::MotorVector<float> joint_positions;
    go2::constants::MotorVector<float> joint_velocities;
    for (size_t i = 0; i < go2::constants::num_joints; ++i) {
        joint_positions(i) = motor_state[i].q;
        joint_velocities(i) = motor_state[i].dq;
    }

    // Update Control Point with Current Joint Positions:
    control_point = joint_positions;

    // Previous Actions:
    go2::constants::MotorVector<float> previous_actions = Eigen::Map<go2::constants::MotorVector<float>>(policy_output.data());

    // Projected Gravity:
    Eigen::Quaternion<float> quaternion(
        quaternion_measurement(0), quaternion_measurement(1), quaternion_measurement(2), quaternion_measurement(3)
    );
    quaternion.normalize();
    Eigen::Matrix3<float> rotation = quaternion.toRotationMatrix();
    constants::Vector3<float> projected_gravity = rotation.transpose() * constants::Vector3<float>(0.0f, 0.0f, -1.0f);
    
    // Set Observation:
    Eigen::Vector<float, Eigen::Dynamic> observation;
    observation.resize(onnx_driver->input_tensor_size());
    observation << gyroscope_measurement,
                    projected_gravity,
                    joint_positions - default_position,
                    joint_velocities,
                    previous_actions,
                    static_cast<float>(command);

    // Set Input Tensor:
    absl::Status status = onnx_driver->set_observation(observation);
    if (!status.ok())
        return absl::InternalError("[Handstand Policy Interface] [make_observation]: Failed to set observation in ONNX driver: " + status.message());

    return absl::OkStatus();
};

Go2Command HandstandPolicy::policy_command() {
    const auto& policy_output = onnx_driver->policy_output();
    go2::constants::MotorVector<float> actions = Eigen::Map<go2::constants::MotorVector<float>>(policy_output.data());
    go2::constants::MotorVector<float> position_setpoints = control_point + master_gain * action_scale * actions;

    Go2Command command = go2::utilities::default_position_command();

    for (size_t i = 0; i < go2::constants::num_joints; ++i) {
        auto& motor_command = command.motor_cmd[i];
        motor_command.q = position_setpoints(i);
        motor_command.kp = kp[i];
        motor_command.kd = kd[i];
    }

    return command;
};

void HandstandPolicy::policy_callback() {
    absl::Status result;
    std::lock_guard<std::mutex> lock(mutex);

    result.Update(this->make_observation());
    if (!result.ok()) {
        RCLCPP_ERROR(this->get_logger(), "[Handstand Policy Interface] Failed to make observation: %s", result.message().c_str());
        control_mode = go2::constants::HighLevelControlMode::DAMPING;
        std::ignore = unitree_driver->update_command(go2::utilities::damping_command());
        return;
    }

    result.Update(onnx_driver->inference_policy());
    if (!result.ok()) {
        RCLCPP_ERROR(this->get_logger(), "[Handstand Policy Interface] Failed to run policy inference: %s", result.message().c_str());
        control_mode = go2::constants::HighLevelControlMode::DAMPING;
        std::ignore = unitree_driver->update_command(go2::utilities::damping_command());
        return;
    }

    // Get Motor Command:
    Go2Command command;
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
    }

    // Send Motor Command:
    std::ignore = unitree_driver->update_command(command);
};
