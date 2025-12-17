#include "src/go2/lib/policies/velocity_control_policy.h"

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
#include "src/go2/lib/utils/utilities.h"
#include "src/go2/msgs/unitree_go_msgs.h"

#include "geometry_msgs/msg/vector3.hpp"


using Go2State = unitree_go::msg::LowState;
using Go2Command = unitree_go::msg::LowCmd;
using namespace robot;
using namespace robot::constants;


VelocityControlPolicy::VelocityControlPolicy(
    const rclcpp::NodeOptions& options,
    std::filesystem::path onnx_model_path,
    std::shared_ptr<Go2Driver> unitree_driver
) : 
    Node("walking_policy_interface", options),
    onnx_model_path(onnx_model_path),
    unitree_driver(unitree_driver) {
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

VelocityControlPolicy::~VelocityControlPolicy() {
    if (executor.is_spinning())
        executor.cancel();
}

absl::Status VelocityControlPolicy::initialize() {
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

absl::Status VelocityControlPolicy::initialize_thread() {
    absl::Status result;

    if (!unitree_driver->is_thread_initialized())
        result.Update(unitree_driver->initialize_thread());

    if (executor.is_spinning())
        return absl::InternalError(
            "[Velocity Control Policy Interface] [initialize_thread]: Node is already spinning."
        );

    thread_initialized = true;
    executor.add_node(this->shared_from_this());
    executor_thread = std::jthread([this] { executor.spin(); });

    return absl::OkStatus();
};

absl::Status VelocityControlPolicy::stop_thread() {
    if (!thread_initialized)
        return absl::FailedPreconditionError(
            "[Velocity Control Policy Interface] [stop_thread]: Node needs to be spinning."
        );

    if (executor.is_spinning())
        executor.cancel();

    executor_thread.join();
    return absl::OkStatus();
};

absl::Status VelocityControlPolicy::make_observation() {
    // Get Measurements:
    std::optional<Go2State> state = unitree_driver->get_state();
    if (!state)
        return absl::InternalError("[Velocity Control Policy Interface] [make_observation]: Failed to get state from Unitree driver");

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

    // Previous Actions:
    const auto& policy_output = onnx_driver->get_policy_output();
    velocity_control::constants::ActionVector<float> previous_actions = Eigen::Map<const velocity_control::constants::ActionVector<float>>(policy_output.data());

    // Projected Gravity:
    Eigen::Quaternion<float> quaternion(
        quaternion_measurement(0), quaternion_measurement(1), quaternion_measurement(2), quaternion_measurement(3)
    );
    quaternion.normalize();
    Eigen::Matrix3<float> rotation = quaternion.toRotationMatrix();
    constants::Vector3<float> projected_gravity = rotation.transpose() * constants::Vector3<float>(0.0f, 0.0f, -1.0f);
    
    // Set Observation:
    Eigen::Vector<float, Eigen::Dynamic> observation;
    observation.resize(onnx_driver->get_input_tensor_size());
    observation << gyroscope_measurement,
                    projected_gravity,
                    joint_positions - default_position,
                    joint_velocities,
                    previous_actions,
                    command;

    // Set Input Tensor:
    absl::Status status = onnx_driver->set_observation(observation);
    if (!status.ok()) {
        std::string message = std::string(status.message());
        return absl::InternalError("[Velocity Control Policy Interface] [make_observation]: Failed to set observation in ONNX driver: " + message);
    }

    return absl::OkStatus();
};

Go2Command VelocityControlPolicy::policy_command() {
    const auto& policy_output = onnx_driver->get_policy_output();
    velocity_control::constants::ActionVector<float> actions = Eigen::Map<const velocity_control::constants::ActionVector<float>>(policy_output.data());
    velocity_control::constants::ActionVector<float> setpoints = default_setpoints + master_gain * action_scale * actions;
    go2::constants::MotorVector<float> position_setpoints = setpoints.head<go2::constants::num_joints>();
    go2::constants::MotorVector<float> velocity_setpoints = setpoints.tail<go2::constants::num_joints>();

    Go2Command command = go2::utilities::default_position_command();

    for (size_t i = 0; i < go2::constants::num_joints; ++i) {
        auto& motor_command = command.motor_cmd[i];
        motor_command.q = position_setpoints(i);
        motor_command.dq = velocity_setpoints(i);
        motor_command.kp = kp[i];
        motor_command.kd = kd[i];
    }

    return command;
};

void VelocityControlPolicy::policy_callback() {
    absl::Status result;
    std::lock_guard<std::mutex> lock(mutex);

    result.Update(this->make_observation());
    if (!result.ok()) {
        std::string message = std::string(result.message());
        RCLCPP_ERROR(this->get_logger(), "[Velocity Control Policy Interface] Failed to make observation: %s", message.c_str());
        std::ignore = unitree_driver->update_command(go2::utilities::damping_command());
        return;
    }

    result.Update(onnx_driver->inference_policy());
    if (!result.ok()) {
        std::string message = std::string(result.message());
        RCLCPP_ERROR(this->get_logger(), "[Velocity Control Policy Interface] Failed to run policy inference: %s", message.c_str());
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
        case go2::constants::HighLevelControlMode::INACTIVE:
            return;
    }

    // Send Motor Command:
    std::ignore = unitree_driver->update_command(command);

};
