#include "src/go2/lib/driver/onnx_driver.h"

#include <iostream>
#include <vector>   
#include <array>
#include <filesystem>
#include <thread>
#include <cmath>
#include <numeric>
#include <chrono>

#include "absl/status/status.h"
#include "absl/log/absl_check.h"

#include <onnxruntime_cxx_api.h>

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


namespace {
    template <typename T>
    T vector_product(const std::vector<T>& v) {
        return std::accumulate(v.begin(), v.end(), 1, std::multiplies<T>());
    }
}


ONNXDriver::ONNXDriver(
    std::filesystem::path onnx_model_path,
    std::shared_ptr<Go2Driver> unitree_driver,
    std::chrono::microseconds control_rate
) : 
    Node("policy_interface"),
    onnx_model_path(onnx_model_path),
    unitree_driver(unitree_driver),
    control_rate(control_rate) {
    // Set Control Rate:
    this->control_rate_us = control_rate.count();

}

ONNXDriver::~ONNXDriver() {
    if (executor.is_spinning())
        executor.cancel();
}

absl::Status ONNXDriver::initialize() {
    absl::Status result;
    // Initialize Robot Driver:
    if (!unitree_driver->is_initialized())
        result.Update(unitree_driver->initialize());

    // Initialize ONNX Session:
    if (!session_initialized)
        result.Update(initialize_session());

    ABSL_CHECK(result.ok()) << result.message();

    initialized = true;
    timer_ = create_wall_timer(
        std::chrono::microseconds(control_rate_us),
        [this]() { this->policy_callback(); }
    );

    return result;
};

absl::Status ONNXDriver::initialize_session() {
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    session_ptr = std::make_unique<Ort::Session>(*env, onnx_model_path.c_str(), session_options);
    if (!session_ptr) {
        return absl::InternalError("Policy Interface: Failed to create ONNX session");
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
        return absl::InternalError("Policy Interface: Expected 1 input and 1 output tensor");
    }
    input_tensor_size = vector_product(input_shapes[0]);
    output_tensor_size = vector_product(output_shapes[0]);
    policy_input.resize(input_tensor_size);
    policy_output.resize(output_tensor_size);
    observation.resize(input_tensor_size);

    session_initialized = true;
    return absl::OkStatus();
};

absl::Status ONNXDriver::initialize_thread() {
    absl::Status result;

    if (!unitree_driver->is_thread_initialized())
        result.Update(unitree_driver->initialize_thread());

    if (executor.is_spinning())
        return absl::InternalError(
            "Policy Interface [initialize_thread]: Node is already spinning."
        );

    thread_initialized = true;
    executor.add_node(this->shared_from_this());
    executor_thread = std::jthread([this] { executor.spin(); });

    return absl::OkStatus();
};

absl::Status ONNXDriver::stop_thread() {
    if (!thread_initialized)
        return absl::FailedPreconditionError(
            "Policy Interface [stop_thread]: Node needs to be spinning."
        );

    if (executor.is_spinning())
        executor.cancel();

    executor_thread.join();
    return absl::OkStatus();
};

absl::Status ONNXDriver::set_control_mode(go2::constants::HighLevelControlMode mode) {
    std::lock_guard<std::mutex> lock(mutex);
    control_mode = mode;
    return absl::OkStatus();
};

const go2::constants::HighLevelControlMode ONNXDriver::get_control_mode() {
    std::lock_guard<std::mutex> lock(mutex);
    return control_mode;
};

absl::Status ONNXDriver::set_command(const constants::Vector3<float>& new_command) {
    std::lock_guard<std::mutex> lock(mutex);
    command = new_command;
    return absl::OkStatus();
};

absl::Status ONNXDriver::set_command(const std::array<float, 3>& new_command) {
    std::lock_guard<std::mutex> lock(mutex);
    command = Eigen::Map<const constants::Vector3<float>>(new_command.data());
    return absl::OkStatus();
};

const constants::Vector3<float> ONNXDriver::get_command() {
    std::lock_guard<std::mutex> lock(mutex);
    return command;
};

absl::Status ONNXDriver::set_master_gain(float gain) {
    std::lock_guard<std::mutex> lock(mutex);
    master_gain = std::clamp(gain, 0.0f, 1.0f);
    if (master_gain < 0.0f || master_gain > 1.0f) {
        return absl::InvalidArgumentError("Policy Interface [set_master_gain]: Gain clamped between 0.0 and 1.0.");
    }
    return absl::OkStatus();
};

const float ONNXDriver::get_master_gain() {
    std::lock_guard<std::mutex> lock(mutex);
    return master_gain;
};

const std::vector<float> ONNXDriver::get_policy_output() {
    std::lock_guard<std::mutex> lock(mutex);
    return policy_output;
};

const Eigen::Vector<float, Eigen::Dynamic> ONNXDriver::get_observation() {
    std::lock_guard<std::mutex> lock(mutex);
    return observation;
};

const bool ONNXDriver::is_initialized() const {
    return session_initialized;
};

const bool ONNXDriver::is_thread_initialized() const {
    return thread_initialized;
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
        return absl::InternalError("Failed to get output tensor");
    }

    return absl::OkStatus();
};

absl::Status ONNXDriver::make_observation() {
    // Get Measurements:
    std::optional<Go2State> state = unitree_driver->get_state();
    if (!state)
        return absl::InternalError("Policy Interface: Failed to get state from Unitree driver");

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
    go2::constants::ActionVector<float> previous_actions = Eigen::Map<go2::constants::ActionVector<float>>(policy_output.data());

    // Projected Gravity:
    Eigen::Quaternion<float> quaternion(
        quaternion_measurement(0), quaternion_measurement(1), quaternion_measurement(2), quaternion_measurement(3)
    );
    quaternion.normalize();
    Eigen::Matrix3<float> rotation = quaternion.toRotationMatrix();
    constants::Vector3<float> projected_gravity = rotation.transpose() * constants::Vector3<float>(0.0f, 0.0f, -1.0f);
    
    // Velocity Commands:
    constants::Vector3<float> commands = command;
    
    // Set Observation:
    observation << gyroscope_measurement,
                    projected_gravity,
                    joint_positions - default_position,
                    joint_velocities,
                    previous_actions,
                    commands;

    // Set Input Tensor:
    for(size_t i = 0; i < input_tensor_size; ++i) {
        policy_input[i] = observation(i);
    }

    return absl::OkStatus();
};

Go2Command ONNXDriver::policy_command() {
    go2::constants::ActionVector<float> actions = Eigen::Map<go2::constants::ActionVector<float>>(policy_output.data());
    go2::constants::ActionVector<float> setpoints = default_setpoints + master_gain * action_scale * actions;
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

void ONNXDriver::policy_callback() {
    std::lock_guard<std::mutex> lock(mutex);

    std::ignore = make_observation();
    std::ignore = inference_policy();

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
