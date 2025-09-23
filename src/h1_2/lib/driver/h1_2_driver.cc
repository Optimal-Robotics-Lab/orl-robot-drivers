#include "src/h1_2/lib/driver/h1_2_driver.h"

#include <memory>
#include <mutex>
#include <thread>
#include <optional>
#include <chrono>
#include <algorithm>

#include "absl/status/status.h"

#include "rclcpp/rclcpp.hpp"

#include "src/h1_2/msgs/unitree_hg_msgs.h"
#include "src/h1_2/lib/utils/constants.h"
#include "src/h1_2/lib/utils/utilities.h"
#include "src/h1_2/lib/utils/crc.h"

using namespace std::chrono_literals;

using H1_2_State = unitree_hg::msg::LowState;
using H1_2_Command = unitree_hg::msg::LowCmd;
using namespace robot;


H1_2_Driver::H1_2_Driver() : Node("h1_2_driver") {
    // Set Command Control Rate:
    declare_parameter("control_rate_us", 2000);
    this->control_rate_us = this->get_parameter("control_rate_us").as_int();

    // Create default QoS Profile:
    rclcpp::QoS qos_profile(10);
    qos_profile.best_effort();

    // Create Subscriber Node:
    state_subscription_ = this->create_subscription<H1_2_State>(
        "/lowstate",
        qos_profile,
        [this](const H1_2_State::SharedPtr msg) {
            this->state_callback(msg);
        }
    );

    // Create Publisher Node:
    command_publisher_ = this->create_publisher<H1_2_Command>(
        "/lowcmd",
        qos_profile
    );

    this->initialize_command();
}

H1_2_Driver::~H1_2_Driver() {
    if (executor.is_spinning())
        executor.cancel();
}

absl::Status H1_2_Driver::initialize() {
    if (!executor.is_spinning())
        return absl::FailedPreconditionError(
            "H1-2 Driver [initialize]: Node needs to be spinning."
        );

    // Get Current Robot State:
    std::optional<H1_2_State> state;
    rclcpp::WallRate loop_rate(initialize_loop_rate_hz);
    const auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        state = this->get_state();
        if (state)
            break;
        loop_rate.sleep();
        if (std::chrono::steady_clock::now() - start_time > 1s)
            break;
    }

    if (!state)
        return absl::InternalError(
            "H1-2 Driver [initialize]: H1-2 State failed to return a value."
        );

    // Initialize Command with Current Joint Positions:
    H1_2_Command default_command = h1_2::utilities::default_position_command();
    for (size_t i = 0; i < h1_2::constants::num_joints; ++i) {
        const auto& motor_state = state->motor_state[i];
        default_command.motor_cmd[i].q = motor_state.q;
    }

    absl::Status status = this->update_command(default_command);
    if (!status.ok())
        return status;

    initialized = true;
    timer_ = create_wall_timer(
        std::chrono::microseconds(control_rate_us),
        [this]() { this->command_callback(); }
    );
    return absl::OkStatus();
}

absl::Status H1_2_Driver::initialize_thread() {
    if (executor.is_spinning())
        return absl::InternalError(
            "H1-2 Driver [initialize_thread]: Node is already spinning."
        );

    thread_initialized = true;
    executor.add_node(this->shared_from_this());
    executor_thread = std::jthread([this] { executor.spin(); });

    return absl::OkStatus();
}

absl::Status H1_2_Driver::stop_thread() {
    if (!thread_initialized)
        return absl::FailedPreconditionError(
            "H1-2 Driver [stop_thread]: Node needs to be spinning."
        );

    if (executor.is_spinning())
        executor.cancel();

    executor_thread.join();
    return absl::OkStatus();
}

std::optional<H1_2_State> H1_2_Driver::get_state() {
    std::lock_guard<std::mutex> lock(state_mutex);
    if (!latest_state_message)
        return std::nullopt;

    return *latest_state_message;
}

absl::Status H1_2_Driver::update_command(const H1_2_Command& new_command) {
    std::lock_guard<std::mutex> lock(command_mutex);

    for (size_t i = 0; i < h1_2::constants::num_joints; ++i) {
        auto& motor_command = command.motor_cmd[i];
        const auto& new_motor_command = new_command.motor_cmd[i];

        // Set Header Fields:
        motor_command.mode = new_motor_command.mode;

        // Set Setpoints and Gains:
        motor_command.q = new_motor_command.q;
        motor_command.dq = std::clamp(
            new_motor_command.dq,
            -h1_2::constants::qd_bounds[i],
            h1_2::constants::qd_bounds[i]
        );
        motor_command.tau = std::clamp(
            new_motor_command.tau,
            -h1_2::constants::torque_bounds[i],
            h1_2::constants::torque_bounds[i]
        );
        motor_command.kp = new_motor_command.kp;
        motor_command.kd = new_motor_command.kd;
    }

    return absl::OkStatus();
}

void H1_2_Driver::initialize_command() {
    std::lock_guard<std::mutex> lock(command_mutex);

    // Header Fields:
    command.mode_pr = static_cast<uint8_t>(h1_2::constants::AnkleControlType::PITCH_ROLL);
    command.mode_machine = static_cast<uint8_t>(h1_2::constants::ModeMachine::DEFAULT);

    for (size_t i = 0; i < h1_2::constants::num_joints; ++i) {
        auto& motor_command = command.motor_cmd[i];

        // Set Header Fields:
        motor_command.mode = static_cast<uint8_t>(h1_2::constants::ControlMode::ENABLE);

        // Set Setpoints and Gains:
        motor_command.q = h1_2::constants::default_position[i];
        motor_command.dq = 0.0;
        motor_command.tau = 0.0;
        motor_command.kp = 0.0;
        motor_command.kd = 0.0;
    }
}

void H1_2_Driver::state_callback(const H1_2_State::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex);
    latest_state_message = msg;
}

void H1_2_Driver::command_callback() {
    if (!initialized) return;

    H1_2_Command command_to_publish;
    {
        std::lock_guard<std::mutex> lock(command_mutex);
        command_to_publish = command;
        
        // Set CRC:
        h1_2::crc::set_crc(command_to_publish);
    }

    command_publisher_->publish(command_to_publish);
}
