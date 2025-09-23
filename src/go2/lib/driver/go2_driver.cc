#include "src/go2/lib/driver/go2_driver.h"

#include <memory>
#include <mutex>
#include <thread>
#include <optional>
#include <chrono>
#include <algorithm>

#include "absl/status/status.h"

#include "rclcpp/rclcpp.hpp"

#include "src/go2/msgs/unitree_go_msgs.h"
#include "src/go2/lib/utils/constants.h"
#include "src/go2/lib/utils/utilities.h"
#include "src/go2/lib/utils/crc.h"

using namespace std::chrono_literals;

using Go2State = unitree_go::msg::LowState;
using Go2Command = unitree_go::msg::LowCmd;
using namespace robot;

Go2Driver::Go2Driver() : Node("go2_driver") {
    // Set Command Control Rate:
    declare_parameter("control_rate_us", 2000);
    this->control_rate_us = this->get_parameter("control_rate_us").as_int();

    // Create default QoS Profile:
    rclcpp::QoS qos_profile(10);
    qos_profile.best_effort();

    // Create Subscriber Node:
    state_subscription_ = this->create_subscription<Go2State>(
        "/lowstate",
        qos_profile,
        [this](const Go2State::SharedPtr msg) {
            this->state_callback(msg);
        }
    );

    // Create Publisher Node:
    command_publisher_ = this->create_publisher<Go2Command>(
        "/lowcmd",
        qos_profile
    );

    this->initialize_command();
}

Go2Driver::~Go2Driver() {
    if (executor.is_spinning())
        executor.cancel();
}

absl::Status Go2Driver::initialize() {
    if (!executor.is_spinning())
        return absl::FailedPreconditionError(
            "Go2 Driver [initialize]: Node needs to be spinning."
        );

    // Get Current Robot State:
    std::optional<Go2State> state;
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
            "Go2 Driver [initialize]: Go2 State failed to return a value."
        );

    // Initialize Command with Current Joint Positions:
    Go2Command default_command = go2::utilities::default_position_command();
    for (size_t i = 0; i < go2::constants::num_joints; ++i) {
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

absl::Status Go2Driver::initialize_thread() {
    if (executor.is_spinning())
        return absl::InternalError(
            "Go2 Driver [initialize_thread]: Node is already spinning."
        );

    thread_initialized = true;
    executor.add_node(this->shared_from_this());
    executor_thread = std::jthread([this] { executor.spin(); });

    return absl::OkStatus();
}

absl::Status Go2Driver::stop_thread() {
    if (!thread_initialized)
        return absl::FailedPreconditionError(
            "Go2 Driver [stop_thread]: Node needs to be spinning."
        );

    if (executor.is_spinning())
        executor.cancel();

    executor_thread.join();
    return absl::OkStatus();
}

std::optional<Go2State> Go2Driver::get_state() {
    std::lock_guard<std::mutex> lock(state_mutex);
    if (!latest_state_message)
        return std::nullopt;

    return *latest_state_message;
}

absl::Status Go2Driver::update_command(const Go2Command& new_command) {
    std::lock_guard<std::mutex> lock(command_mutex);

    for (size_t i = 0; i < go2::constants::num_joints; ++i) {
        auto& motor_command = command.motor_cmd[i];
        const auto& new_motor_command = new_command.motor_cmd[i];

        // Set Header Fields:
        motor_command.mode = new_motor_command.mode;

        // Set Setpoints and Gains:
        motor_command.q = new_motor_command.q;
        motor_command.dq = std::clamp(
            new_motor_command.dq,
            -go2::constants::qd_bounds[i],
            go2::constants::qd_bounds[i]
        );
        motor_command.tau = std::clamp(
            new_motor_command.tau,
            -go2::constants::torque_bounds[i],
            go2::constants::torque_bounds[i]
        );
        motor_command.kp = new_motor_command.kp;
        motor_command.kd = new_motor_command.kd;
    }

    return absl::OkStatus();
}

void Go2Driver::initialize_command() {
    std::lock_guard<std::mutex> lock(command_mutex);
    command = robot::go2::utilities::default_position_command();
}

void Go2Driver::state_callback(const Go2State::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex);
    latest_state_message = msg;
}

void Go2Driver::command_callback() {
    if (!initialized) return;

    Go2Command command_to_publish;
    {
        std::lock_guard<std::mutex> lock(command_mutex);
        command_to_publish = command;
        
        // Set CRC:
        go2::crc::set_crc(command_to_publish);
    }

    command_publisher_->publish(command_to_publish);
}
