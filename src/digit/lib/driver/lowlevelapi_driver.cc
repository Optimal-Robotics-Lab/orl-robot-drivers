#include "src/digit/lib/driver/lowlevelapi_driver.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <optional>

#include "absl/status/status.h"

#include "rclcpp/rclcpp.hpp"

#include "lowlevelapi.h"

#include "src/digit/lib/utils/constants.h"
#include "src/digit/lib/utils/utilities.h"
#include "src/digit/msgs/digit_msgs.h"

using DigitState = digit_interface::msg::DigitState;
using DigitCommand = digit_interface::msg::DigitCommand;


LowLevelApiDriver::LowLevelApiDriver() 
    : Node("lowlevelapi_driver"),
    latest_command_(std::make_shared<DigitCommand>(robot::digit::utilities::initialize_command()))
{
    declare_parameter("control_rate_us", 1000);
    declare_parameter("publisher_address", "127.0.0.1");
    this->control_rate_us = this->get_parameter("control_rate_us").as_int();
    this->publisher_address = this->get_parameter("publisher_address").as_string();

    // Create default QoS Profile:
    rclcpp::QoS qos_profile(10);
    qos_profile.best_effort();

    // Create Subscriber Node:
    command_subscriber_ = this->create_subscription<DigitCommand>(
        "/command",
        qos_profile,
        [this](std::shared_ptr<const DigitCommand> msg) {
            this->command_callback(msg);
        }
    );

    // Create Publisher Node:
    state_publisher_ = this->create_publisher<DigitState>(
        "/state",
        qos_profile
    );
}

LowLevelApiDriver::~LowLevelApiDriver() {
    this->keep_running_ = false;
    if (llapi_thread_.joinable()) {
        llapi_thread_.join();
    }
}

absl::Status LowLevelApiDriver::initialize() {
    llapi_init(this->publisher_address.c_str());

    auto start = std::chrono::steady_clock::now();
    while (!llapi_connected()) {
        llapi_observation_t obs;
        llapi_get_observation(&obs);
        
        if (std::chrono::steady_clock::now() - start > std::chrono::seconds(5))
            return absl::DeadlineExceededError("LowLevelApi Driver: Failed to connect to Digit.");

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const llapi_limits_t* limits_ptr = llapi_get_limits();
    if (limits_ptr) {
        this->limits_ = *limits_ptr;
    }
    else {
        this->limits_ = std::nullopt;
        RCLCPP_WARN(get_logger(), "Could not fetch limits (returned NULL).");
    }

    this->keep_running_ = true;
    this->llapi_thread_ = std::jthread([this]() {
        auto next_time = std::chrono::steady_clock::now();
        const auto period = std::chrono::microseconds(this->control_rate_us);
        while (this->keep_running_) {
            next_time += period;
            const auto status = this->internal();
            if (!status.ok()) {
                RCLCPP_ERROR_STREAM_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000, 
                    "LowLevelApi Driver: " << status.message()
                );
                this->keep_running_ = false;
                break;
            }
            const auto now = std::chrono::steady_clock::now();
            if (now > next_time) {
                const auto overrun = std::chrono::duration_cast<std::chrono::microseconds>(now - next_time);
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000, 
                    "LowLevelApi Driver: Loop Overrun! Exceeded cycle time by %ld us. Resetting schedule.", 
                    overrun.count()
                );
                next_time = now;
                std::this_thread::yield();
            }
            else {
                std::this_thread::sleep_until(next_time);
            }
        }
     });

    return absl::OkStatus();
}

absl::Status LowLevelApiDriver::set_command(const DigitCommand::ConstSharedPtr command) {
    this->command_callback(command);
    return absl::OkStatus();
}

std::optional<DigitState> LowLevelApiDriver::get_state() {
    llapi_observation_t snapshot = {};

    {
        std::scoped_lock lock(this->state_mutex_);
        if (!this->latest_observation_)
            return std::nullopt;
        
        snapshot = this->latest_observation_.value();
    }

    DigitState state = llapi_observation_to_msg(snapshot);
    return state;
}

void LowLevelApiDriver::state_callback(const llapi_observation_t& observation) {
    DigitState msg = llapi_observation_to_msg(observation);
    state_publisher_->publish(msg);
}

DigitState LowLevelApiDriver::llapi_observation_to_msg(const llapi_observation_t& observation) {
    DigitState msg;

    msg.time = observation.time;
    msg.error = observation.error;
    msg.battery_charge = observation.battery_charge;

    std::ranges::copy(observation.base.translation, msg.base.translation.begin());
    std::ranges::copy(observation.base.linear_velocity, msg.base.linear_velocity.begin());
    std::ranges::copy(observation.base.angular_velocity, msg.base.angular_velocity.begin());
    msg.base.orientation[0] = observation.base.orientation.w;
    msg.base.orientation[1] = observation.base.orientation.x;
    msg.base.orientation[2] = observation.base.orientation.y;
    msg.base.orientation[3] = observation.base.orientation.z;

    std::ranges::copy(observation.imu.angular_velocity, msg.imu.angular_velocity.begin());
    std::ranges::copy(observation.imu.linear_acceleration, msg.imu.linear_acceleration.begin());
    std::ranges::copy(observation.imu.magnetic_field, msg.imu.magnetic_field.begin());
    msg.imu.orientation[0] = observation.imu.orientation.w;
    msg.imu.orientation[1] = observation.imu.orientation.x;
    msg.imu.orientation[2] = observation.imu.orientation.y;
    msg.imu.orientation[3] = observation.imu.orientation.z;

    std::ranges::copy(observation.motor.position, msg.motor.position.begin());
    std::ranges::copy(observation.motor.velocity, msg.motor.velocity.begin());
    std::ranges::copy(observation.motor.torque, msg.motor.torque.begin());

    std::ranges::copy(observation.joint.position, msg.joint.position.begin());
    std::ranges::copy(observation.joint.velocity, msg.joint.velocity.begin());

    return msg;
}

void LowLevelApiDriver::command_callback(const DigitCommand::ConstSharedPtr msg) {
    std::scoped_lock lock(this->command_mutex_);
    this->latest_command_ = msg;
}

absl::Status LowLevelApiDriver::internal() {
    // Check if disconnected:
    if (!llapi_connected()) {
        // Attempt to send damping command. But this error should kill the node.
        llapi_send_command(&this->DAMPING_COMMAND);

        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000, 
            "LowLevelApi Driver: Disconnected from robot."
        );
        return absl::InternalError("LowLevelApi Driver: Disconnected from robot.");
    }

    constexpr auto LLAPI_ERROR = -1;
    constexpr auto LLAPI_NO_NEW_DATA = 0;

    auto result = llapi_get_observation(&this->observation_);
    if (result == LLAPI_NO_NEW_DATA) return absl::OkStatus();
    if (result == LLAPI_ERROR) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000, 
            "LowLevelApi Driver: Error receiving observation."
        );
        return absl::InternalError("LowLevelApi Driver: Error receiving observation.");
    }

    {
        std::scoped_lock lock(this->state_mutex_);
        this->latest_observation_ = this->observation_;
    }

    const DigitCommand::ConstSharedPtr command_snapshot = [this]() -> DigitCommand::ConstSharedPtr {
        std::scoped_lock lock(this->command_mutex_);
        return this->latest_command_;
    }();
    if (!command_snapshot) {
        this->state_callback(this->observation_);
        llapi_send_command(&this->DAMPING_COMMAND);
        return absl::InternalError("LowLevelApi Driver: Command received is null.");
    }

    for (std::size_t i = 0; const auto& cmd : command_snapshot->motors) {
        if (i >= robot::digit::constants::num_motors) 
            return absl::InternalError("LowLevelApi Driver: Exceeded max motors.");

        this->command_.motors[i].torque = 
            cmd.stiffness * (cmd.position - this->observation_.motor.position[i])
            + cmd.torque;

        this->command_.motors[i].velocity = cmd.velocity;
        this->command_.motors[i].damping = cmd.damping;
        
        ++i;
    }
    
    this->command_.fallback_opmode = command_snapshot->fallback_opmode;
    this->command_.apply_command = command_snapshot->apply_command;

    llapi_send_command(&this->command_);

    this->state_callback(this->observation_);
    return absl::OkStatus();
}
