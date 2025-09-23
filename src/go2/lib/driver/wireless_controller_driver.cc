#include "src/go2/lib/driver/wireless_controller_driver.h"

#include <memory>
#include <mutex>
#include <thread>
#include <optional>
#include <chrono>
#include <algorithm>

#include "absl/status/status.h"

#include "rclcpp/rclcpp.hpp"

#include "src/go2/msgs/unitree_go_msgs.h"


WirelessControllerDriver::WirelessControllerDriver() : Node("wireless_controller_driver") {
    // Set Poll Rate:
    this->declare_parameter<int>("poll_rate_us", 20000);
    this->poll_rate_us = this->get_parameter("poll_rate_us").as_int();

    // Clamp Values:
    this->smoothing_factor = std::clamp(this->smoothing_factor, 0.0f, 1.0f);
    this->deadzone = std::clamp(this->deadzone, 0.0f, 1.0f);

    // Create default QoS Profile:
    rclcpp::QoS qos_profile(10);
    qos_profile.best_effort();

    // Create Subscriber Node:
    subscription_ = this->create_subscription<unitree_go::msg::WirelessController>(
        "/wirelesscontroller",
        qos_profile,
        [this](const unitree_go::msg::WirelessController::SharedPtr msg) {
            this->callback(msg);
        }
    );
}

WirelessControllerDriver::~WirelessControllerDriver() {
    if (executor.is_spinning())
        executor.cancel();
}

absl::Status WirelessControllerDriver::initialize_thread() {
    if (executor.is_spinning())
        return absl::InternalError(
            "Wireless Controller Driver [initialize_thread]: Node is already spinning."
        );

    thread_initialized = true;
    executor.add_node(this->shared_from_this());
    executor_thread = std::jthread([this] { executor.spin(); });

    return absl::OkStatus();
}

absl::Status WirelessControllerDriver::stop_thread() {
    if (!thread_initialized)
        return absl::FailedPreconditionError(
            "Wireless Controller Driver [stop_thread]: Node needs to be spinning."
        );

    if (executor.is_spinning())
        executor.cancel();

    executor_thread.join();
    return absl::OkStatus();
}

void WirelessControllerDriver::callback(const unitree_go::msg::WirelessController::SharedPtr msg) {
    const std::lock_guard<std::mutex> lock(mutex_);

    button_value_ = msg->keys;

    left_stick_x_ = process_axis(left_stick_x_, msg->lx);
    left_stick_y_ = process_axis(left_stick_y_, msg->ly);
    right_stick_x_ = process_axis(right_stick_x_, msg->rx);
    right_stick_y_ = process_axis(right_stick_y_, msg->ry);
}

float apply_deadzone_and_rescale(const float value, const float deadzone_threshold) {
    if (std::abs(value) < deadzone_threshold)
        return 0.0f;
    const float sign = (value > 0.0f) ? 1.0f : -1.0f;
    const float remapped_value = (std::abs(value) - deadzone_threshold) / (1.0f - deadzone_threshold);
    return std::clamp(remapped_value * sign, -1.0f, 1.0f);
}

float WirelessControllerDriver::process_axis(float current_value, float new_value) const {
    constexpr float zero_threshold = 0.075f;
    const float target_value = apply_deadzone_and_rescale(new_value, deadzone);
    const float lerp_value = std::lerp(current_value, target_value, smoothing_factor);
    const float smoothed_value = 
        (target_value == 0.0f && std::abs(lerp_value) < zero_threshold)
        ? 0.0f
        : lerp_value;
    return std::clamp(smoothed_value, -1.0f, 1.0f);
}

bool WirelessControllerDriver::is_pressed(Button b) const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return (button_value_ >> static_cast<uint16_t>(b)) & 1;
}

float WirelessControllerDriver::get_left_stick_x() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return left_stick_x_;
}

float WirelessControllerDriver::get_left_stick_y() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return left_stick_y_;
}

float WirelessControllerDriver::get_right_stick_x() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return right_stick_x_;
}

float WirelessControllerDriver::get_right_stick_y() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return right_stick_y_;
}
