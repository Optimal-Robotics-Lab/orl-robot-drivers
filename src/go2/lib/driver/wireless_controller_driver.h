#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <optional>

#include "absl/status/status.h"

#include "rclcpp/rclcpp.hpp"

#include "src/go2/msgs/unitree_go_msgs.h"

/**
 * @class WirelessControllerDriver
 * @brief A ROS2 node that subscribes to Unitree Go2 wireless controller data,
 * processes it, and provides a thread-safe interface to access the state.
 */
class WirelessControllerDriver : public rclcpp::Node {
public:
    enum class Button : uint16_t {
        R1 = 0, 
        L1 = 1, 
        START = 2,
        SELECT = 3,
        R2 = 4, 
        L2 = 5,
        F1 = 6,
        F2 = 7,
        A = 8,
        B = 9,
        X = 10,
        Y = 11,
        UP = 12,
        RIGHT = 13,
        DOWN = 14,
        LEFT = 15
    };

    /**
     * @brief Construct a new Wireless Controller Node object
     */
    explicit WirelessControllerDriver();

    /**
     * @brief Destroy the Wireless Controller Node object
     */
    ~WirelessControllerDriver();

    // Disable copy and move semantics to prevent accidental duplication
    WirelessControllerDriver(const WirelessControllerDriver&) = delete;
    WirelessControllerDriver& operator=(const WirelessControllerDriver&) = delete;
    WirelessControllerDriver(WirelessControllerDriver&&) = delete;
    WirelessControllerDriver& operator=(WirelessControllerDriver&&) = delete;

    /**
     * @brief Initializes the ROS 2 executor and starts spinning in a separate thread.
     * @return absl::Status OkStatus on success, or an error status if already spinning.
     */
    absl::Status initialize_thread();

    /**
     * @brief Shuts down the ROS 2 executor and cleans up resources.
     * @return absl::Status OkStatus on success, or an error status if not spinning.
     */
    absl::Status stop_thread();

    // Accessor methods for button and stick states
    [[nodiscard]] bool is_pressed(Button b) const;
    [[nodiscard]] float get_left_stick_x() const;
    [[nodiscard]] float get_left_stick_y() const;
    [[nodiscard]] float get_right_stick_x() const;
    [[nodiscard]] float get_right_stick_y() const;

    /**
     * @brief Checks if the ROS 2 executor thread has been initialized.
     * @return true if the executor thread is initialized, false otherwise.
     */
    const bool is_thread_initialized() const {
        return thread_initialized;
    }

private:
    /**
     * @brief Callback function for processing incoming messages
     * @param msg The incoming message
     */
    void callback(const unitree_go::msg::WirelessController::SharedPtr msg);

    /**
     * @brief Process the axis values with smoothing and deadzone
     * @param current_value The current axis value
     * @param new_value The new axis value
     * @return The processed axis value
     */
    [[nodiscard]] float process_axis(float current_value, float new_value) const;

    /**
     * @brief Subscription for wireless controller messages
     */
    rclcpp::Subscription<unitree_go::msg::WirelessController>::SharedPtr subscription_;

    // State variables:
    uint16_t button_value_{0};
    float left_stick_x_{0.0f};
    float left_stick_y_{0.0f};
    float right_stick_x_{0.0f};
    float right_stick_y_{0.0f};
    
    // Configuration parameters:
    int poll_rate_us{20000};
    float smoothing_factor{0.25};
    float deadzone{0.01};

    // Thread variables:
    bool thread_initialized{false};
    mutable std::mutex mutex_;
    rclcpp::executors::MultiThreadedExecutor executor;
    std::jthread executor_thread;

};