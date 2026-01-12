#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <optional>

#include "absl/status/status.h"

#include "rclcpp/rclcpp.hpp"


class DigitDriver : public rclcpp::Node {
    public:
        /**
         * @brief Construct a new Digit Driver object
         * 
         */
        DigitDriver();

        /**
         * @brief Destroy the Digit Driver object
         * 
         */
        ~DigitDriver();

        // Disable copy and move semantics:
        DigitDriver(const DigitDriver&) = delete;
        DigitDriver& operator=(const DigitDriver&) = delete;
        DigitDriver(DigitDriver&&) = delete;
        DigitDriver& operator=(DigitDriver&&) = delete;

        /**
         * @brief Initializes the driver by setting the command to the current robot state and setting up the command publishing timer.
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
         * @brief Gets the latest state message received from the robot.
         * @return std::optional<DigitState> The latest state message, or std::nullopt if no message has been received or an error occured.
         */
        std::optional<DigitState> get_state();

        /**
         * @brief Updates the command to be sent to the robot.
         * @param new_command The new command to send.
         * @return absl::Status OkStatus on success, or an error if the command is invalid.
         */
        absl::Status update_command(const DigitCommand& new_command);

        /**
         * @brief Checks if the driver has been initialized.
         * @return true if the driver is initialized, false otherwise.
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
         * @brief Initializes the internal command structure with default values.
         */
        void initialize_command();

        /**
         * @brief Callback function for the robot state subscriber.
         * @param msg The received DigitState message.
         */
        void state_callback(const DigitState::SharedPtr msg);

        /**
         * @brief Callback function for the command publishing timer.
         */
        void command_callback();

        // ROS 2 Subscriber, Publisher, and Timer:
        rclcpp::Subscription<DigitState>::SharedPtr state_subscription_;
        rclcpp::Publisher<DigitCommand>::SharedPtr command_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Digit State:
        std::mutex state_mutex;
        DigitState::SharedPtr latest_state_message;

        // Digit Command:
        std::mutex command_mutex;
        DigitCommand command;

        // ROS2 Thread Executor:
        rclcpp::executors::MultiThreadedExecutor executor;
        std::jthread executor_thread;

        // Member Variables:
        bool initialized = false;
        bool thread_initialized = false;
        int control_rate_us;
        static constexpr int initialize_loop_rate_hz = 100;
};