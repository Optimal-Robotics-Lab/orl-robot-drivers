#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <optional>

#include "absl/status/status.h"

#include "rclcpp/rclcpp.hpp"

#include "src/go2/msgs/unitree_go_msgs.h"
#include "nav_msgs/msg/odometry.hpp"


using Go2State = unitree_go::msg::LowState;
using Go2Command = unitree_go::msg::LowCmd;

class Go2Driver : public rclcpp::Node {
    public:
        /**
         * @brief Construct a new Go2 Driver object
         * 
         */
        Go2Driver();

        /**
         * @brief Destroy the Go2 Driver object
         * 
         */
        ~Go2Driver();

        // Disable copy and move semantics:
        Go2Driver(const Go2Driver&) = delete;
        Go2Driver& operator=(const Go2Driver&) = delete;
        Go2Driver(Go2Driver&&) = delete;
        Go2Driver& operator=(Go2Driver&&) = delete;

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
         * @return std::optional<Go2State> The latest state message, or std::nullopt if no message has been received or an error occured.
         */
        std::optional<Go2State> get_state();

        /**
         * @brief Gets the latest odometry information from the Invariant EKF State Estimator.
         * @return std::optional<nav_msgs::msg::Odometry> The latest odometry information, or std::nullopt if no state message has been received or an error occurred.
         */
        [[nodiscard]] std::optional<nav_msgs::msg::Odometry> get_state_estimation();

        /**
         * @brief Validates if the robot has all feet on the ground based on the low-level state.
         * Required to start the State Estimator.
         */
        [[nodiscard]] bool feet_in_contact();

        /**
         * @brief Checks if the state estimation is available.
         * @return true if state estimation is available, false otherwise.
         */
        [[nodiscard]] bool has_state_estimation() const {
            return state_estimation_received_.load(std::memory_order_relaxed);
        }

        /**
         * @brief Updates the command to be sent to the robot.
         * @param new_command The new command to send.
         * @return absl::Status OkStatus on success, or an error if the command is invalid.
         */
        absl::Status update_command(const Go2Command& new_command);

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
         * @param msg The received Go2State message.
         */
        void state_callback(const Go2State::ConstSharedPtr msg);
        
        /**
         * @brief Callback function for the state estimator subscriber.
         * @param msg The received Odometry message.
         */
        void state_estimator_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

        /**
         * @brief Callback function for the command publishing timer.
         */
        void command_callback();

        // ROS 2 Subscriber, Publisher, and Timer:
        rclcpp::Subscription<Go2State>::SharedPtr state_subscription_;
        rclcpp::Publisher<Go2Command>::SharedPtr command_publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_estimator_subscription_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Go2 State:
        std::mutex state_mutex;
        Go2State::ConstSharedPtr latest_state_message;

        // Go2 Command:
        std::mutex command_mutex;
        Go2Command command;

        // Go2 State Estimation:
        std::mutex state_estimator_mutex;
        nav_msgs::msg::Odometry::ConstSharedPtr latest_state_estimator_message;
        std::atomic<bool> state_estimation_received_{false};

        // ROS2 Thread Executor:
        rclcpp::executors::MultiThreadedExecutor executor;
        std::jthread executor_thread;

        // Member Variables:
        bool initialized = false;
        bool thread_initialized = false;
        int control_rate_us;
        static constexpr int initialize_loop_rate_hz = 100;
};