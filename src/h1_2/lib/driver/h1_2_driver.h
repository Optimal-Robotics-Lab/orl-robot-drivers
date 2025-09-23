#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <optional>

#include "absl/status/status.h"

#include "rclcpp/rclcpp.hpp"

#include "src/h1_2/msgs/unitree_hg_msgs.h"


using H1_2_State = unitree_hg::msg::LowState;
using H1_2_Command = unitree_hg::msg::LowCmd;

/**
 * @class H1_2_Driver
 * @brief A class to interface with Unitree's H1-2 robot, handling state subscription and command publishing.
 *
 * This class encapsulates the ROS 2 node functionality for controlling the H1-2 robot.
 * It provides methods to initialize the driver, manage the ROS 2 executor in a separate thread,
 * get the latest robot state, and send commands to the robot.
 */
class H1_2_Driver : public rclcpp::Node {
    public:
        /**
         * @brief Construct a new H1-2 Driver object
         */
        H1_2_Driver();
        
        /**
         * @brief Destroy the H1-2 Driver object, ensuring proper cleanup of resources.
         */
        ~H1_2_Driver();

        // Disable copy and move semantics to prevent accidental duplication
        H1_2_Driver(const H1_2_Driver&) = delete;
        H1_2_Driver& operator=(const H1_2_Driver&) = delete;
        H1_2_Driver(H1_2_Driver&&) = delete;
        H1_2_Driver& operator=(H1_2_Driver&&) = delete;

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
         * @return std::optional<H1-2State> The latest state message, or std::nullopt if no message has been received or an error occured.
         */
        std::optional<H1_2_State> get_state();

        /**
         * @brief Updates the command to be sent to the robot.
         * @param new_command The new command to send.
         * @return absl::Status OkStatus on success, or an error if the command is invalid.
         */
        absl::Status update_command(const H1_2_Command& new_command);

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
         * @param msg The received H1-2State message.
         */
        void state_callback(const H1_2_State::SharedPtr msg);

        /**
         * @brief Callback function for the command publishing timer.
         */
        void command_callback();

        // ROS 2 Subscriber, Publisher, and Timer:
        rclcpp::Subscription<H1_2_State>::SharedPtr state_subscription_;
        rclcpp::Publisher<H1_2_Command>::SharedPtr command_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        // H1-2 State:
        std::mutex state_mutex;
        H1_2_State::SharedPtr latest_state_message;

        // H1-2 Command:
        std::mutex command_mutex;
        H1_2_Command command;

        // ROS2 Thread Executor:
        rclcpp::executors::MultiThreadedExecutor executor;
        std::jthread executor_thread;

        // Member Variables:
        bool initialized = false;
        bool thread_initialized = false;
        int control_rate_us;
        static constexpr int initialize_loop_rate_hz = 100;
};