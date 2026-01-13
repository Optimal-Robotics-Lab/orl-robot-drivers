#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <optional>

#include "absl/status/status.h"

#include "rclcpp/rclcpp.hpp"

#include "lowlevelapi.h"

#include "src/digit/lib/utils/constants.h"
#include "src/digit/msgs/digit_msgs.h"

using DigitState = digit_interface::msg::DigitState;
using DigitCommand = digit_interface::msg::DigitCommand;


class LowLevelApiDriver : public rclcpp::Node {
    public:
        /**
         * @brief Construct a ROS 2 Wrapper Node for Low Level Communication with Digit.
         */
        LowLevelApiDriver();

        /** 
         * @brief Destroy the Low Level API Driver object
         */
        ~LowLevelApiDriver();

        // Disable copy and move semantics:
        LowLevelApiDriver(const LowLevelApiDriver&) = delete;
        LowLevelApiDriver& operator=(const LowLevelApiDriver&) = delete;
        LowLevelApiDriver(LowLevelApiDriver&&) = delete;
        LowLevelApiDriver& operator=(LowLevelApiDriver&&) = delete;

        /** 
         * @brief Initializes the driver by setting up the Low-level API connection and starting the ROS2 executor thread.
         * @return absl::Status OkStatus on success. This cannot fail currently.
         */
        absl::Status initialize();

        /**
         * @brief Callback function to update command.
         * @param command Command to be sent to the Low-level API.
         */
        absl::Status update_command(const DigitCommand::ConstSharedPtr command);

        /**
         * @brief Retrieves the limits of the Low-level API if initialized.
         * @return std::optional<llapi_limits_t> The limits if initialized, std::nullopt otherwise.
        */
        std::optional<llapi_limits_t> get_limits() const {
            if (!this->initialized)
                return std::nullopt;

            return this->limits_;
        }

    private:
        void state_callback(const llapi_observation_t& observation);
        void command_callback(const DigitCommand::ConstSharedPtr msg);
        absl::Status internal();

        // Command Cache:
        std::mutex command_mutex_;
        bool has_command_ = false;
        DigitCommand::ConstSharedPtr latest_command_;

        // Agility llapi Observation and Command Structs:
        llapi_command_t command_ = {};
        llapi_observation_t observation_ = {};

        // Threading
        std::jthread llapi_thread_;
        std::atomic<bool> keep_running_{false};

        // ROS 2 Subscriber, Publisher, and Timer:
        rclcpp::Publisher<DigitState>::SharedPtr state_publisher_;
        rclcpp::Subscription<DigitCommand>::SharedPtr command_subscriber_;

        // Member Variables:
        bool initialized = false;
        std::optional<llapi_limits_t> limits_ = std::nullopt;
        int control_rate_us;
        std::string publisher_address;
};
