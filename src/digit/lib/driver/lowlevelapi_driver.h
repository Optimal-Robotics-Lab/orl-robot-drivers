#pragma once

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

    private:
        void state_callback(const llapi_observation_t& observation);
        void command_callback(const DigitCommand::ConstSharedPtr msg);
        void internal();

        // Command Cache:
        std::mutex command_mutex_;
        bool has_command_ = false;
        DigitCommand::ConstSharedPtr latest_command_;

        // Agility llapi Observation and Command Structs:
        llapi_observation_t observation_ = { 0 }; 
        llapi_command_t command_ = { 0 };

        // Threading
        std::jthread llapi_thread_;
        std::atomic<bool> keep_running_{false};

        // ROS 2 Subscriber, Publisher, and Timer:
        rclcpp::Publisher<DigitState>::SharedPtr state_publisher_;
        rclcpp::Subscription<DigitCommand>::SharedPtr command_subscriber_;

        // Member Variables:
        bool initialized = false;
        int control_rate_us;
        std::string publisher_address;
}