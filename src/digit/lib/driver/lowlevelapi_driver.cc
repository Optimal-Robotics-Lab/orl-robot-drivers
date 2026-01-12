LowLevelApiDriver::LowLevelApiDriver() : Node("lowlevelapi_driver") {
    // Set Command Control Rate:
    declare_parameter("control_rate_us", 1000);
    declare_parameter("publisher_address", "127.0.0.1");
    this->control_rate_us = this->get_parameter("control_rate_us").as_int();
    this->publisher_address = this->get_parameter("publisher_address").as_string();

    // Create default QoS Profile:
    rclcpp::QoS qos_profile(10);
    qos_profile.best_effort();

    // Create Subscriber Node:
    command_subscription_ = this->create_subscription<DigitCommand>(
        "/command",
        qos_profile,
        [this](std::shared_ptr<const Command> msg) {
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

    this->keep_running_ = true;
    this->llapi_thread_ = std::jthread([this]() {
        auto next_time = std::chrono::steady_clock::now();
        const auto period = std::chrono::microseconds(this->control_rate_us);
        while (this->keep_running_) {
            next_time += period;
            const auto status = this->internal();
            if (!status.ok()) {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000, 
                    "LowLevelApi Driver: %s", status.message().c_str()
                );
            }
            const auto now = std::chrono::steady_clock::now();
            if (now > next_time) {
                const auto overrun = std::chrono::duration_cast<std::chrono::microseconds>(now - next_time);
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000, 
                    "Loop Overrun! Exceeded cycle time by %ld us. Resetting schedule.", 
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

absl::Status LowLevelApiDriver::update_command(const DigitCommand::ConstSharedPtr command) {
    this->command_callback(command);
    return absl::OkStatus();
}

void LowLevelApiDriver::state_callback(const llapi_observation_t& observation) {
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

    state_publisher_->publish(msg);
}

void LowLevelApiDriver::command_callback(const DigitCommand::ConstSharedPtr msg) {
    std::scoped_lock lock(this->command_mutex_);
    this->latest_command_ = msg;
    this->has_command_ = true;
}

absl::Status LowLevelApiDriver::internal() {
    constexpr auto LLAPI_ERROR = -1;
    constexpr auto LLAPI_NO_NEW_DATA = 0;

    auto result = llapi_get_observation(&this->observation_);
    switch (result) {
        case LLAPI_NO_NEW_DATA:
            return absl::OkStatus();
        case LLAPI_ERROR:
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000, 
                "LowLevelApi Driver: Error receiving observation."
            );
            return absl::InternalError("LowLevelApi Driver: Error receiving observation.");
        default:
            break;
    }

    const DigitCommand::ConstSharedPtr command_snapshot = [this]() -> DigitCommand::ConstSharedPtr {
        std::scoped_lock lock(this->command_mutex_);
        if (!this->has_command_) return nullptr;
        return this->latest_command_;
    }();
    if (!command_snapshot) {
        this->state_callback(this->observation_);
        return absl::OkStatus();
    }

    for (size_t i = 0; const auto& cmd : command_snapshot->motors) {
        if (i >= NUM_MOTORS) 
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