#pragma once

#include <array>
#include <vector>
#include <cstdint>
#include <bit>
#include <cmath>
#include <algorithm>

namespace robot::h1_2::containers {

    struct ButtonState {
        uint16_t value;

        enum class Button : uint16_t {
            R1 = 0, 
            L1 = 1, 
            Start = 2,
            Select = 3,
            R2 = 4, 
            L2 = 5,
            F1 = 6,
            F2 = 7,
            A = 8,
            B = 9,
            X = 10,
            Y = 11,
            Up = 12,
            Right = 13,
            Down = 14,
            Left = 15
        };

        [[nodiscard]] constexpr bool is_R1_pressed() const { return (value >> static_cast<uint16_t>(Button::R1)) & 1; }
        [[nodiscard]] constexpr bool is_L1_pressed() const { return (value >> static_cast<uint16_t>(Button::L1)) & 1; }
        [[nodiscard]] constexpr bool is_start_pressed() const { return (value >> static_cast<uint16_t>(Button::Start)) & 1; }
        [[nodiscard]] constexpr bool is_select_pressed() const { return (value >> static_cast<uint16_t>(Button::Select)) & 1; }
        [[nodiscard]] constexpr bool is_R2_pressed() const { return (value >> static_cast<uint16_t>(Button::R2)) & 1; }
        [[nodiscard]] constexpr bool is_L2_pressed() const { return (value >> static_cast<uint16_t>(Button::L2)) & 1; }
        [[nodiscard]] constexpr bool is_F1_pressed() const { return (value >> static_cast<uint16_t>(Button::F1)) & 1; }
        [[nodiscard]] constexpr bool is_F2_pressed() const { return (value >> static_cast<uint16_t>(Button::F2)) & 1; }
        [[nodiscard]] constexpr bool is_A_pressed() const { return (value >> static_cast<uint16_t>(Button::A)) & 1; }
        [[nodiscard]] constexpr bool is_B_pressed() const { return (value >> static_cast<uint16_t>(Button::B)) & 1; }
        [[nodiscard]] constexpr bool is_X_pressed() const { return (value >> static_cast<uint16_t>(Button::X)) & 1; }
        [[nodiscard]] constexpr bool is_Y_pressed() const { return (value >> static_cast<uint16_t>(Button::Y)) & 1; }
        [[nodiscard]] constexpr bool is_up_pressed() const { return (value >> static_cast<uint16_t>(Button::Up)) & 1; }
        [[nodiscard]] constexpr bool is_right_pressed() const { return (value >> static_cast<uint16_t>(Button::Right)) & 1; }
        [[nodiscard]] constexpr bool is_down_pressed() const { return (value >> static_cast<uint16_t>(Button::Down)) & 1; }
        [[nodiscard]] constexpr bool is_left_pressed() const { return (value >> static_cast<uint16_t>(Button::Left)) & 1; }
    
    };

    struct WirelessRemote {
        std::array<uint8_t, 2> header;
        ButtonState buttons;
        float left_stick_x;
        float right_stick_x;
        float right_stick_y;
        float left_trigger_axis;
        float left_stick_y;
        std::array<uint8_t, 16> idle_data;
        
        private:
            static constexpr float deadzone = 0.05f;
            [[nodiscard]] static constexpr float process_axis(float value) {
                return (std::abs(value) < deadzone) ? 0.0f : std::clamp(value, -1.0f, 1.0f);
            }
        
        public:
            [[nodiscard]] constexpr float get_left_stick_x() const { return process_axis(left_stick_x); }
            [[nodiscard]] constexpr float get_left_stick_y() const { return process_axis(left_stick_y); }
            [[nodiscard]] constexpr float get_right_stick_x() const { return process_axis(right_stick_x); }
            [[nodiscard]] constexpr float get_right_stick_y() const { return process_axis(right_stick_y); }
    };

    struct MotorCmd {
        uint8_t mode;
        float q;
        float dq;
        float tau;
        float kp;
        float kd;
        uint32_t reserve = 0;
    };

    struct MotorState {
        uint8_t mode;
        float q;
        float dq;
        float ddq;
        float tau_est;
        std::array<int16_t, 2> temperature;
        float vol;
        std::array<uint32_t, 2> sensor;
        uint32_t motorstate;
        std::array<uint32_t, 4> reserve;
    };

    struct IMUState {
        std::array<float, 4> quaternion;
        std::array<float, 3> gyroscope;
        std::array<float, 3> accelerometer;
        std::array<float, 3> rpy; 
        int16_t temperature;
    };

    struct LowCmd {
        uint8_t mode_pr;
        uint8_t mode_machine;
        std::array<MotorCmd, 35> motor_cmd;
        std::array<uint32_t, 4> reserve;
        uint32_t crc;
    };

    struct LowState {
        std::array<uint32_t, 2> version;
        uint8_t mode_pr;
        uint8_t mode_machine;
        uint32_t tick;
        IMUState imu_state;
        std::array<MotorState, 35> motor_state;
        std::array<uint8_t, 40> wireless_remote;
        std::array<uint32_t, 4> reserve;
        uint32_t crc;
    };

    struct PressSensorState {
        std::array<float, 12> pressure;
        std::array<float, 12> temperature;
    };

    struct HandCmd {
        std::vector<MotorCmd> motor_cmd;
    };

    struct HandState {
        std::vector<MotorState> motor_state;
        IMUState imu_state;
        std::vector<PressSensorState> press_sensor_state;
        float power_v;
        float power_a;
        std::array<uint32_t, 2> reserve;
    };

    struct BmsCmd {
        uint8_t cmd;
        std::array<uint8_t, 40> reserve;
    };

    struct BmsState {
        uint8_t version_high;
        uint8_t version_low;
        uint8_t fn;
        std::array<uint16_t, 40> cell_vol;
        std::array<uint32_t, 3> bmsvoltage;
        int32_t current;
        uint8_t soc;
        uint8_t soh;
        std::array<uint16_t, 12> temperature;
        uint16_t cycle;
        uint16_t manufacturer_date;
        std::array<uint32_t, 5> bmsstate;
        std::array<uint8_t, 3> reserve;
    };

    struct MainBoardState {
        std::array<uint16_t, 6> fan_state;
        std::array<uint16_t, 6> temperature;
        std::array<float, 6> value;
        std::array<uint32_t, 6> state;
    };

} // namespace robot::h1_2::containers
