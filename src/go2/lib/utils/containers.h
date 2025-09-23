#pragma once

#include <array>
#include <vector>
#include <cstdint>
#include <bit>
#include <cmath>

namespace robot::go2::containers {

    struct MotorCmd {
        uint8_t mode{0};
        float q{0.0f};
        float dq{0.0f};
        float tau{0.0f};
        float kp{0.0f};
        float kd{0.0f};
        std::array<uint32_t, 3> reserve{};
    };

    struct MotorCmds {
        std::vector<MotorCmd> cmds{};
    };

    struct MotorState {
        uint8_t mode{0};
        float q{0.0f};
        float dq{0.0f};
        float ddq{0.0f};
        float tau_est{0.0f};
        float q_raw{0.0f};
        float dq_raw{0.0f};
        float ddq_raw{0.0f};
        int8_t temperature{0};
        uint32_t lost{0};
        std::array<uint32_t, 2> reserve{};
    };

    struct MotorStates {
        std::vector<MotorState> states{};
    };

    struct AudioData {
        uint64_t time_frame;
        uint8_t data;
    };

    struct BmsCmd {
        uint8_t off;
        std::array<uint8_t, 3> reserve;
    };

    struct BmsState {
        uint8_t version_high;
        uint8_t version_low;
        uint8_t status;
        uint8_t soc;
        int32_t current;
        uint16_t cycle;
        std::array<uint8_t, 2> bq_ntc;
        std::array<uint8_t, 2> mcu_ntc;
        std::array<uint16_t, 15> cell_vol;
    };

    struct Error {
        uint32_t source;
        uint32_t state;
    };

    struct Go2FrontVideoData {
        uint64_t time_frame;
        std::vector<uint8_t> video720p;
        std::vector<uint8_t> video360p;
        std::vector<uint8_t> video180p;
    };

    struct HeightMap {
        // ## Header ##
        std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> stamp{};
        std::string frame_id{};
        float resolution{0.0f};
        uint32_t width{0};
        uint32_t height{0};
        std::array<float, 2> origin{};
        std::vector<float> data{};
    };

    struct IMUState {
        std::array<float, 4> quaternion{};
        std::array<float, 3> gyroscope{};
        std::array<float, 3> accelerometer{};
        std::array<float, 3> rpy{};
        int8_t temperature{};
    };

    struct InterfaceConfig {
        uint8_t mode{0};
        uint8_t value{0};
        std::array<uint8_t, 2> reserve{};
    };

    struct LidarState {
        std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> stamp{};
        std::string firmware_version{};
        std::string software_version{};
        std::string sdk_version{};
        float sys_rotation_speed{0.0f};
        float com_rotation_speed{0.0f};
        uint8_t error_state{0};
        float cloud_frequency{0.0f};
        float cloud_packet_loss_rate{0.0f};
        uint32_t cloud_size{0};
        uint32_t cloud_scan_num{0};
        float imu_frequency{0.0f};
        float imu_packet_loss_rate{0.0f};
        std::array<float, 3> imu_rpy{};
        std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> serial_recv_stamp{};
        uint32_t serial_buffer_size{0};
        uint32_t serial_buffer_read{0};
    };

    struct LowCmd {
        std::array<uint8_t, 2> head{};
        uint8_t level_flag{0};
        uint8_t frame_reserve{0};
        std::array<uint32_t, 2> sn{};
        std::array<uint32_t, 2> version{};
        uint16_t bandwidth{0};
        std::array<MotorCmd, 20> motor_cmd{};
        BmsCmd bms_cmd{};
        std::array<uint8_t, 40> wireless_remote{};
        std::array<uint8_t, 12> led{};
        std::array<uint8_t, 2> fan{};
        uint8_t gpio{0};
        uint32_t reserve{0};
        uint32_t crc{0};
    };

    struct LowState {
        std::array<uint8_t, 2> head{};
        uint8_t level_flag{0};
        uint8_t frame_reserve{0};
        std::array<uint32_t, 2> sn{};
        std::array<uint32_t, 2> version{};
        uint16_t bandwidth{0};
        IMUState imu_state{};
        std::array<MotorState, 20> motor_state{};
        BmsState bms_state{};
        std::array<int16_t, 4> foot_force{};
        std::array<int16_t, 4> foot_force_est{};
        uint32_t tick{0};
        std::array<uint8_t, 40> wireless_remote{};
        uint8_t bit_flag{0};
        float adc_reel{0.0f};
        int8_t temperature_ntc1{0};
        int8_t temperature_ntc2{0};
        float power_v{0.0f};
        float power_a{0.0f};
        std::array<uint16_t, 4> fan_frequency{};
        uint32_t reserve{0};
        uint32_t crc{0};
    };

    struct PathPoint {
        std::chrono::duration<float> t_from_start{};
        float x{0.0f};
        float y{0.0f};
        float yaw{0.0f};
        float vx{0.0f};
        float vy{0.0f};
        float vyaw{0.0f};
    };

    struct Req {
        std::string uuid{};
        std::string body{};
    };

    struct Res {
        std::string uuid{};
        std::vector<uint8_t> data{};
        std::string body{};
    };

    struct SportModeCmd {
        uint8_t mode{0};
        uint8_t gait_type{0};
        uint8_t speed_level{0};
        float foot_raise_height{0.0f};
        float body_height{0.0f};
        std::array<float, 2> position{};
        std::array<float, 3> euler{};
        std::array<float, 2> velocity{};
        float yaw_speed{0.0f};
        BmsCmd bms_cmd{};
        std::array<PathPoint, 30> path_point{};
    };

    struct TimeSpec {
        int32_t sec{0};
        uint32_t nanosec{0};
    };

    struct SportModeState {
        TimeSpec stamp{};
        uint32_t error_code{0};
        IMUState imu_state{};
        uint8_t mode{0};
        float progress{0.0f};
        uint8_t gait_type{0};
        float foot_raise_height{0.0f};
        float body_height{0.0f};
        std::array<float, 3> position{};
        std::array<float, 3> velocity{};
        float yaw_speed{0.0f};
        std::array<float, 4> range_obstacle{};
        std::array<int16_t, 4> foot_force{};
        std::array<float, 12> foot_position_body{};
        std::array<float, 12> foot_speed_body{};
    };

    struct UwbState {
        std::array<uint8_t, 2> version{};
        uint8_t channel{0};
        uint8_t joy_mode{0};
        float orientation_est{0.0f};
        float pitch_est{0.0f};
        float distance_est{0.0f};
        float yaw_est{0.0f};
        float tag_roll{0.0f};
        float tag_pitch{0.0f};
        float tag_yaw{0.0f};
        float base_roll{0.0f};
        float base_pitch{0.0f};
        float base_yaw{0.0f};
        std::array<float, 2> joystick{};
        uint8_t error_state{0};
        uint8_t buttons{0};
        uint8_t enabled_from_app{0};
    };

    struct UwbSwitch {
        uint8_t enabled{0};
    };

    struct WirelessController {
        float lx{0.0f};
        float ly{0.0f};
        float rx{0.0f};
        float ry{0.0f};
        uint16_t keys{0};
    };

} // namespace robot::go2::containers