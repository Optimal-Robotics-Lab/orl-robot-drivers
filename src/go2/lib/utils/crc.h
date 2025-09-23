#pragma once

#include <array>
#include <concepts>
#include <cstddef> // offsetof
#include <cstdint>
#include <ranges>  // std::span and std::ranges::transform
#include <type_traits> // std::is_standard_layout_v

#include "src/go2/msgs/unitree_go_msgs.h"
#include "src/go2/lib/utils/containers.h"

namespace robot::go2::crc {

    static_assert(std::is_standard_layout_v<robot::go2::containers::MotorCmd>, "MotorCmd must be a standard-layout type.");
    static_assert(std::is_standard_layout_v<robot::go2::containers::LowCmd>, "LowCmd must be a standard-layout type.");
    static_assert(
        offsetof(robot::go2::containers::LowCmd, crc) % sizeof(uint32_t) == 0,
        "The data size for CRC calculation must be a multiple of 4 bytes."
    );

    static_assert(std::is_standard_layout_v<robot::go2::containers::MotorCmd>, "MotorCmd must be a standard-layout type.");
    static_assert(std::is_standard_layout_v<robot::go2::containers::BmsCmd>, "BmsCmd must be a standard-layout type.");
    static_assert(std::is_standard_layout_v<robot::go2::containers::LowCmd>, "LowCmd must be a standard-layout type.");

    static_assert(
        offsetof(robot::go2::containers::LowCmd, crc) % sizeof(uint32_t) == 0,
        "The data size for CRC calculation must be a multiple of 4 bytes."
    );

    /**
     * @brief Calculates CRC32 on a span of 32-bit words.
     *
     * @param data A non-owning view over the data to checksum.
     * @return The calculated 32-bit CRC.
     */
    constexpr uint32_t crc32_core(std::span<const uint32_t> data) {
        uint32_t crc = 0xFFFFFFFF;
        constexpr uint32_t polynomial = 0x04C11DB7;

        for (const uint32_t word : data) {
            uint32_t xbit = 1 << 31;
            for (int i = 0; i < 32; ++i) {
                bool const crc_top_bit_set = (crc & 0x80000000);
                crc <<= 1;
                if (crc_top_bit_set) {
                    crc ^= polynomial;
                }
                if (word & xbit) {
                    crc ^= polynomial;
                }
                xbit >>= 1;
            }
        }
        return crc;
    };

    /**
     * @brief Converts the Go2 LowCmd message to a packed struct for the CRC calculation.
     */
    robot::go2::containers::LowCmd to_packed_format(const unitree_go::msg::LowCmd& msg) {
        robot::go2::containers::LowCmd packed{};

        // Direct copies for simple types and std::arrays
        packed.head = msg.head;
        packed.level_flag = msg.level_flag;
        packed.frame_reserve = msg.frame_reserve;
        packed.sn = msg.sn;
        packed.version = msg.version;
        packed.bandwidth = msg.bandwidth;
        
        // Copy BMS command
        packed.bms_cmd.off = msg.bms_cmd.off;
        packed.bms_cmd.reserve = msg.bms_cmd.reserve;

        // Direct copies for the remaining std::array types
        packed.wireless_remote = msg.wireless_remote;
        packed.led = msg.led;
        packed.fan = msg.fan;
        packed.gpio = msg.gpio;
        packed.reserve = msg.reserve;

        // Field-by-field copy for the array of motor commands
        for (size_t i = 0; i < 20; ++i) {
            packed.motor_cmd[i].mode = msg.motor_cmd[i].mode;
            packed.motor_cmd[i].q = msg.motor_cmd[i].q;
            packed.motor_cmd[i].dq = msg.motor_cmd[i].dq;
            packed.motor_cmd[i].tau = msg.motor_cmd[i].tau;
            packed.motor_cmd[i].kp = msg.motor_cmd[i].kp;
            packed.motor_cmd[i].kd = msg.motor_cmd[i].kd;
            packed.motor_cmd[i].reserve = msg.motor_cmd[i].reserve;
        }
        
        return packed;
    };

    /**
     * @brief Calculates the CRC for the Go2 LowCmd message and sets the 'crc' field.
     */
    void set_crc(unitree_go::msg::LowCmd& msg) {
        robot::go2::containers::LowCmd packed_cmd = to_packed_format(msg);

        constexpr auto size_to_check_bytes = offsetof(robot::go2::containers::LowCmd, crc);
        constexpr auto size_to_check_u32 = size_to_check_bytes / sizeof(uint32_t);

        std::span<const uint32_t> data_span(
            reinterpret_cast<const uint32_t*>(&packed_cmd),
            size_to_check_u32
        );

        msg.crc = crc32_core(data_span);
    };

} // namespace robot::go2::crc
