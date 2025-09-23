#pragma once

#include <array>
#include <concepts>
#include <cstddef> // offsetof
#include <cstdint>
#include <ranges>  // std::span and std::ranges::transform
#include <type_traits> // std::is_standard_layout_v

#include "src/h1_2/msgs/unitree_hg_msgs.h"
#include "src/h1_2/lib/utils/containers.h"


namespace robot::h1_2::crc {

    static_assert(std::is_standard_layout_v<robot::h1_2::containers::MotorCmd>, "MotorCmd must be a standard-layout type.");
    static_assert(std::is_standard_layout_v<robot::h1_2::containers::LowCmd>, "LowCmd must be a standard-layout type.");
    static_assert(
        offsetof(robot::h1_2::containers::LowCmd, crc) % sizeof(uint32_t) == 0,
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
     * @brief Converts the H1-2 LowCmd message to a packed struct for the CRC calculation.
     */
    robot::h1_2::containers::LowCmd to_packed_format(const unitree_hg::msg::LowCmd& msg) {
        robot::h1_2::containers::LowCmd packed{};
        packed.mode_pr = msg.mode_pr;
        packed.mode_machine = msg.mode_machine;

        for (int i = 0; i < 35; ++i) {
            packed.motor_cmd[i].mode = msg.motor_cmd[i].mode;
            packed.motor_cmd[i].q = msg.motor_cmd[i].q;
            packed.motor_cmd[i].dq = msg.motor_cmd[i].dq;
            packed.motor_cmd[i].tau = msg.motor_cmd[i].tau;
            packed.motor_cmd[i].kp = msg.motor_cmd[i].kp;
            packed.motor_cmd[i].kd = msg.motor_cmd[i].kd;
            packed.motor_cmd[i].reserve = msg.motor_cmd[i].reserve;
        }
        
        // Note: Unitree source only copies the first 4 bytes of reserve
        packed.reserve = msg.reserve;
        return packed;
    };

    /**
     * @brief Calculates the CRC for the H1-2 LowCmd message and sets the 'crc' field.
     */
    void set_crc(unitree_hg::msg::LowCmd& msg) {
        robot::h1_2::containers::LowCmd raw_cmd = to_packed_format(msg);

        constexpr auto size_to_check_bytes = offsetof(robot::h1_2::containers::LowCmd, crc);
        constexpr auto size_to_check_u32 = size_to_check_bytes / sizeof(uint32_t);

        std::span<const uint32_t> data_span(
            reinterpret_cast<const uint32_t*>(&raw_cmd),
            size_to_check_u32
        );

        msg.crc = crc32_core(data_span);
    };

} // namespace robot::h1_2::crc