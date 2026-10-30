#pragma once

#include "cobs.hpp"
#include <cstdint>
#include <cstring>
#include <vector>

namespace robomas_bridge
{
// Firmware mode 2 (position) and mode 4 (velocity-limited position)
// both reset the zero position on receipt of their configuration frame.
inline bool resetsPositionOrigin(uint8_t mode)
{
    return mode == 2 || mode == 4;
}

// Format defined by robomas_center/Core/Src/MotorCtrl.cpp, setFrame case 4:
// [0] 0x30 | motor_index (0..7)
// [1] 0x04 | (is_C620 ? 0x80 : 0)
// [2] thermal limit
// [3..6] velocity Kp (float32)
// [7..10] velocity Ki (float32)
// [11..14] position Kp (float32)
// [15..18] maximum velocity magnitude (float32, rad/s)
//
// The target position in degrees is a separate 5-byte RobomasTarget frame.
inline std::vector<uint8_t> makeStablePositionConfig(
    uint8_t motor, bool c620, uint8_t temp,
    float vel_kp, float vel_ki, float pos_kp, float max_vel)
{
    static_assert(sizeof(float) == 4, "STM32 protocol requires 32-bit floats");
    std::vector<uint8_t> frame(19, 0);
    frame[0] = static_cast<uint8_t>(0x30 | (motor & 0x07));
    frame[1] = static_cast<uint8_t>(0x04 | (c620 ? 0x80 : 0x00));
    frame[2] = temp;
    std::memcpy(frame.data() + 3,  &vel_kp, sizeof(float));
    std::memcpy(frame.data() + 7,  &vel_ki, sizeof(float));
    std::memcpy(frame.data() + 11, &pos_kp, sizeof(float));
    std::memcpy(frame.data() + 15, &max_vel, sizeof(float));
    return frame;
}

inline std::vector<uint8_t> encodeRobomasFrameWithFeedback(std::vector<uint8_t> raw_data)
{
    const uint8_t mode = raw_data[1] & 0x7f;
    if (mode == 0)
    {
        // Firmware mode 0 controls the global CAN-feedback forwarding flag.
        raw_data[3] = 1;
    }
    std::vector<uint8_t> output;
    if (resetsPositionOrigin(mode))
    {
        // Enable forwarding before a position-related mode resets this motor's origin.
        std::vector<uint8_t> diagnostic(19, 0);
        diagnostic[0] = raw_data[0];
        diagnostic[1] = raw_data[1] & 0x80;
        diagnostic[2] = raw_data[2];
        diagnostic[3] = 1;
        output = cobs::encode(diagnostic);
    }
    const auto frame = cobs::encode(raw_data);
    output.insert(output.end(), frame.begin(), frame.end());
    return output;
}
}
