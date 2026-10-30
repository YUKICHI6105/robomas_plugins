#pragma once

#include "cobs.hpp"

namespace robomas_bridge
{
inline std::vector<uint8_t> encodeRobomasFrameWithFeedback(std::vector<uint8_t> raw_data)
{
    const uint8_t mode = raw_data[1] & 0x7f;
    if (mode == 0)
    {
        // Firmware mode 0 controls the global CAN-feedback forwarding flag.
        raw_data[3] = 1;
    }
    std::vector<uint8_t> output;
    if (mode == 2)
    {
        // Enable forwarding before the position frame resets this motor's origin.
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
