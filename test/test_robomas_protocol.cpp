#include "robomas_plugins/robomas_protocol.hpp"

#include <cstdlib>
#include <iostream>

namespace
{
void require(bool condition, const char *message)
{
    if (!condition)
    {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

std::vector<std::vector<uint8_t>> decodeFrames(const std::vector<uint8_t> &stream)
{
    std::vector<std::vector<uint8_t>> frames;
    std::vector<uint8_t> encoded;
    for (uint8_t byte : stream)
    {
        encoded.push_back(byte);
        if (byte == 0)
        {
            frames.push_back(cobs::decode(encoded));
            encoded.clear();
        }
    }
    require(encoded.empty(), "serial stream must end on a COBS delimiter");
    return frames;
}
}

int main()
{
    // Removing the diagnostic prefix loses firmware feedback after power-on.
    for (uint8_t motor = 0; motor < 8; ++motor)
    {
        for (uint8_t c620 : {uint8_t{0}, uint8_t{0x80}})
        {
            std::vector<uint8_t> position = {
                uint8_t(0x30 | motor), uint8_t(2 | c620), 75,
                0, 0, 0x80, 0x3f, 0, 0, 0, 0x40,
                0, 0, 0x40, 0x40, 0, 0, 0, 0};
            const auto frames = decodeFrames(
                robomas_bridge::encodeRobomasFrameWithFeedback(position));
            require(frames.size() == 2, "position mode must send diagnostic-on before the position frame");
            std::vector<uint8_t> diagnostic(19, 0);
            diagnostic[0] = uint8_t(0x30 | motor);
            diagnostic[1] = c620;
            diagnostic[2] = 75;
            diagnostic[3] = 1;
            require(frames[0] == diagnostic, "diagnostic frame must address the same motor and preserve controller type/temperature");
            require(frames[1] == position, "position PID bytes must be preserved exactly");
        }
    }

    // A later mode-0 frame must keep global firmware feedback enabled.
    std::vector<uint8_t> disabled(19, 0);
    disabled[0] = 0x37;
    disabled[1] = 0x80;
    disabled[2] = 60;
    auto frames = decodeFrames(robomas_bridge::encodeRobomasFrameWithFeedback(disabled));
    require(frames.size() == 1 && frames[0][3] == 1,
            "mode 0 must preserve diagnostic feedback");
    disabled[3] = 1;
    require(frames[0] == disabled, "mode 0 must preserve all other packet bytes");

    // The other existing wire modes retain their original payload and frame count.
    for (uint8_t mode : {uint8_t{1}, uint8_t{3}, uint8_t{4}})
    {
        std::vector<uint8_t> raw(mode == 3 ? 23 : 19, 0x42);
        raw[0] = 0x34;
        raw[1] = mode;
        frames = decodeFrames(robomas_bridge::encodeRobomasFrameWithFeedback(raw));
        require(frames.size() == 1 && frames[0] == raw,
                "non-position modes must retain their original packet");
    }
    std::cout << "robomas_protocol: PASS\n";
}
