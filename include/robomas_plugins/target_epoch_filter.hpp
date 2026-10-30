#pragma once

#include <array>
#include <cstdint>

namespace robomas_bridge
{
inline int64_t stampNanoseconds(int32_t sec, uint32_t nanosec)
{
    return int64_t(sec) * 1000000000LL + nanosec;
}

class TargetEpochFilter
{
public:
    void reset(uint8_t motor, int64_t stamp)
    {
        reset_stamps_[motor] = stamp;
    }

    bool accepts(uint8_t motor, int64_t stamp) const
    {
        return reset_stamps_[motor] == 0 || stamp >= reset_stamps_[motor];
    }

private:
    std::array<int64_t, 8> reset_stamps_{};
};
}
