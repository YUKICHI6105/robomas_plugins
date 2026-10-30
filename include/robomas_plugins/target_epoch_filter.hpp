#pragma once

#include <array>
#include <cstdint>
#include <mutex>

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
        std::lock_guard<std::mutex> lock(mutex_);
        reset_stamps_[motor] = stamp;
    }

    bool accepts(uint8_t motor, int64_t stamp) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return reset_stamps_[motor] == 0 || stamp >= reset_stamps_[motor];
    }

private:
    mutable std::mutex mutex_;
    std::array<int64_t, 8> reset_stamps_{};
};
}
