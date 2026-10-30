#pragma once

#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace robomas_bridge
{
class SerialWriteQueue
{
public:
    using Completion = std::function<void(bool)>;
    using StartWrite = std::function<void(const std::vector<uint8_t> &, Completion)>;

    explicit SerialWriteQueue(StartWrite start)
        : state_(std::make_shared<State>(std::move(start))) {}

    void write(std::vector<uint8_t> data)
    {
        bool idle;
        {
            std::lock_guard<std::mutex> lock(state_->mutex);
            idle = state_->pending.empty();
            state_->pending.push_back(std::make_shared<const std::vector<uint8_t>>(std::move(data)));
        }
        if (idle)
            state_->startNext();
    }

private:
    struct State : std::enable_shared_from_this<State>
    {
        explicit State(StartWrite writer) : start(std::move(writer)) {}

        void startNext()
        {
            std::shared_ptr<const std::vector<uint8_t>> data;
            {
                std::lock_guard<std::mutex> lock(mutex);
                data = pending.front();
            }
            // Both the buffer and queue state outlive the asynchronous operation.
            start(*data, [self = shared_from_this(), data](bool success)
            {
                bool more;
                {
                    std::lock_guard<std::mutex> lock(self->mutex);
                    self->pending.pop_front();
                    if (!success)
                        self->pending.clear();
                    more = !self->pending.empty();
                }
                if (more)
                    self->startNext();
            });
        }

        StartWrite start;
        std::mutex mutex;
        std::deque<std::shared_ptr<const std::vector<uint8_t>>> pending;
    };

    std::shared_ptr<State> state_;
};
}
