#include "robomas_plugins/serial_write_queue.hpp"

#include <cstdlib>
#include <iostream>

using Queue = robomas_bridge::SerialWriteQueue;

void require(bool condition, const char *message)
{
    if (!condition)
    {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

int main()
{
    // The asynchronous boundary deliberately borrows the bytes until completion,
    // as Boost.Asio does. Only the transport boundary is replaced in this test.
    const std::vector<uint8_t> *in_flight = nullptr;
    Queue::Completion completion;
    std::vector<std::vector<uint8_t>> received;
    unsigned starts = 0;
    Queue queue([&](const std::vector<uint8_t> &data, Queue::Completion done)
    {
        require(!completion, "a second serial write must not start before completion");
        in_flight = &data;
        completion = std::move(done);
        ++starts;
    });
    auto finish = [&](bool success)
    {
        received.push_back(*in_flight);
        auto done = std::move(completion);
        completion = {};
        done(success);
    };

    {
        std::vector<uint8_t> position_stream = {2, 0x33, 1, 1, 0, 2, 0x33, 2, 0};
        queue.write(position_stream);
        position_stream.assign(10000, 0xff);
    }
    queue.write({0x38, 0, 0, 0x80, 0x3f, 0});
    queue.write({0x39, 0, 0, 0, 0x40, 0});
    require(starts == 1, "queued targets must wait for the complete frame stream");
    finish(true);
    finish(true);
    finish(true);
    require(received == std::vector<std::vector<uint8_t>>{
                {2, 0x33, 1, 1, 0, 2, 0x33, 2, 0},
                {0x38, 0, 0, 0x80, 0x3f, 0},
                {0x39, 0, 0, 0, 0x40, 0}},
            "packet buffers must survive callers and retain FIFO ordering");
    require(!completion && starts == 3, "queue must stop after its final write");

    queue.write({1});
    queue.write({2});
    finish(false);
    require(!completion && starts == 4,
            "a failed stream must discard pending commands rather than replay stale targets");
    queue.write({3});
    finish(true);
    require(received.back() == std::vector<uint8_t>{3}, "queue must allow a new write after failure");

    // Destruction of the initiating wrapper must not invalidate an active buffer.
    {
        Queue temporary([&](const std::vector<uint8_t> &data, Queue::Completion done)
        {
            in_flight = &data;
            completion = std::move(done);
        });
        temporary.write({4, 5, 6});
    }
    require(*in_flight == std::vector<uint8_t>{4, 5, 6},
            "an outstanding completion must retain its packet and queue state");
    finish(true);
    std::cout << "serial_write_queue: PASS\n";
}
