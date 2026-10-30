#include "robomas_plugins/target_epoch_filter.hpp"

#include <cstdlib>
#include <iostream>
#include <thread>

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
    robomas_bridge::TargetEpochFilter filter;
    require(filter.accepts(3, 0), "legacy unstamped targets must work before a stamped reset");
    const auto reset = robomas_bridge::stampNanoseconds(12, 100);
    require(reset == 12000000100LL, "stamp conversion must preserve seconds and nanoseconds");
    filter.reset(3, reset);
    require(!filter.accepts(3, reset - 1), "older targets must not cross a position reset");
    require(!filter.accepts(3, 0), "unstamped targets must not cross a stamped position reset");
    require(filter.accepts(3, reset), "equal-stamped target must be accepted");
    require(filter.accepts(3, reset + 1), "newer target must be accepted");
    require(filter.accepts(4, 0), "one motor reset must not block another motor");
    filter.reset(3, 0);
    require(filter.accepts(3, 0), "an unstamped frame must retain legacy target support");
    require(robomas_bridge::stampNanoseconds(-1, 900000000) == -100000000LL,
            "negative ROS stamps must use signed arithmetic");
    // Concurrent callbacks are possible in a multi-threaded ROS 2 executor.
    std::thread writer([&filter]()
    {
        for (int i = 1; i <= 10000; ++i)
            filter.reset(0, i);
    });
    std::thread reader([&filter]()
    {
        for (int i = 1; i <= 10000; ++i)
            (void)filter.accepts(0, i);
    });
    writer.join();
    reader.join();
    require(filter.accepts(0, 10000), "final concurrent timestamp missing");
    require(!filter.accepts(0, 9999), "stale target accepted after concurrent updates");
    std::cout << "target_epoch_filter: PASS\n";
}
