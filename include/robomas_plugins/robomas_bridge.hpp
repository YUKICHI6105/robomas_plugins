#pragma once

#include <cstring>
#include <chrono>
#include <atomic>
#include <cstdint>
#include <future>
#include <vector>

#include "robomas_bridge.hpp"

#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include "robomas_protocol.hpp"
#include "serial_write_queue.hpp"
#include "target_epoch_filter.hpp"
#include "test.hpp"
#include "robomas_plugins/msg/frame.hpp"
#include "robomas_plugins/msg/robomas_frame.hpp"
#include "robomas_plugins/msg/robomas_target.hpp"
#include "robomas_plugins/msg/robomas_state.hpp"
#include "robomas_plugins/msg/tape_led.hpp"
using namespace std::chrono_literals;
using namespace std::placeholders;

namespace slcan_command
{
    enum Command : uint8_t
    {
        Normal = 0,
        Negotiation = 1,
        StateNotify = 4,
        LedSet = 5,

    };
} // namespace slcan_command

namespace robomas_bridge
{

    class RobomasBridge : public rclcpp::Node
    {
    private:
        /////////////Slacan Status///////////////////
        // ctrl+c or ros2 shutdown is called.
        std::atomic<bool> is_shutdown_{false};
        // serial port connected but "HelloSlcan" has not been returned yet.
        std::atomic<bool> is_connected_{false};
        // sconection with usbcan is active. the serial port is new usbcan.
        std::atomic<bool> is_active_{false};
        /////////////Slacan Status///////////////////

        std::string port_name_ = "/dev/robomas";

        std::shared_ptr<boost::asio::io_context> io_context_;
        std::shared_ptr<boost::asio::serial_port> serial_port_;
        std::unique_ptr<SerialWriteQueue> write_queue_;
        TargetEpochFilter target_epochs_;
        // it will prohabit the io_context to stop.
        std::unique_ptr<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>> work_guard_;

        std::shared_ptr<boost::asio::streambuf> read_streambuf_;
        std::unique_ptr<boost::asio::steady_timer> reconnect_timer_;
        std::unique_ptr<boost::asio::steady_timer> handshake_timer_;
        uint64_t connection_generation_ = 0;
        std::chrono::steady_clock::time_point handshake_started_;

        // thread fot running io_context
        std::thread io_context_thread_;

        //std::unique_ptr<std::thread> reading_thread_;

        rclcpp::Publisher<robomas_plugins::msg::Frame>::SharedPtr can_rx_pub_;
        rclcpp::Subscription<robomas_plugins::msg::Frame>::SharedPtr can_tx_sub_;
        rclcpp::Subscription<robomas_plugins::msg::RobomasFrame>::SharedPtr robomas_sub_;
        rclcpp::Subscription<robomas_plugins::msg::RobomasTarget>::SharedPtr robomas_target0_;
        rclcpp::Subscription<robomas_plugins::msg::RobomasTarget>::SharedPtr robomas_target1_;
        rclcpp::Subscription<robomas_plugins::msg::RobomasTarget>::SharedPtr robomas_target2_;
        rclcpp::Subscription<robomas_plugins::msg::RobomasTarget>::SharedPtr robomas_target3_;
        rclcpp::Subscription<robomas_plugins::msg::RobomasTarget>::SharedPtr robomas_target4_;
        rclcpp::Subscription<robomas_plugins::msg::RobomasTarget>::SharedPtr robomas_target5_;
        rclcpp::Subscription<robomas_plugins::msg::RobomasTarget>::SharedPtr robomas_target6_;
        rclcpp::Subscription<robomas_plugins::msg::RobomasTarget>::SharedPtr robomas_target7_;
        rclcpp::Publisher<robomas_plugins::msg::RobomasState>::SharedPtr robo_state_pub_;
        rclcpp::Subscription<robomas_plugins::msg::TapeLed>::SharedPtr tape_led_sub_;

        void canTxCallback(const robomas_plugins::msg::Frame::SharedPtr msg);
        void robomasCallback(const robomas_plugins::msg::RobomasFrame::SharedPtr msg);
        void robomasCallback1(const robomas_plugins::msg::RobomasTarget::SharedPtr msg);
        void robomasCallback2(const robomas_plugins::msg::RobomasTarget::SharedPtr msg);
        void robomasCallback3(const robomas_plugins::msg::RobomasTarget::SharedPtr msg);
        void robomasCallback4(const robomas_plugins::msg::RobomasTarget::SharedPtr msg);
        void robomasCallback5(const robomas_plugins::msg::RobomasTarget::SharedPtr msg);
        void robomasCallback6(const robomas_plugins::msg::RobomasTarget::SharedPtr msg);
        void robomasCallback7(const robomas_plugins::msg::RobomasTarget::SharedPtr msg);
        void robomasCallback8(const robomas_plugins::msg::RobomasTarget::SharedPtr msg);
        void tapeLedCallback(const robomas_plugins::msg::TapeLed::SharedPtr msg);

        // send a color to the tape LED. r/g/b are 0-255.
        void sendTapeLedColor(uint8_t r, uint8_t g, uint8_t b);

        // Transport state is owned by the single Asio event-loop thread.
        void initializeSerialPort(const std::string port_name);
        void scheduleReconnect();
        void scheduleHandshake();
        void disconnect(const std::string &reason);
        void initializeWriteQueue();

        // convert message from usbcan, and process it.
        void readingProcess(const std::vector<uint8_t> data);

        void asyncWrite(const robomas_plugins::msg::Frame::SharedPtr msg);
        void asyncWrite(const slcan_command::Command command, const std::vector<uint8_t> data);
        void asyncWrite(const robomas_plugins::msg::RobomasFrame::SharedPtr robomasFrame);
        void asyncWrite(const robomas_plugins::msg::RobomasTarget::SharedPtr robomasTarget, uint8_t motor);

        // Directly use is deprecated.
        void asyncWrite(const std::vector<uint8_t> data);

        void asyncRead();

        // these function will be called when the data is written to the serial port. for error handling.
        void writeHandler(const boost::system::error_code &error, std::size_t bytes_transferred);

    public:
        RobomasBridge(const rclcpp::NodeOptions &options);

        ~RobomasBridge() override;
        void onShutdown();
    };
}
