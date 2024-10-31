#pragma once

// #include <can_plugins2/msg/frame.hpp>
#include <robomas_plugins/msg/robomas_frame.hpp>
#include <robomas_plugins/msg/robomas_target.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robomas
{
    static std::unique_ptr<robomas_plugins::msg::RobomasTarget> get_target(const float target_data)
    {
        auto target = std::make_unique<robomas_plugins::msg::RobomasTarget>();
        target->target = target_data;
        return target;
    }

    static std::unique_ptr<robomas_plugins::msg::RobomasFrame> get_dis_frame(const uint16_t motor_index,const bool c620) 
    {
        auto frame = std::make_unique<robomas_plugins::msg::RobomasFrame>();
        frame->mode = 0;
        frame->motor = motor_index-1;
        frame->c620 = c620;
        frame->temp = 50;
        return frame;
    }

    static std::unique_ptr<robomas_plugins::msg::RobomasFrame> get_vel_frame(const uint16_t motor_index,const bool is_c620) 
    {
        auto frame = std::make_unique<robomas_plugins::msg::RobomasFrame>();
        frame->mode = 1;
        frame->motor = motor_index-1;
        frame->c620 = is_c620;
        frame->temp = 50;
        frame->velkp = 0.15;
        frame->velki = 9;
        return frame;
    }

    static std::unique_ptr<robomas_plugins::msg::RobomasFrame> get_pos_frame(const uint16_t motor_index,const bool c620) 
    {
        auto frame = std::make_unique<robomas_plugins::msg::RobomasFrame>();
        frame->mode = 2;
        frame->motor = motor_index-1;
        frame->c620 = c620;
        frame->temp = 50;
        frame->velkp = 0.15;
        frame->velki = 9;
        frame->poskp = 0.5;
        return frame;
    }

    static std::unique_ptr<robomas_plugins::msg::RobomasFrame> get_berutyoku_frame(const uint16_t index,const bool c620, const float target_vel,const float target_pos) 
    {
        auto frame = std::make_unique<robomas_plugins::msg::RobomasFrame>();
        frame->mode = 3;
        frame->motor = index-1;
        frame->c620 = c620;
        frame->temp = 50;
        frame->velkp = 0.15;
        frame->velki = 9;
        frame->poskp = 0.5;
        frame->tyoku_vel_target = target_vel;
        frame->tyoku_pos_target = target_pos;
        return frame;
    }

    static std::unique_ptr<robomas_plugins::msg::RobomasFrame> get_stablepos_frame(const uint16_t motor_index,const bool c620,const float limitVel) 
    {
        auto frame = std::make_unique<robomas_plugins::msg::RobomasFrame>();
        frame->mode = 4;
        frame->motor = motor_index-1;
        frame->c620 = c620;
        frame->temp = 50;
        frame->velkp = 0.15;
        frame->velki = 9;
        frame->poskp = 0.5;
        frame->stable_pos_limit_vel = limitVel;
        return frame;
    }
}

