#pragma once

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include <gpiod.hpp>

#include "rclcpp/rclcpp.hpp"

namespace speedy_control
{
    class HallEncoder
    {
        public:
            bool start(int hall_pin, int hall_en_pin = -1)
            {
                std::string chip_name = "/dev/gpiochip4";
                if (!std::filesystem::exists(chip_name))
                    chip_name = "/dev/gpiochip0";

                try
                {
                    auto chip = gpiod::chip(chip_name);

                    if (hall_en_pin >= 0)
                    {
                        hall_en_request_ =
                            std::make_unique<gpiod::line_request>(
                                chip.prepare_request()
                                    .set_consumer("speedy_hall_en")
                                    .add_line_settings(
                                        hall_en_pin,
                                        gpiod::line_settings()
                                            .set_direction(
                                                gpiod::line::direction::OUTPUT
                                            )
                                            .set_output_value(
                                                gpiod::line::value::ACTIVE
                                            )
                                    )
                                    .do_request()
                            );
                    }

                    hall_request_ = std::make_unique<gpiod::line_request>(
                        chip.prepare_request()
                            .set_consumer("speedy_hall")
                            .add_line_settings(
                                hall_pin,
                                gpiod::line_settings()
                                    .set_direction(
                                        gpiod::line::direction::INPUT
                                    )
                                    .set_edge_detection(
                                        gpiod::line::edge::RISING
                                    )
                                    .set_bias(gpiod::line::bias::PULL_UP)
                            )
                            .do_request()
                    );
                }
                catch (const std::exception&)
                {
                    return false;
                }

                stop_ = false;
                thread_ = std::thread(&HallEncoder::loop, this);
                return true;
            }

            int64_t pulses() const
            {
                return count_.load(std::memory_order_relaxed);
            }

            void stop()
            {
                stop_ = true;
                if (thread_.joinable())
                    thread_.join();
                release(hall_request_);
                release(hall_en_request_);
            }

        private:
            void loop()
            {
                while (!stop_ && rclcpp::ok())
                {
                    if (hall_request_ && hall_request_->wait_edge_events(
                                             std::chrono::milliseconds(100)
                                         ))
                    {
                        gpiod::edge_event_buffer events;
                        hall_request_->read_edge_events(events, 1);
                        count_.fetch_add(1, std::memory_order_relaxed);
                    }
                }
            }

            static void release(std::unique_ptr<gpiod::line_request>& req)
            {
                if (req)
                {
                    try
                    {
                        req->release();
                    }
                    catch (...)
                    {
                    }
                    req.reset();
                }
            }

            std::unique_ptr<gpiod::line_request> hall_request_;
            std::unique_ptr<gpiod::line_request> hall_en_request_;
            std::thread thread_;
            std::atomic<bool> stop_{false};
            std::atomic<int64_t> count_{0};
    };

} // namespace speedy_control
