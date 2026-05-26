#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include <fcntl.h>
#include <unistd.h>

#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace speedy_control
{
/**
 * @brief Motor de tração: ponte H (BTS7960) com dois canais PWM via sysfs (fwd/rev).
 *        Recebe um esforço normalizado em [-1,1] e trata a zona morta (dead-time) ao
 *        inverter o sentido, evitando shoot-through.
 */
class MotorDriver
{
public:
    bool init(int fwd_channel, int rev_channel, int64_t period_ns)
    {
        period_ns_ = period_ns;
        return fwd_.init(fwd_channel, period_ns) && rev_.init(rev_channel, period_ns);
    }

    /**
     * @param effort      esforço em [-1,1]
     * @param time        instante atual (para medir o dead-time)
     * @param deadband_ms duração do dead-time da ponte H ao inverter
     * @param min_effort  abaixo disso o motor é desligado
     * @param invert      inverte o sentido (fiação)
     */
    void apply(double effort, const rclcpp::Time& time, int64_t deadband_ms,
               double min_effort, bool invert)
    {
        // Salva a direção lógica (vontade do controlador) para telemetria e odometria.
        if (std::abs(effort) < min_effort)
            last_direction_ = 0;
        else
            last_direction_ = (effort > 0) ? 1 : -1;

        if (invert)
            effort = -effort;

        if (std::abs(effort) < min_effort)
        {
            fwd_.set_duty_ns(0);
            rev_.set_duty_ns(0);
            last_physical_dir_ = 0;
            in_deadband_ = false;
            return;
        }

        int8_t target_dir = (effort > 0) ? 1 : -1;

        // Inversão de sentido (físico): segura zero pela duração do dead-time ANTES de aplicar
        // o novo sentido (a partir do repouso, last_physical_dir_==0, não há dead-time).
        if (target_dir != last_physical_dir_ && last_physical_dir_ != 0)
        {
            if (!in_deadband_)
            {
                fwd_.set_duty_ns(0);
                rev_.set_duty_ns(0);
                in_deadband_ = true;
                deadband_start_ = time;
                return;
            }
            if ((time - deadband_start_).seconds() < (deadband_ms / 1000.0))
            {
                fwd_.set_duty_ns(0);
                rev_.set_duty_ns(0);
                return; // ainda no dead-time
            }
            in_deadband_ = false; // dead-time concluído → aplica o novo sentido abaixo
        }
        else
        {
            in_deadband_ = false;
        }

        last_physical_dir_ = target_dir;
        int64_t duty = static_cast<int64_t>(std::abs(effort) * period_ns_);
        if (target_dir > 0)
        {
            rev_.set_duty_ns(0);
            fwd_.set_duty_ns(duty);
        }
        else
        {
            fwd_.set_duty_ns(0);
            rev_.set_duty_ns(duty);
        }
    }

    int8_t direction() const { return last_direction_; }

    void stop()
    {
        fwd_.stop();
        rev_.stop();
        last_direction_ = 0;
        last_physical_dir_ = 0;
    }

private:
    /// Manipulação segura de PWM via SysFS (RP1 do Raspberry Pi 5).
    struct SafePWM
    {
        bool init(int channel, int64_t period_ns)
        {
            period_ns_ = period_ns;
            chip_path_ = "/sys/class/pwm/pwmchip0";
            if (!std::filesystem::exists(chip_path_))
            {
                for (int i = 1; i < 10; i++)
                {
                    std::string p = "/sys/class/pwm/pwmchip" + std::to_string(i);
                    if (std::filesystem::exists(p))
                    {
                        chip_path_ = p;
                        break;
                    }
                }
            }
            if (!std::filesystem::exists(chip_path_))
                return false;

            std::string pwm_path = chip_path_ + "/pwm" + std::to_string(channel);
            if (!std::filesystem::exists(pwm_path))
            {
                write_sysfs(chip_path_ + "/export", std::to_string(channel));
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }

            write_sysfs(pwm_path + "/period", std::to_string(period_ns));
            write_sysfs(pwm_path + "/enable", "1");

            fd_duty_ = open((pwm_path + "/duty_cycle").c_str(), O_WRONLY);
            return (fd_duty_ >= 0);
        }

        bool set_duty_ns(int64_t duty_ns)
        {
            if (fd_duty_ < 0)
                return false;
            if (duty_ns > period_ns_)
                duty_ns = period_ns_;
            if (duty_ns < 0)
                duty_ns = 0;
            char buf[32];
            int len = snprintf(buf, sizeof(buf), "%lld\n", (long long)duty_ns);
            return pwrite(fd_duty_, buf, len, 0) >= 0;
        }

        void stop()
        {
            set_duty_ns(0);
            if (fd_duty_ >= 0)
            {
                close(fd_duty_);
                fd_duty_ = -1;
            }
        }

    private:
        int fd_duty_ = -1;
        int64_t period_ns_ = 20000000;
        std::string chip_path_;

        void write_sysfs(const std::string& path, const std::string& value)
        {
            std::ofstream fs(path);
            if (fs.is_open())
                fs << value;
        }
    };

    SafePWM fwd_, rev_;
    int64_t period_ns_ = 50000;
    int8_t last_direction_ = 0;      // Direção lógica (intent)
    int8_t last_physical_dir_ = 0;   // Direção elétrica (pós-inversão)
    bool in_deadband_ = false;
    rclcpp::Time deadband_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
};

} // namespace speedy_control
