#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include <pigpiod_if2.h>

namespace speedy_control
{
/**
 * @brief Servo de direção via pigpio (timing DMA). Recebe comando em radianos,
 *        converte para graus, aplica trim/limites/rate-limit e mapeia para largura
 *        de pulso (linear ou regressão polinomial). Subsistema independente do motor.
 */
class ServoDriver
{
public:
    struct Config
    {
        double center_pulse_ms = 1.5;
        double max_deflection_pulse_ms = 1.0;
        // Limites físicos do chassi (assimétricos).
        double max_angle_left_deg = 45.0;
        double max_angle_right_deg = 45.0;
        double trim_deg = 0.0;
        double max_speed_deg_per_sec = 0.0;
        bool invert_direction = false;
        // Calibração via regressão polinomial: pulse_ms = a0 + a1*deg + a2*deg²
        bool use_polynomial = false;
        std::array<double, 3> poly_coefs = {0.0, 0.0, 0.0};
    };

    bool init(int pin)
    {
        pin_ = pin;
        pi_ = pigpio_start(nullptr, nullptr);
        if (pi_ < 0)
            return false;
        set_mode(pi_, pin_, PI_OUTPUT);
        set_servo_pulsewidth(pi_, pin_, 1500);
        prev_pulse_us_ = 1500;
        current_deg_ = 0.0;
        return true;
    }

    bool ok() const { return pi_ >= 0; }
    double current_pulse_ms() const { return prev_pulse_us_ / 1000.0; }

    /// Aplica o comando (rad). dt_s é o período do ciclo para o rate-limit angular.
    void apply(double cmd_rad, double dt_s, const Config& cfg)
    {
        if (pi_ < 0)
            return;

        double deg = cmd_rad * (180.0 / M_PI);
        if (cfg.invert_direction)
            deg = -deg;

        // Clamp assimétrico: esquerda e direita podem ter limites físicos diferentes.
        deg = std::clamp(deg, -cfg.max_angle_right_deg, cfg.max_angle_left_deg);
        double target_deg = deg + cfg.trim_deg;

        if (cfg.max_speed_deg_per_sec <= 0.0)
        {
            current_deg_ = target_deg;
        }
        else
        {
            double max_delta = cfg.max_speed_deg_per_sec * dt_s;
            double err = target_deg - current_deg_;
            current_deg_ += std::clamp(err, -max_delta, max_delta);
        }

        double pulse_ms;
        if (cfg.use_polynomial)
        {
            const double x = current_deg_;
            pulse_ms = cfg.poly_coefs[0] + cfg.poly_coefs[1] * x +
                       cfg.poly_coefs[2] * x * x;
        }
        else
        {
            // O mapeamento garante que pedir max_angle_* resulte exatamente na deflexão máxima configurada.
            // Os ângulos são os limites absolutos de software que atingem os limites físicos/elétricos do servo.
            if (current_deg_ >= 0.0)
            {
                double ref = std::max(cfg.max_angle_left_deg, 0.1); // Evita divisão por zero
                pulse_ms = cfg.center_pulse_ms + (current_deg_ / ref) * cfg.max_deflection_pulse_ms;
            }
            else
            {
                double ref = std::max(cfg.max_angle_right_deg, 0.1);
                // current_deg_ é negativo, então ele vai SUBTRAIR do centro naturalmente.
                pulse_ms = cfg.center_pulse_ms + (current_deg_ / ref) * cfg.max_deflection_pulse_ms;
            }
        }

        // Trava de segurança: garante que mesmo a regressão polinomial não quebre o servo.
        double min_safe = cfg.center_pulse_ms - cfg.max_deflection_pulse_ms;
        double max_safe = cfg.center_pulse_ms + cfg.max_deflection_pulse_ms;
        pulse_ms = std::clamp(pulse_ms, min_safe, max_safe);

        unsigned int pulse_us = static_cast<unsigned int>(pulse_ms * 1000.0);
        if (pulse_us != prev_pulse_us_)
        {
            set_servo_pulsewidth(pi_, pin_, pulse_us);
            prev_pulse_us_ = pulse_us;
        }
    }

    void stop()
    {
        if (pi_ >= 0)
        {
            set_servo_pulsewidth(pi_, pin_, 0);
            pigpio_stop(pi_);
            pi_ = -1;
        }
    }

private:
    int pi_ = -1;
    int pin_ = 0;
    unsigned int prev_pulse_us_ = 0;
    double current_deg_ = 0.0;
};

} // namespace speedy_control
