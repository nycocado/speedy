#pragma once

#include <Arduino.h>

/**
 * @namespace Pins
 * @brief Mapeamento de hardware para o ESP32-S3.
 */
namespace Pins
{
    /** @brief Pino de Habilitação da Ponte H (R_EN + L_EN). */
    constexpr uint8_t MOTOR_EN = 1;
    /** @brief Pino PWM para sentido reverso (LPWM). */
    constexpr uint8_t MOTOR_LPWM = 2;
    /** @brief Pino PWM para sentido frente (RPWM). */
    constexpr uint8_t MOTOR_RPWM = 3;
    /** @brief Pino de sinal do Servo de Direção. */
    constexpr uint8_t SERVO = 4;
} // namespace Pins
 
/**
 * @namespace Config
 * @brief Parâmetros lógicos globais de sistema.
 */
namespace Config
{
    /**
     * @namespace Motor
     * @brief Configurações de controle e sinal do motor de tração.
     */
    namespace Motor
    {
        /** @brief Frequência do PWM (Hz). */
        constexpr uint32_t FREQUENCY = 20000;
        /** @brief Resolução do PWM (bits). */
        constexpr uint8_t RESOLUTION = 10;
        /** @brief Tempo morto entre inversões de rotação (ms). */
        constexpr uint16_t DEADBAND_MS = 10;
        /** @brief Esforço mínimo normalizado (0.0 a 1.0) para acionar o motor (Deadzone). */
        constexpr float MIN_EFFORT = 0.05f;
        /** @brief Esforço mínimo para manter o motor em movimento (Sustentação). */
        constexpr float MIN_DRIVE_EFFORT = 0.15f;
        /** @brief Inverter o sentido de rotação do motor (true/false). */
        constexpr bool INVERT_DIRECTION = false;
        /** @brief Tempo do pulso inicial para vencer a inércia (ms). */
        constexpr uint16_t KICKSTART_MS = 80;
        /** @brief Intensidade do pulso inicial (0.0 a 1.0). */
        constexpr float KICKSTART_EFFORT = 0.20f;
    } // namespace Motor

    /**
     * @namespace Steering
     * @brief Configurações do servo de direção (MG996R).
     */
    namespace Steering
    {
        /** @brief Frequência de operação (Hz). */
        constexpr uint16_t FREQUENCY = 50;
        /** @brief Pulso mínimo absoluto em microssegundos (Extrema
         * Esquerda/Direita). */
        constexpr uint16_t MIN_PULSE_US = 500;
        /** @brief Pulso máximo absoluto em microssegundos (Extrema
         * Direita/Esquerda). */
        constexpr uint16_t MAX_PULSE_US = 2500;
        /** @brief Pulso de centro (Trimming) em microssegundos. Normalmente
         * 1500us. */
        constexpr uint16_t CENTER_PULSE_US = 1500;
        /** @brief Desvio máximo permitido do centro em microssegundos (Ex:
         * 500us = amplitude de 1000us a 2000us). */
        constexpr uint16_t MAX_DEFLECTION_US = 500;
        /** @brief Inverter o sentido de giro da direção (true/false). */
        constexpr bool INVERT_DIRECTION = false;
        /** @brief Taxa máxima de giro (Slew Rate) em microssegundos por
         * segundo. 0 para desativar. */
        constexpr uint16_t MAX_SPEED_US_PER_SEC = 5000;
    } // namespace Steering

    /**
     * @namespace Gamepad
     * @brief Mapeamento de índices do controle ROS (joy).
     */
    namespace Gamepad
    {
        /** @brief Zona morta (Deadzone) para ignorar variações ruidosas dos
         * eixos analógicos do Joystick. */
        constexpr float DEADZONE = 0.05f;

        // Eixos
        constexpr uint8_t AXIS_STEERING = 0;
        constexpr uint8_t AXIS_THROTTLE = 5;
        constexpr uint8_t AXIS_REVERSE = 4;

        // Botões
        constexpr uint8_t BTN_BRAKE = 0;
        constexpr uint8_t BTN_MANUAL = 1;
        constexpr uint8_t BTN_AUTO = 2;
    } // namespace Gamepad
} // namespace Config
