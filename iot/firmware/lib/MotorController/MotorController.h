#pragma once

#include <Arduino.h>

/**
 * @struct MotorConfig
 * @brief Parâmetros de operação do motor.
 */
struct MotorConfig
{
        /** @brief Frequência do PWM (Hz). */
        uint32_t frequency;
        /** @brief Resolução do PWM (bits). */
        uint8_t resolution;
        /** @brief Tempo morto entre inversões de rotação (ms). */
        uint16_t deadbandMs;
        /** @brief Esforço mínimo normalizado para acionar o motor. */
        float minEffort;
        /** @brief Inverter o sentido lógico de rotação. */
        bool invertDirection;
};

/**
 * @enum MotorDirection
 * @brief Estados de direção do motor.
 */
enum class MotorDirection : int8_t
{
    REVERSE = -1,
    STOPPED = 0,
    FORWARD = 1
};

/**
 * @class MotorController
 * @brief Controlador de tração para ponte H BTS7960.
 */
class MotorController
{
    public:
        /**
         * @brief Instancia o controlador de motor.
         * @param pinEn Pino Enable.
         * @param pinLpwm Pino PWM Esquerdo (Ré).
         * @param pinRpwm Pino PWM Direito (Frente).
         * @param config Estrutura de configuração.
         */
        MotorController(
            uint8_t pinEn,
            uint8_t pinLpwm,
            uint8_t pinRpwm,
            const MotorConfig& config
        );

        /**
         * @brief Configura os pinos e canais PWM.
         * @return Sucesso da operação.
         */
        bool begin();

        /**
         * @brief Aciona o motor com o esforço normalizado especificado.
         * @param effort Valor normalizado entre -1.0 (Ré Máxima) e 1.0 (Frente
         * Máxima).
         */
        void setEffort(float effort);

        /**
         * @brief Aciona a frenagem eletromecânica.
         */
        void brake();

        /**
         * @brief Interrompe a alimentação (rotação livre).
         */
        void stop();

    private:
        const uint8_t _pinEn;
        const uint8_t _pinLpwm;
        const uint8_t _pinRpwm;
        const MotorConfig _config;

        int16_t _maxSpeed;
        MotorDirection _currentDirection;
        uint32_t _deadbandStartTime;
        bool _inDeadband;

        /**
         * @brief Zera os sinais PWM em ambos os canais.
         */
        void _clearPwm();
};
