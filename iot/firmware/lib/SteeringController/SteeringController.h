#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

/**
 * @struct SteeringConfig
 * @brief Parâmetros de operação do servo de direção.
 */
struct SteeringConfig
{
        /** @brief Frequência de operação (Hz). */
        uint16_t periodHertz;
        /** @brief Pulso mínimo absoluto em microssegundos (us). */
        uint16_t minPulseUs;
        /** @brief Pulso máximo absoluto em microssegundos (us). */
        uint16_t maxPulseUs;
        /** @brief Pulso de repouso/centro do servo (us). */
        uint16_t centerPulseUs;
        /** @brief Desvio máximo permitido do centro em microssegundos (us). */
        uint16_t maxDeflectionUs;
        /** @brief Inverter o sentido lógico da direção. */
        bool invertDirection;
        /** @brief Taxa máxima de giro (Slew Rate) em us/s. 0 = instantâneo. */
        uint16_t maxSpeedUsPerSec;
};

/**
 * @class SteeringController
 * @brief Controlador de direção de alta resolução utilizando Servo Motor.
 */
class SteeringController
{
    public:
        /**
         * @brief Instancia o controlador de direção.
         * @param pin Pino de sinal PWM do servo.
         * @param config Estrutura de configuração de temporização e limites.
         */
        SteeringController(uint8_t pin, const SteeringConfig& config);

        /**
         * @brief Configura e anexa o periférico do servo.
         */
        void begin();

        /**
         * @brief Define a posição de esterçamento desejada (Setpoint).
         * @param value Comando normalizado de -1.0 (Esquerda) a 1.0 (Direita).
         */
        void setSteering(float value);

        /**
         * @brief Move o servo para a posição central configurada
         * instantaneamente.
         */
        void center();

        /**
         * @brief Atualiza a posição atual do servo em direção ao Setpoint,
         * respeitando o Slew Rate. Deve ser chamado periodicamente no loop
         * principal ou task.
         */
        void update();

    private:
        const uint8_t _pin;
        const SteeringConfig _config;
        Servo _servo;

        float _targetPulseUs;
        float _currentPulseUs;
        uint32_t _lastUpdateTime;
};
