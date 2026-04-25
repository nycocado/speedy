#pragma once

#include <sensor_msgs/msg/joy.h>

/**
 * @struct GamepadConfig
 * @brief Mapeamento de índices para diferentes modelos de controle.
 */
struct GamepadConfig
{
        /** @brief Índice do eixo de direção (Geralmente Left Stick X). */
        uint8_t axisSteering;
        /** @brief Índice do eixo de aceleração (Geralmente RT / R2). */
        uint8_t axisThrottle;
        /** @brief Índice do eixo de ré (Geralmente LT / L2). */
        uint8_t axisReverse;

        /** @brief Índice do botão de freio (Geralmente A/Square). */
        uint8_t btnBrake;
        /** @brief Índice do botão para modo manual (Geralmente B/Cross). */
        uint8_t btnManual;
        /** @brief Índice do botão para modo autônomo (Geralmente X/Circle). */
        uint8_t btnAuto;

        /** @brief Zona morta para os eixos analógicos. */
        float deadzone;
};

/**
 * @class Gamepad
 * @brief Abstrai o parser da mensagem Joy do ROS, fornecendo dados lógicos
 * (Aceleração, Direção, Freio).
 */
class Gamepad
{
    public:
        /**
         * @brief Instancia o parser com uma configuração específica.
         * @param config Estrutura de configuração de eixos e botões.
         */
        Gamepad(const GamepadConfig& config);

        /**
         * @brief Atualiza o estado interno baseado na mensagem Joy.
         * @param msg Ponteiro para a mensagem ROS.
         */
        void update(const sensor_msgs__msg__Joy* msg);

        /** @brief Retorna o valor da direção de -1.0 (Direita) a 1.0
         * (Esquerda). */
        float getSteering() const { return _steering; }

        /** @brief Retorna a aceleração de -1.0 (Ré total) a 1.0 (Frente total).
         */
        float getThrottle() const { return _throttle; }

        /** @brief Retorna o estado do freio. */
        bool isBraking() const { return _braking; }

        /** @brief Retorna true se o botão de modo Autônomo foi pressionado
         * nesta atualização. */
        bool requestAutonomous() const { return _reqAutonomous; }

        /** @brief Retorna true se o botão de modo Manual foi pressionado nesta
         * atualização. */
        bool requestManual() const { return _reqManual; }

    private:
        /**
         * @brief Converte o eixo bruto para 0.0 a 1.0 considerando o
         * comportamento do ROS Joy.
         */
        float _normalizeTrigger(float rawValue, bool& touched);

        void _resetState();
        void _parseButtons(const sensor_msgs__msg__Joy* msg);
        void _parseSteering(const sensor_msgs__msg__Joy* msg);
        void _parseThrottle(const sensor_msgs__msg__Joy* msg);

        const GamepadConfig _config;

        bool _rtTouched;
        bool _ltTouched;

        float _steering;
        float _throttle;
        bool _braking;
        bool _reqAutonomous;
        bool _reqManual;
};
