#pragma once

/**
 * @enum DriveMode
 * @brief Modos de operação do veículo.
 */
enum class DriveMode
{
    MANUAL,    ///< Operação manual via comandos de Gamepad (Joy)
    AUTONOMOUS ///< Operação autônoma via planejamento do ROS (cmd_vel)
};

/**
 * @enum RosState
 * @brief Estados da máquina de estados do ROS 2.
 */
enum class RosState
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};
