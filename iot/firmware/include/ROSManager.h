#pragma once

#include <DriveControl.h>
#include <Gamepad.h>
#include <MotorController.h>
#include <ROSComms.h>
#include <ROSCore.h>
#include <SystemState.h>

/**
 * @class ROSManager
 * @brief Gerencia a máquina de estados e a conexão com o Agente micro-ROS.
 */
class ROSManager
{
    public:
        /**
         * @brief Executa um ciclo da máquina de estados de conexão do ROS.
         * @param rosCore Referência para o núcleo do ROS.
         * @param motor Referência para o controlador do motor (para freio
         * seguro).
         * @param driveControl Referência para o controle de direção.
         * @param gamepad Referência para o gamepad.
         */
        static void manageConnection(
            ROSCore& rosCore,
            MotorController& motor,
            DriveControl& driveControl,
            Gamepad& gamepad
        );

    private:
        static bool create_entities(
            ROSCore& rosCore,
            DriveControl& driveControl,
            Gamepad& gamepad
        );
        static void destroy_entities(ROSCore& rosCore);

        static RosState currentState;
};