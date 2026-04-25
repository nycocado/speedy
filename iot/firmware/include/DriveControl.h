#pragma once

#include <Gamepad.h>
#include <MotorController.h>
#include <SteeringController.h>
#include <SystemState.h>

/**
 * @class DriveControl
 * @brief Gerenciador (Mediador) da lógica de negócios. Lê abstrações lógicas
 * (Gamepad/Autônomo) e comanda o Hardware.
 */
class DriveControl
{
    public:
        DriveControl(MotorController& motor, SteeringController& steering);

        /**
         * @brief Atualiza a intenção de movimento baseado no estado lógico do
         * gamepad.
         * @param gamepad Instância do gamepad com dados mais recentes.
         */
        void handleManualInput(const Gamepad& gamepad);

        /**
         * @brief Executa o ciclo de vida contínuo do controle de hardware.
         * Deve ser chamado a cada iteração do loop principal (independente do
         * modo).
         */
        void update();

        /** @brief Obtém o modo de condução atual. */
        DriveMode getMode() const { return _currentMode; }

        /** @brief Altera o modo de condução. */
        void setMode(DriveMode mode) { _currentMode = mode; }

    private:
        MotorController& _motor;
        SteeringController& _steering;
        DriveMode _currentMode;
};
