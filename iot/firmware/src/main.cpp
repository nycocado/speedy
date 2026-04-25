#include <Arduino.h>
#include <Constants.h>
#include <DriveControl.h>
#include <Gamepad.h>
#include <MotorController.h>
#include <ROSComms.h>
#include <ROSCore.h>
#include <ROSManager.h>
#include <SteeringController.h>
#include <SystemState.h>

#include <micro_ros_platformio.h>

// --- Estruturas de Configuração ---

// Parâmetros do motor de tração baseados nas constantes do projeto
MotorConfig motorCfg = {
    Config::Motor::FREQUENCY,
    Config::Motor::RESOLUTION,
    Config::Motor::DEADBAND_MS,
    Config::Motor::MIN_EFFORT,
    Config::Motor::INVERT_DIRECTION};

// Mapeamento do gamepad baseado nas constantes do projeto
GamepadConfig gamepadCfg = {
    Config::Gamepad::AXIS_STEERING,
    Config::Gamepad::AXIS_THROTTLE,
    Config::Gamepad::AXIS_REVERSE,
    Config::Gamepad::BTN_BRAKE,
    Config::Gamepad::BTN_MANUAL,
    Config::Gamepad::BTN_AUTO,
    Config::Gamepad::DEADZONE};

// Parâmetros do servo de direção baseados nas constantes do projeto
SteeringConfig steeringCfg = {
    Config::Steering::FREQUENCY,
    Config::Steering::MIN_PULSE_US,
    Config::Steering::MAX_PULSE_US,
    Config::Steering::CENTER_PULSE_US,
    Config::Steering::MAX_DEFLECTION_US,
    Config::Steering::INVERT_DIRECTION,
    Config::Steering::MAX_SPEED_US_PER_SEC};

// --- Instâncias de Hardware e Lógica ---

SteeringController steering(Pins::SERVO, steeringCfg);
MotorController
    motor(Pins::MOTOR_EN, Pins::MOTOR_LPWM, Pins::MOTOR_RPWM, motorCfg);

ROSCore rosCore;
Gamepad gamepad(gamepadCfg);
DriveControl driveControl(motor, steering);

TaskHandle_t RosTaskHandle = NULL;

/**
 * @brief Task dedicada ao processamento de mensagens ROS e controle manual.
 */
void rosTask(void* pvParameters)
{
    for (;;)
    {
        // Executa a máquina de estados do ROS
        ROSManager::manageConnection(rosCore, motor, driveControl, gamepad);

        // Atualiza a intenção de movimento manual se o modo estiver ativo
        if (driveControl.getMode() == DriveMode::MANUAL)
        {
            driveControl.handleManualInput(gamepad);
        }

        // Executa a física contínua do chassi (ex: limites mecânicos, slew
        // rate)
        driveControl.update();

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// --- Funções Principais ---

void setup()
{
    // Inicializa transporte micro-ROS via porta Serial configurada no
    // PlatformIO
    Serial.begin(921600);
    set_microros_serial_transports(Serial);

    // Inicialização do Hardware
    motor.begin();
    steering.begin();
    motor.brake();

    // Criação da task de controle no Core 1 com prioridade 5
    xTaskCreatePinnedToCore(
        rosTask, "RosTask", 8192, NULL, 5, &RosTaskHandle, 1
    );
}

void loop()
{
    // O loop principal permanece em standby devido à task dedicada
    vTaskDelay(pdMS_TO_TICKS(100));
}