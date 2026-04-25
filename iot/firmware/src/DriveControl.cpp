#include <Arduino.h>
#include <Constants.h>
#include <DriveControl.h>

DriveControl::DriveControl(MotorController& motor, SteeringController& steering)
    : _motor(motor), _steering(steering), _currentMode(DriveMode::MANUAL)
{
}

void DriveControl::handleManualInput(const Gamepad& gamepad)
{
    if (gamepad.isBraking())
    {
        _motor.brake();
        return;
    }

    // Direção abstrata (A matemática do ângulo não vaza mais pro negócio)
    _steering.setSteering(gamepad.getSteering());

    // Tração abstrata (A matemática de resolução de PWM fica isolada)
    float throttle = gamepad.getThrottle();

    // Utiliza deadzone global (o gamepad lida com a intenção do usuário,
    // o motor vai lidar com a zona morta física no próprio setEffort,
    // mas evitamos repassar ruído)
    if (abs(throttle) > Config::Gamepad::DEADZONE)
    {
        _motor.setEffort(throttle);
    }
    else
    {
        _motor.stop();
    }
}

void DriveControl::update()
{
    // A física da direção (como Slew Rate Limiters) deve ser calculada
    // continuamente independente de quem ditou o "Setpoint" (Manual ou
    // futuramente Autônomo).
    _steering.update();
}
