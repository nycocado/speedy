#include "MotorController.h"

MotorController::MotorController(
    uint8_t pinEn,
    uint8_t pinLpwm,
    uint8_t pinRpwm,
    const MotorConfig& config
)
    : _pinEn(pinEn), _pinLpwm(pinLpwm), _pinRpwm(pinRpwm), _config(config),
      _maxSpeed(0), _currentDirection(MotorDirection::STOPPED),
      _deadbandStartTime(0), _inDeadband(false)
{
}

bool MotorController::begin()
{
    // Calcula limite do PWM com base na resolução configurada
    _maxSpeed = (1 << _config.resolution) - 1;

    pinMode(_pinEn, OUTPUT);
    digitalWrite(_pinEn, LOW); // Inicializa desabilitado por segurança

    // No ESP32-S3, garante que os pinos de sinal estejam configurados para
    // saída
    pinMode(_pinLpwm, OUTPUT);
    pinMode(_pinRpwm, OUTPUT);

    // Vincula os pinos aos canais PWM gerenciados pelo Arduino Core v3
    if (ledcAttach(_pinLpwm, _config.frequency, _config.resolution) == 0)
        return false;
    if (ledcAttach(_pinRpwm, _config.frequency, _config.resolution) == 0)
        return false;

    _clearPwm();
    _currentDirection = MotorDirection::STOPPED;
    _inDeadband = false;

    return true;
}

void MotorController::setEffort(float effort)
{
    // Garante que o esforço está dentro do intervalo lógico [-1, 1]
    effort = constrain(effort, -1.0f, 1.0f);

    // Inversão lógica
    if (_config.invertDirection)
    {
        effort = -effort;
    }

    // Zona morta para acionar o freio
    if (abs(effort) < _config.minEffort)
    {
        brake();
        _inDeadband = false;
        return;
    }

    // Mapeia o esforço normalizado para a resolução real de PWM do ESP32
    int16_t speed = (int16_t)(effort * _maxSpeed);

    MotorDirection targetDirection =
        (speed > 0) ? MotorDirection::FORWARD : MotorDirection::REVERSE;

    // Gerenciamento de dead-time não-bloqueante na inversão de rotação
    // O delay de proteção é aplicado apenas na inversão brusca de sentido
    // (FORWARD <-> REVERSE)
    if (targetDirection != _currentDirection &&
        _currentDirection != MotorDirection::STOPPED)
    {
        if (!_inDeadband)
        {
            _clearPwm();
            digitalWrite(_pinEn, LOW);
            _inDeadband = true;
            _deadbandStartTime = millis();
            return;
        }
        else
        {
            // Aguarda o tempo de segurança configurado (dead-time)
            if (millis() - _deadbandStartTime < _config.deadbandMs)
            {
                return;
            }
            _inDeadband = false;
        }
    }
    else
    {
        _inDeadband = false;
    }

    _currentDirection = targetDirection;
    digitalWrite(_pinEn, HIGH); // Habilita a ponte H

    // Aplica sinal PWM no canal correspondente ao sentido desejado
    if (_currentDirection == MotorDirection::FORWARD)
    {
        ledcWrite(_pinLpwm, 0);
        ledcWrite(_pinRpwm, (uint32_t)abs(speed));
    }
    else
    {
        ledcWrite(_pinRpwm, 0);
        ledcWrite(_pinLpwm, (uint32_t)abs(speed));
    }
}

void MotorController::brake()
{
    _clearPwm();
    // EN ativo com PWM 0 resulta em freio dinâmico na ponte BTS7960
    digitalWrite(_pinEn, HIGH);
    _currentDirection = MotorDirection::STOPPED;
}

void MotorController::stop()
{
    _clearPwm();
    // EN inativo desabilita a ponte (motores em roda livre)
    digitalWrite(_pinEn, LOW);
    _currentDirection = MotorDirection::STOPPED;
}

void MotorController::_clearPwm()
{
    ledcWrite(_pinLpwm, 0);
    ledcWrite(_pinRpwm, 0);
}
