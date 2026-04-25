#include "SteeringController.h"

SteeringController::SteeringController(
    uint8_t pin,
    const SteeringConfig& config
)
    : _pin(pin), _config(config), _targetPulseUs(config.centerPulseUs),
      _currentPulseUs(config.centerPulseUs), _lastUpdateTime(0)
{
}

void SteeringController::begin()
{
    // Configura a frequência e anexa o pino com os limites absolutos de pulso
    _servo.setPeriodHertz(_config.periodHertz);
    _servo.attach(_pin, _config.minPulseUs, _config.maxPulseUs);

    // Inicia na posição central configurada de forma instantânea
    center();
}

void SteeringController::setSteering(float value)
{
    // Garante que o valor está dentro do intervalo lógico [-1, 1]
    value = constrain(value, -1.0f, 1.0f);

    // Inversão lógica
    if (_config.invertDirection)
    {
        value = -value;
    }

    // Mapeia o valor normalizado para a largura de pulso em microssegundos
    // (alta resolução)
    _targetPulseUs =
        _config.centerPulseUs + (value * (float)_config.maxDeflectionUs);

    // Aplica limites absolutos de segurança configurados para o chassi
    float minSafePulse =
        (float)_config.centerPulseUs - (float)_config.maxDeflectionUs;
    float maxSafePulse =
        (float)_config.centerPulseUs + (float)_config.maxDeflectionUs;

    _targetPulseUs = constrain(_targetPulseUs, minSafePulse, maxSafePulse);
    _targetPulseUs = constrain(
        _targetPulseUs, (float)_config.minPulseUs, (float)_config.maxPulseUs
    );
}

void SteeringController::update()
{
    uint32_t now = millis();

    // Se o Slew Rate estiver desativado (0), movemos instantaneamente
    if (_config.maxSpeedUsPerSec == 0)
    {
        _currentPulseUs = _targetPulseUs;
    }
    else if (_lastUpdateTime > 0)
    {
        // Calcula o tempo decorrido desde a última atualização (em segundos)
        float dt = (now - _lastUpdateTime) / 1000.0f;

        // Calcula a variação máxima permitida neste ciclo
        float maxDelta = _config.maxSpeedUsPerSec * dt;

        // Diferença entre o alvo e o atual
        float error = _targetPulseUs - _currentPulseUs;

        // Limita a variação (Slew Rate)
        if (abs(error) <= maxDelta)
        {
            _currentPulseUs = _targetPulseUs; // Chegou no alvo
        }
        else
        {
            // Move na direção do erro o máximo permitido
            _currentPulseUs += (error > 0) ? maxDelta : -maxDelta;
        }
    }

    _lastUpdateTime = now;

    // Escreve a largura de pulso exata no servo
    _servo.writeMicroseconds((int)_currentPulseUs);
}

void SteeringController::center()
{
    _targetPulseUs = (float)_config.centerPulseUs;
    _currentPulseUs = _targetPulseUs;
    _servo.writeMicroseconds((int)_currentPulseUs);
    _lastUpdateTime = millis();
}
