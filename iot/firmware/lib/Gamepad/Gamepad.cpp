#include "Gamepad.h"
#include <Arduino.h>

Gamepad::Gamepad(const GamepadConfig& config)
    : _config(config), _rtTouched(false), _ltTouched(false), _steering(0.0f),
      _throttle(0.0f), _braking(false), _reqAutonomous(false), _reqManual(false)
{
}

float Gamepad::_normalizeTrigger(float rawValue, bool& touched)
{
    // O ROS Joy node reporta 0.0 para eixos não tocados na inicialização.
    // O range real de gatilhos (como PS4/Xbox) no ROS2 é [1.0 (solto) a -1.0
    // (pressionado)]. Se recebermos 0.0 e nunca tivermos tocado no gatilho,
    // tratamos como solto (0.0 normalizado).
    if (!touched && (rawValue == 0.0f))
    {
        return 0.0f;
    }

    // Marca como tocado se o valor for diferente de 0.0 pela primeira vez
    if (!touched && (rawValue != 0.0f))
    {
        touched = true;
    }

    // Mapeamento: 1.0 (solto) -> 0.0, -1.0 (pressionado) -> 1.0
    float val = (1.0f - rawValue) / 2.0f;
    return constrain(val, 0.0f, 1.0f);
}

void Gamepad::update(const sensor_msgs__msg__Joy* msg)
{
    // Se a mensagem for nula ou vazia, mantém estado seguro.
    if (!msg || (msg->axes.size == 0 && msg->buttons.size == 0))
    {
        _resetState();
        return;
    }

    _parseButtons(msg);
    _parseSteering(msg);
    _parseThrottle(msg);
}

void Gamepad::_resetState()
{
    _steering = 0.0f;
    _throttle = 0.0f;
    _braking = false;
    _reqAutonomous = false;
    _reqManual = false;
}

void Gamepad::_parseButtons(const sensor_msgs__msg__Joy* msg)
{
    _braking = (msg->buttons.size > _config.btnBrake)
                   ? (msg->buttons.data[_config.btnBrake] == 1)
                   : false;
    _reqManual = (msg->buttons.size > _config.btnManual)
                     ? (msg->buttons.data[_config.btnManual] == 1)
                     : false;
    _reqAutonomous = (msg->buttons.size > _config.btnAuto)
                         ? (msg->buttons.data[_config.btnAuto] == 1)
                         : false;
}

void Gamepad::_parseSteering(const sensor_msgs__msg__Joy* msg)
{
    float raw_steer = (msg->axes.size > _config.axisSteering)
                          ? msg->axes.data[_config.axisSteering]
                          : 0.0f;
    _steering = (abs(raw_steer) < _config.deadzone) ? 0.0f : raw_steer;
}

void Gamepad::_parseThrottle(const sensor_msgs__msg__Joy* msg)
{
    float raw_lt = (msg->axes.size > _config.axisReverse)
                       ? msg->axes.data[_config.axisReverse]
                       : 1.0f;
    float raw_rt = (msg->axes.size > _config.axisThrottle)
                       ? msg->axes.data[_config.axisThrottle]
                       : 1.0f;

    float lt = _normalizeTrigger(raw_lt, _ltTouched);
    float rt = _normalizeTrigger(raw_rt, _rtTouched);

    // Throttle: Gatilho de aceleração - Gatilho de ré
    _throttle = rt - lt;

    // Zona morta para aceleração
    if (abs(_throttle) < _config.deadzone)
    {
        _throttle = 0.0f;
    }
}
