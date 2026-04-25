#include <Arduino.h>
#include <ROSComms.h>
#include <sensor_msgs/msg/joy.h>
#include <std_msgs/msg/string.h>

// --- Contexto Local ---
static DriveControl* _driveControlPtr = nullptr;
static Gamepad* _gamepadPtr = nullptr;

static rcl_subscription_t _joySubscriber;
static sensor_msgs__msg__Joy _joyMsg;
static float _joyAxes[20];
static int32_t _joyButtons[20];

static rcl_publisher_t _debugPublisher;
static std_msgs__msg__String _debugMsg;
static char _debugBuffer[128];

static uint32_t _lastMsgTime = 0;

/**
 * @brief Callback de processamento do Joystick.
 */
static void joy_callback(const void* msgin)
{
    _lastMsgTime = millis();

    if (!_driveControlPtr || !_gamepadPtr)
        return;

    const sensor_msgs__msg__Joy* msg = (const sensor_msgs__msg__Joy*)msgin;

    // Traduz os eixos e botões do ROS para estados lógicos
    _gamepadPtr->update(msg);

    // Publica o estado atual para monitoramento remoto (Limitado a 10Hz para
    // estabilidade)
    static uint32_t last_debug_pub = 0;
    if (millis() - last_debug_pub > 100)
    {
        snprintf(
            _debugBuffer,
            sizeof(_debugBuffer),
            "[Gamepad] Steer: %5.2f | Throttle: %5.2f | Brake: %d",
            _gamepadPtr->getSteering(),
            _gamepadPtr->getThrottle(),
            _gamepadPtr->isBraking()
        );
        ROSComms::publishDebug(_debugBuffer);
        last_debug_pub = millis();
    }

    // Gerencia transição de modos
    if (_gamepadPtr->requestAutonomous())
    {
        _driveControlPtr->setMode(DriveMode::AUTONOMOUS);
    }
    else if (_gamepadPtr->requestManual() || _gamepadPtr->isBraking())
    {
        _driveControlPtr->setMode(DriveMode::MANUAL);
    }
}

bool ROSComms::init(
    ROSCore& rosCore,
    DriveControl& driveControl,
    Gamepad& gamepad
)
{
    _driveControlPtr = &driveControl;
    _gamepadPtr = &gamepad;

    // Inicialização da mensagem de entrada (Joy)
    _joyMsg.axes.data = _joyAxes;
    _joyMsg.axes.capacity = 20;
    _joyMsg.axes.size = 0;
    _joyMsg.buttons.data = _joyButtons;
    _joyMsg.buttons.capacity = 20;
    _joyMsg.buttons.size = 0;

    // Assinatura do tópico de controle
    if (rclc_subscription_init_best_effort(
            &_joySubscriber,
            rosCore.getNode(),
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
            "/joy"
        ) != RCL_RET_OK)
    {
        return false;
    }

    // Inicialização da mensagem de saída (Log)
    _debugMsg.data.data = _debugBuffer;
    _debugMsg.data.capacity = sizeof(_debugBuffer);
    _debugMsg.data.size = 0;

    // Publicador para telemetria de texto
    if (rclc_publisher_init_default(
            &_debugPublisher,
            rosCore.getNode(),
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "/speedy/debug"
        ) != RCL_RET_OK)
    {
        return false;
    }

    // Registro no executor
    if (rclc_executor_add_subscription(
            rosCore.getExecutor(),
            &_joySubscriber,
            &_joyMsg,
            &joy_callback,
            ON_NEW_DATA
        ) != RCL_RET_OK)
    {
        return false;
    }

    return true;
}

void ROSComms::destroy(ROSCore& rosCore)
{
    rcl_subscription_fini(&_joySubscriber, rosCore.getNode());
    rcl_publisher_fini(&_debugPublisher, rosCore.getNode());
    _driveControlPtr = nullptr;
    _gamepadPtr = nullptr;
}

void ROSComms::publishDebug(const char* message)
{
    strncpy(_debugBuffer, message, sizeof(_debugBuffer) - 1);
    _debugBuffer[sizeof(_debugBuffer) - 1] = '\0';
    _debugMsg.data.size = strlen(_debugBuffer);
    rcl_publish(&_debugPublisher, &_debugMsg, NULL);
}

uint32_t ROSComms::getLastMessageTime() { return _lastMsgTime; }
