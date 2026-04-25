#include <ROSManager.h>
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>

RosState ROSManager::currentState = RosState::WAITING_AGENT;

bool ROSManager::create_entities(
    ROSCore& rosCore,
    DriveControl& driveControl,
    Gamepad& gamepad
)
{
    if (!rosCore.begin("speedy_base_node"))
        return false;
    if (!rosCore.initExecutor(1))
        return false;
    if (!ROSComms::init(rosCore, driveControl, gamepad))
        return false;
    return true;
}

void ROSManager::destroy_entities(ROSCore& rosCore)
{
    ROSComms::destroy(rosCore);
    rosCore.end();
}

void ROSManager::manageConnection(
    ROSCore& rosCore,
    MotorController& motor,
    DriveControl& driveControl,
    Gamepad& gamepad
)
{
    switch (currentState)
    {
        case RosState::WAITING_AGENT:
            if (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
            {
                currentState = RosState::AGENT_AVAILABLE;
            }
            else
            {
                motor.brake(); // Garante parada enquanto espera conexão
            }
            break;

        case RosState::AGENT_AVAILABLE:
            if (create_entities(rosCore, driveControl, gamepad))
            {
                currentState = RosState::AGENT_CONNECTED;
            }
            else
            {
                destroy_entities(rosCore);
                currentState = RosState::WAITING_AGENT;
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            break;

        case RosState::AGENT_CONNECTED:
        {
            static uint32_t last_ping = 0;
            static uint8_t missed_pings = 0;

            // Se o robô não recebeu nada nos últimos 200ms, aí sim aciona o
            // Ping bloqueante
            if (millis() - ROSComms::getLastMessageTime() > 200)
            {
                if (millis() - last_ping > 200)
                {
                    // Ping com timeout de 50ms (mais tolerante que 10ms)
                    if (RMW_RET_OK != rmw_uros_ping_agent(50, 1))
                    {
                        missed_pings++;
                        // Desconecta apenas se perder 3 pings consecutivos
                        // (timeout total de ~600ms)
                        if (missed_pings >= 3)
                        {
                            currentState = RosState::AGENT_DISCONNECTED;
                            missed_pings = 0;
                        }
                    }
                    else
                    {
                        missed_pings = 0; // Resetou o contador porque respondeu
                    }
                    last_ping = millis();
                }
            }
            else
            {
                // Estamos recebendo mensagens ativamente. A conexão está viva!
                // Reseta os contadores para evitar falsos positivos quando as
                // mensagens pararem.
                missed_pings = 0;
                last_ping = millis();
            }

            if (currentState == RosState::AGENT_CONNECTED)
            {
                rosCore.spinSome(0);
            }
        }
        break;

        case RosState::AGENT_DISCONNECTED:
            destroy_entities(rosCore);
            motor.brake();
            currentState = RosState::WAITING_AGENT;
            break;

        default:
            break;
    }
}