#pragma once

#include <DriveControl.h>
#include <Gamepad.h>
#include <ROSCore.h>

/**
 * @class ROSComms
 * @brief Encapsula a comunicação do ROS 2 (inscrições, publicações e
 * callbacks). Isola a complexidade e alocações estáticas da API C do micro-ROS.
 */
class ROSComms
{
    public:
        /**
         * @brief Inicializa as inscrições e vincula os callbacks ao executor.
         * @param rosCore Referência para o núcleo do ROS.
         * @param driveControl Referência para o controlador de condução.
         * @param gamepad Referência para o estado do gamepad.
         * @return true se inicializado com sucesso.
         */
        static bool
        init(ROSCore& rosCore, DriveControl& driveControl, Gamepad& gamepad);

        /**
         * @brief Limpa as inscrições e publicações.
         * @param rosCore Referência para o núcleo do ROS.
         */
        static void destroy(ROSCore& rosCore);

        /**
         * @brief Retorna o tempo da última mensagem recebida (em
         * milissegundos).
         * @return Timestamp gerado pelo millis().
         */
        static uint32_t getLastMessageTime();

        /**
         * @brief Publica uma mensagem de log no tópico /speedy/debug.
         * @param message Texto a ser publicado (max 127 caracteres).
         */
        static void publishDebug(const char* message);
};
