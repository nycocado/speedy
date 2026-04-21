#include <Arduino.h>
#include <Bluepad32.h>

// Array para armazenar os controles conectados
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf(">>> SUCESSO: Machenike G5 Pro (ou similar) conectado no slot %d <<<\n", i);
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Vendor ID: %04x, Product ID: %04x\n", properties.vendor_id, properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("AVISO: Controle conectou, mas nao ha mais slots disponiveis.");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf(">>> AVISO: Controle desconectado do slot %d <<<\n", i);
            myControllers[i] = nullptr;
            break;
        }
    }
}

void processGamepad(ControllerPtr ctl) {
    if (ctl->a()) Serial.println("Acao: Botao A Pressionado!");
    if (ctl->b()) Serial.println("Acao: Botao B Pressionado!");
    if (ctl->x()) Serial.println("Acao: Botao X Pressionado!");
    if (ctl->y()) Serial.println("Acao: Botao Y Pressionado!");

    if (ctl->brake() > 0)    Serial.printf("Gatilho Esquerdo (Freio): %d\n", ctl->brake());
    if (ctl->throttle() > 0) Serial.printf("Gatilho Direito (Acelerador): %d\n", ctl->throttle());

    if (abs(ctl->axisY()) > 50 || abs(ctl->axisX()) > 50) {
        Serial.printf("Joystick Esquerdo -> Eixo X: %d, Eixo Y: %d\n", ctl->axisX(), ctl->axisY());
    }
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("==================================================");
    Serial.println("   SPEEDY: Sistema de Controle Ativado (v2 + BP32) ");
    Serial.println("==================================================");
    Serial.println("Aguardando conexao do Machenike G5 Pro...");

    String fv = BP32.firmwareVersion();
    Serial.print("Motor Bluepad32 Versao: ");
    Serial.println(fv);

    BP32.setup(&onConnectedController, &onDisconnectedController);
}

void loop() {
    BP32.update();
    processControllers();
    delay(50);
}