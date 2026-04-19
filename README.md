# Speedy - Carro Autônomo Reactivo

Projeto de um veículo autônomo de corrida focado em navegação reativa, utilizando arquitetura distribuída e controle manual independente via ESP32-S3.

Este projeto é desenvolvido no âmbito do **3º Ano (6º Semestre) de Engenharia Informática no IADE - Universidade Europeia**.

## Especificações Técnicas

- **Cérebro (Alto Nível):** Raspberry Pi 4B (4GB) - **ROS 2 Jazzy**.
- **Cerebelo (Baixo Nível):** ESP32-S3 (**N16R8**: 16MB Flash, 8MB PSRAM Octal).
- **Visão & Navegação:** LiDAR ToF **LDROBOT D500** + Câmera (YOLOv8/11).
- **Atuadores:** Servo MG996R (Direção Ackermann) + Driver **BTS7960** (Tração Traseira).
- **Sensores:** IMU MPU6050 (Estabilização) + Sensor Hall A3144 (Velocidade/Odometria).
- **Energia:** Bateria LiPo 3S (11.1V) + Supercapacitores (2.35F @ 5.4V) para estabilização do RPi.

## Estrutura do Projeto

O projeto está organizado nas seguintes pastas:

### 1. iot/ (Internet of Things & Robotics)

- **`firmware/`**: Código do ESP32-S3 (Bluetooth Bluepad32, Micro-ROS, PID de Velocidade).
- **`software/`**: Workspace ROS 2 Jazzy para o Raspberry Pi.
- **`scripts/`**: Scripts auxiliares para instalação e configuração do sistema.

### 2. gui/ (Graphical User Interface)

- Configurações do **Foxglove Studio** para telemetria em tempo real.
- Arquivos de layout para monitoramento de câmeras e status.

### 3. media/ (Recursos e Documentação)

- **`docs/`**: Diagramas elétricos, esquemas de montagem e relatórios técnicos.
- **`models/`**: Arquivos de pesos e configuração para YOLOv8/11 (ex: `.tflite`, `.onnx`).
- **`mídia/`**: Fotos e vídeos demonstrativos do protótipo.

## Bibliotecas (ESP32-S3)

- `micro_ros_platformio`
- `ESP32Servo`
- `PID`
- `Adafruit MPU6050`
- `Adafruit Unified Sensor`
- `Madgwick`
- `CircularBuffer`
- `ArduinoJson`

## Pré-requisitos (Host - Fedora)

Para compilar o firmware do ESP32 e o ambiente micro-ROS:

```bash
# Instalação de ferramentas de build e compiladores C++
sudo dnf install -y cmake gcc-c++ python3-pip python3-devel git
```

## 🔌 Configuração do ESP32-S3 (N16R8)

O projeto utiliza flags específicas no `platformio.ini` para o hardware N16R8:

- **PSRAM Octal (8MB):** Habilitada para buffer do micro-ROS.
- **USB CDC nativo:** Permite telemetria via porta USB direta do chip.
- **Flash 16MB (Quad Mode):** Particionada para suportar builds complexos de ROS 2.

---

*IADE - Universidade Europeia | Engenharia Informática - 2026*
