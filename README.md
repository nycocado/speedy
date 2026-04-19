# Speedy - Carro Autônomo Ackermann Reativo

Projeto de um veículo autônomo de corrida focado em navegação reativa, utilizando arquitetura distribuída e controle manual independente via ESP32-S3.

## 📂 Estrutura do Projeto

O projeto está organizado nas seguintes pastas para a entrega final:

### 1. 🌐 iot/ (Internet of Things & Robotics)

Contém todo o código-fonte e lógica de controle do robô:

- **`firmware/`**: Código do ESP32-S3 (Controle de motores, sensores Hall/MPU6050, Bluetooth Machenike e Micro-ROS).
- **`software/`**: Workspace do ROS 2 Humble para o Raspberry Pi 4B (Processamento LiDAR, Visão Computacional YOLO e Navegação Nav2).
- **`scripts/`**: Scripts auxiliares para instalação de dependências e configuração do sistema.

### 2. 🖥️ gui/ (Graphical User Interface)

Contém os elementos de interface e visualização:

- Configurações do **Foxglove Studio** para telemetria em tempo real e visualização de sensores.
- Arquivos de layout para monitoramento de câmeras e status do sistema.

### 3. 📂 media/ (Recursos e Documentação)

Contém todos os materiais de suporte e evidências:

- **Documentação:** Diagramas elétricos, esquemas de montagem e relatórios técnicos.
- **Modelos IA:** Arquivos de pesos e configuração para YOLOv8/11 (ex: `.tflite`, `.blob`).
- **Mídia:** Fotos e vídeos demonstrativos do protótipo em funcionamento.

---

## 🏎️ Especificações Técnicas

- **Processamento:** Raspberry Pi 4B (4GB) + ESP32-S3.
- **Cinemática:** Ackermann com tração traseira e direção via servo de alto torque.
- **Sensores:** LiDAR 2D, Câmera, IMU MPU6050 e Sensores Hall.
- **Controle:** Manual via Bluetooth (Controle Machenike) com prioridade de override sobre o modo autônomo.
- **Energia:** Bateria LiPo 3S com supercapacitores (1F 5V) para estabilização de tensão do RPi.

*Projeto acadêmico - 2026.*
