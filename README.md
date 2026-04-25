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

## Pré-requisitos do Sistema Host

O projeto utiliza **Ansible** para garantir um provisionamento idempotente, limpo e reprodutível, tanto para o ambiente de desenvolvimento local (via Distrobox) quanto para o robô físico (Raspberry Pi 4).

O sistema anfitrião deve dispor das seguintes dependências instaladas e operacionais (a instalação e configuração destas ferramentas no sistema operativo anfitrião são da inteira responsabilidade do utilizador):

- **Ansible**: Motor de automação e provisionamento.
- **Python 3**: Interpretador necessário para a execução local do Ansible (geralmente instalado por defeito na maioria das distribuições Linux).
- **sshpass**: Necessário para a autenticação inicial por palavra-passe via SSH (antes da troca de chaves).
- **Podman** (ou Docker): Motor de contentores utilizado para hospedar o ambiente de desenvolvimento.
- **Distrobox**: Utilitário para gerir o contentor de desenvolvimento integrado ao sistema.

## Como Executar a Automação (Ansible)

A configuração está dividida em três frentes: Host (Seu PC), Distrobox (Seu ambiente de dev) e Robots (O hardware real).

1. Navegue até a pasta de automação:

```bash
cd iot/ansible
```

1. **Preparar o seu PC (Host):**
Instala pacotes do sistema, configura o firewall e a partilha de rede, e cria o container base.

```bash
ansible-playbook -i inventory.ini host/main.yml -K
```

*(O `-K` pedirá a sua senha de `sudo` para permissões de sistema).*

1. **Configurar o seu Ambiente de Dev (Distrobox):**
Configura o Zsh, Powerlevel10k, ROS 2 Desktop, Foxglove e compila o código dentro do seu container isolado.

```bash
ansible-playbook -i inventory.ini distrobox/main.yml
```

*Para entrar no seu ambiente de dev após a instalação, use: `distrobox enter speedy-dev`*

1. **Provisionar o Robô Físico (Raspberry Pi):**
*(Certifique-se que o Pi está ligado à rede ou cabo e verifique o IP/Hostname no ficheiro `inventory.ini`)*

```bash
ansible-playbook -i inventory.ini raspberrypi/main.yml -k -k
```

*(O `-k` pedirá a senha SSH do utilizador no Raspberry Pi).*

## Configuração do ESP32-S3 (N16R8)

O projeto utiliza flags específicas no `platformio.ini` para o hardware N16R8:

- **PSRAM Octal (8MB):** Habilitada para buffer do micro-ROS.
- **USB CDC nativo:** Permite telemetria via porta USB direta do chip.
- **Flash 16MB (Quad Mode):** Particionada para suportar builds complexos de ROS 2.

---

*IADE - Universidade Europeia | Engenharia Informática - 2026*
