<div align="center">

![Speedy](media/photos/analogue-01.png)

# Speedy

Um veículo de corrida autónomo reativo que combina seguimento de faixa por visão, desvio de obstáculos por LiDAR e _heading-hold_ por IMU/odometria, tudo standalone num Raspberry Pi 4 sob ROS 2 Jazzy.

[![License: CC BY-NC 4.0](https://img.shields.io/badge/License-CC%20BY--NC%204.0-3da639.svg)](LICENSE)
![Status](https://img.shields.io/badge/status-conclu%C3%ADdo-6f42c1)

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![C++](https://img.shields.io/badge/C%2B%2B-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp)
[![Python](https://img.shields.io/badge/Python-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?logo=opencv&logoColor=white)](https://opencv.org/)
[![YOLO11n](https://img.shields.io/badge/YOLO11n-NCNN-00FFFF)](https://github.com/ultralytics/ultralytics)
[![Ansible](https://img.shields.io/badge/Ansible-EE0000?logo=ansible&logoColor=white)](https://www.ansible.com/)
[![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi%204-A22846?logo=raspberrypi&logoColor=white)](https://www.raspberrypi.com/)

[English](README.md) | Portuguese

</div>

## Sobre

O **Speedy** é um veículo de corrida autónomo construído para navegação reativa de alta performance em pista. O projeto começou como uma arquitetura híbrida — um Raspberry Pi cuidando da perceção junto de um ESP32-S3 a correr micro-ROS para a atuação de baixo nível, ligados por XRCE-DDS — e evoluiu para um desenho **standalone no Raspberry Pi 4**, onde a própria interface de hardware corre como um plugin `ros2_control` no Pi. Mover toda a stack para uma única placa eliminou o vaivém serial entre perceção e atuação, de modo que o pipeline de visão e o loop de controlo passaram a partilhar um orçamento de tempo determinístico.

Construído para a Licenciatura em Engenharia Informática do **IADE — Universidade Europeia**, o Speedy corre inteiramente em **ROS 2 Jazzy** e combina controlo clássico com deep learning embarcado: um seguidor de faixa PID reage a um detetor de linhas OpenCV em tempo real, um planeador local baseado em LiDAR veta trajetórias inseguras, e um modelo YOLO11n (exportado para NCNN) identifica obstáculos e sinalética de pista.

## Como funciona

- **Direção guiada por visão.** O [`speedy_vision`](iot/software/src/speedy_vision) varre o frame da câmara em busca das duas fitas da pista, rastreia-as de forma independente entre frames (em vez de as colapsar num único centroide) e publica o erro lateral e de heading. O controlador reativo do [`speedy_navigation`](iot/software/src/speedy_navigation) transforma isso num comando de direção com um loop PID e feedforward de heading, conseguindo antecipar curvaturas em vez de apenas reagir a elas.
- **A visão propõe, o LiDAR veta.** Em vez de disputar com a saída da visão sempre que surge um obstáculo, o controlador reativo simula ~20 arcos de direção candidatos a cada ciclo, descarta os que um scan de LiDAR indica que colidiriam, e escolhe entre os arcos seguros o mais parecido com o que a câmara pediu (uma variante leve do Dynamic Window Approach). Isto permite ao carro atravessar aberturas estreitas e chicanes suavemente, sem sobre-corrigir.
- **Heading-hold às cegas.** Quando as duas fitas da pista se perdem (por exemplo, a meio de uma rampa), o controlador recorre a um heading-hold guiado pelo yaw absoluto do EKF, até as fitas serem reencontradas.
- **Gestão da rampa.** Uma deteção de `FLOOR` pelo modelo de obstáculos (bounding box acima de um tamanho mínimo, confirmada por vários frames consecutivos) suprime o detetor de linhas, já pouco fiável nesse troço, mantém o último yaw confiável de linha reta, e conduz com uma velocidade fixa de momento para levar o carro sobre a crista da rampa.
- **Deteção de obstáculos e sinalética.** O [`obstacle_detector_node`](iot/software/src/speedy_vision/speedy_vision/obstacle_detector_node.py) corre o modelo YOLO11n/NCNN num processo dedicado que consome sempre o frame mais recente — frames atrasados são descartados em vez de enfileirados, para a latência de inferência nunca se acumular no Pi 4.
- **Máquina de estados e segurança.** O [`speedy_supervisor`](iot/software/src/speedy_supervisor) arbitra entre os controladores manual (joystick) e autónomo do `ros2_control` através de uma combinação de botões do comando, e mantém um E-stop com latch que zera toda a saída dos atuadores até ser explicitamente limpo.
- **Interface de hardware determinística.** O [`speedy_control`](iot/software/src/speedy_control) é uma interface de hardware `ros2_control` em C++ que controla o PWM do motor e o driver BTS7960, os pulsos do servo via `pigpio`, e a odometria dos encoders Hall via `libgpiod` — mantida fora do caminho Python/DDS para evitar jitter induzido pelo sistema operativo.
- **Telemetria remota.** Uma ponte WebSocket com o [Foxglove](https://foxglove.dev/) transmite câmara, nuvens de pontos do LiDAR e diagnósticos dos controladores em tempo real, para afinar ganhos e depurar sem monitor ligado ao carro.

## Arquitetura

### Hardware

| Componente      | Peça                                                            |
| --------------- | --------------------------------------------------------------- |
| Processamento   | Raspberry Pi 4 (4GB)                                            |
| LiDAR           | LDROBOT D500 / LD19 (ToF, 230400 baud)                          |
| IMU             | MPU-6050 (DMP embarcado)                                        |
| Câmara          | Raspberry Pi Camera OV5647 160°, 640×480                        |
| Odometria       | 5× sensores de efeito Hall (2 rodas dianteiras + eixo do motor) |
| Motor de tração | JGB37-520 12V 600RPM, redução 2.3:1                             |
| Driver de motor | Ponte H BTS7960                                                 |
| Direção         | Servo MG996R, geometria Ackermann, entre-eixos de 0.251 m       |
| Alimentação     | LiPo 3S 11.1V 5000mAh                                           |

<table>
<tr>
<td width="50%"><img src="media/photos/studio-01.png" width="100%"><br>Compartimento de computação — portas do Pi 4, suporte do LiDAR LD19, cabo flat da câmara</td>
<td width="50%"><img src="media/photos/studio-02.png" width="100%"><br>Dissipador do driver de motor e suporte do LiDAR, visto do outro lado</td>
</tr>
</table>

### Software — workspace ROS 2 (`iot/software/src`)

| Pacote               | Responsabilidade                                                                                                                                                                 |
| -------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `speedy_bringup`     | Launch files e configuração de cada subsistema (câmara, IMU, LiDAR, EKF, controladores, Foxglove)                                                                                |
| `speedy_control`     | Interface de hardware `ros2_control` em C++ (motor, servo, encoders Hall)                                                                                                        |
| `speedy_vision`      | Detetor de linhas (OpenCV) e detetor de obstáculos/sinalética (YOLO11n NCNN)                                                                                                     |
| `speedy_navigation`  | Controlador reativo: seguimento de faixa PID + arcos de direção vetados por LiDAR + heading-hold                                                                                 |
| `speedy_supervisor`  | Máquina de estados manual/autónomo e E-stop                                                                                                                                      |
| `speedy_calibration` | Calibração pulso do servo → ângulo de esterço                                                                                                                                    |
| `speedy_teleop`      | Teleop por joystick, desacoplado do controlador ativo no momento                                                                                                                 |
| `speedy_dataset`     | Recolha de imagens em pista para o dataset do YOLO                                                                                                                               |
| `speedy_description` | Modelo do robô em URDF/xacro, parametrizado a partir do `hardware.yaml`                                                                                                          |
| `deps/`              | Dependências ROS 2 vendorizadas (`camera_ros`, `ldlidar_ros2`, `robot_localization`, `imu_tools`, `foxglove-sdk`, `rosx_introspection`, `mpu6050_driver`, …) como submódulos git |

Perceção e controlo estão totalmente desacoplados do provisionamento: o [`iot/ansible`](iot/ansible) é uma camada de Infrastructure-as-Code que provisiona o Pi a partir de uma imagem Raspberry Pi OS limpa (overlays de kernel, rede/hotspot, ROS 2, build do workspace, auto-start via systemd) e replica o mesmo workspace num container Distrobox para desenvolvimento em qualquer host Linux.

## Perceção e fusão sensorial

- **Câmara.** A OV5647 é calibrada (`camera_info.yaml`) com um alvo checkerboard; os coeficientes de intrínsecos/distorção resultantes alimentam o passo opcional de undistortion no coletor de dataset, e ficam disponíveis a qualquer nó via `camera_info`.
- **LiDAR.** O driver do LD19 corta o scan cru aos 180° frontais (`angle_crop_min/max = 90°/270°`), mascarando a metade traseira como NaN — isto evita que o scan-matching e o planeador mini-DWA reajam a obstáculos atrás do carro, e reduz para metade os pontos processados a cada ciclo.
- **IMU.** O MPU6050 corre num modo híbrido: a orientação vem do **DMP** embarcado no chip (correção contínua do bias do giroscópio, pelo que o yaw não deriva como aconteceria integrando a taxa crua), enquanto a aceleração linear e a velocidade angular são lidas dos registos crus, cuja escala é mais fiável que a do FIFO do DMP. Uma calibração de bias one-shot de 200 amostras corre no arranque, com o carro parado.
- **EKF (`robot_localization`).** Funde a odometria Hall das rodas dianteiras (apenas `vx`) com o yaw absoluto do DMP e a taxa de yaw do IMU a 20 Hz em modo 2D, publicando a transformação `odom → base_link`. É este yaw fundido que serve de âncora ao heading-hold do controlador reativo quando as fitas da pista se perdem — não deriva como uma integração crua do giroscópio derivaria ao longo de um ponto cego de vários segundos, como uma rampa.

## Loops de controlo

Três loops encadeados transformam o objetivo de seguir a faixa em comandos de motor e servo:

1. **Externo — trajetória (`speedy_navigation`).** Um PID sobre o erro lateral e de heading (mais feedforward de curvatura) propõe um ângulo de esterço a partir do detetor de linhas; a camada mini-DWA veta os arcos, entre ~20 candidatos, que um scan de LiDAR indica que colidiriam, e vence o arco seguro mais próximo da proposta da visão.
2. **Intermédio — cinemática (`ros2_control`).** Em `AUTO`, o `bicycle_steering_controller` converte esse ângulo de esterço e a velocidade em comandos individuais por junta via cinemática Ackermann. Em `MANUAL`, o `manual_steering_controller`/`manual_drive_controller` passa o joystick diretamente. O `speedy_supervisor` alterna entre os dois conjuntos de controladores de forma **atómica** (`SwitchController` com strictness `STRICT`) numa combinação de botões do joystick, e só atualiza o seu próprio estado depois de a troca ser confirmada — nunca de forma otimista.
3. **Interno — atuação (`speedy_control`).** A interface de hardware fecha um PID de velocidade (`kp=1.0`, `ki=0.4`) com feedforward sobre a velocidade derivada dos encoders Hall (filtro exponencial, timeout de velocidade zero para uma paragem limpa), e mapeia o ângulo de esterço comandado para um pulso de servo através de uma **regressão quadrática** (`pulse = a0 + a1·deg + a2·deg²`) em vez de linear — ajustada conduzindo o carro e relacionando o pulso comandado com o ângulo físico estimado a partir da velocidade angular e da velocidade medidas (modelo bicycle), o que corrigiu cerca de 3° de assimetria por folga (_backlash_) na direção que um mapeamento linear não conseguia captar.

## Dataset e treino do modelo

- O [`speedy_dataset`](iot/software/src/speedy_dataset) captura frames JPEG únicos ou em rajada ao premir um botão do joystick, **apenas em modo `MANUAL`** (cancela a subscrição fora disso para poupar CPU), com um passo opcional de undistort/grayscale em tempo real e uma thread de escrita em segundo plano para as rajadas não bloquearem o executor do ROS. Estes frames alimentaram o dataset no Roboflow (525 imagens, 5 classes: `BOX`, `FLOOR`, `WALL`, `RAMP`, `TOP`) usado para treinar o modelo de obstáculos — ver [`yolo/README.roboflow.txt`](yolo/README.roboflow.txt).
- O [`yolo/train_yolo.py`](yolo/train_yolo.py) afina um **YOLO11n** a 320px (um quarto do processamento face a 640px, ainda suficiente para objetos grandes como a rampa e as caixas) e exporta-o para **NCNN** para inferência em ARM.
- No dispositivo, o [`yolo_ncnn.py`](iot/software/src/speedy_vision/speedy_vision/yolo_ncnn.py) corre a inferência apenas com NumPy/OpenCV — sem PyTorch no robô — fazendo o letterbox, o decode e o NMS por conta própria. A inferência fica fixada a uma única thread para nunca esfomear o loop de controlo do Pi 4.

## Telemetria

A ponte Foxglove é afinada contra o _buffer bloat_ sobre o hotspot WiFi do robô: um buffer de envio de 2 MB e uma profundidade de QoS de 1 fazem a ponte descartar sempre o backlog em favor do frame mais recente em vez de enfileirar, e o feed de vídeo é o stream da câmara já comprimido em JPEG — nunca o tópico `image_raw` cru — para o WebSocket nunca queimar CPU do Pi 4 a recodificá-lo.

## Requisitos

| Ferramenta | Versão mínima                              |
| ---------- | ------------------------------------------ |
| ROS 2      | Jazzy                                      |
| Ansible    | 2.15+                                      |
| Distrobox  | última (apenas para desenvolvimento no PC) |
| Python     | 3.12+ (treino do YOLO)                     |

## Como executar

```bash
git clone --recursive https://github.com/nycocado/speedy.git
```

**Provisionamento do robô** (Raspberry Pi, a partir de uma instalação limpa do Raspberry Pi OS Lite):

```bash
cd iot/ansible
ansible-playbook -i inventory.ini raspberrypi/main.yml
```

Isto instala o ROS 2 Jazzy nativamente, sincroniza e compila o workspace, e regista o serviço systemd `speedy.service` que lança o `speedy_bringup` automaticamente no arranque.

**Ambiente de desenvolvimento no PC** (container Distrobox espelhando o workspace do robô):

```bash
ansible-playbook -i inventory.ini distrobox/main.yml
```

**Lançamento manual** (robô já provisionado):

```bash
source /opt/ros/jazzy/setup.bash
source ~/speedy_ws/install/setup.bash
ros2 launch speedy_bringup speedy.launch.py
```

**Treino do modelo YOLO** ([`yolo/`](yolo)):

```bash
cd yolo
python train_yolo.py
```

## Estrutura do repositório

```
speedy/
├── iot/
│   ├── software/src/    # Workspace ROS 2 Jazzy — pacotes speedy_* + deps/ vendorizadas
│   └── ansible/         # Provisionamento do robô, setup do host e ambiente de dev Distrobox
├── yolo/                # Configuração do dataset Roboflow e script de treino do YOLO11n
├── media/               # Relatórios das milestones, slides, BOM e diagramas elétricos
└── LICENSE
```

## Documentação

#### Milestone 1

- [Relatório](media/milestone-1/report.pdf) — arquitetura híbrida inicial Pi + ESP32-S3, requisitos e BOM.
- [Slides](media/milestone-1/slides.pdf)
- [Diagrama elétrico](media/milestone-1/circuit.pdf)
- [Diagrama de componentes](media/milestone-1/component.pdf)
- [BOM](media/milestone-1/bom.xlsx)

#### Milestone 2

- [Diagrama elétrico](media/milestone-2/circuit.pdf) — arquitetura final standalone no Raspberry Pi 4.
- [Diagrama de componentes](media/milestone-2/component.pdf)
- [BOM](media/milestone-2/bom.xlsx)
- [Vídeo de demonstração](media/milestone-2/video.mp4)

#### Fotos

- [Fotos de estúdio e analógicas](media/photos) — fotos de detalhe do chassi e da eletrónica.

## Equipa

- [**Nycolas Souza**](https://github.com/nycocado) — firmware (interface de hardware em C++), pipeline de visão computacional, controladores de navegação.
- [**Kira Sousa**](https://github.com/Kira-Sousa) — treino do dataset e gestão das sessões no Roboflow, exportação YOLO11n → NCNN.
- [**Luan Ribeiro**](https://github.com/Ninjaok) — engenharia de hardware, calibração da direção (regressão via modelo bicycle), montagem do chassi.
- [**Lohanne Guedes**](https://github.com/lohanneguedes) — prototipagem física, fiação eletrónica, infraestrutura de rede e hotspot.

## Licença

Distribuído sob a licença **CC BY-NC 4.0**, © 2026 Nycolas Souza, Luan Ribeiro, Lohanne Guedes, Kira Sousa.

Permite a partilha e adaptação do trabalho com atribuição, exclusivamente para fins não comerciais. O texto completo está em [LICENSE](LICENSE).
