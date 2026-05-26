# Speedy - Carro Autônomo Reactivo
**Data de Publicação:** 26/05/2026

**Grupo:**
- Nycolas Souza - 20230989
- Luan Ribeiro - 20230692
- Lohanne Guedes - 20220085
- Kira Sousa - 20231205

---

## 1. Introdução
O **Project Speedy** consiste num veículo robótico de corrida em escala 1:10, concebido para navegação reativa de alta performance. O projeto, que inicialmente explorava uma arquitetura híbrida entre microcontroladores independentes e computação de bordo, evoluiu para uma solução **standalone centralizada no Raspberry Pi 4**. Esta mudança estratégica visou eliminar latências de comunicação série e garantir uma sincronização determinística entre a percepção visual de alta frequência e a atuação física em tempo real.

Desenvolvido no contexto da Licenciatura em Engenharia Informática no **IADE - Universidade Europeia**, o Speedy integra conceitos avançados de robótica móvel, sistemas distribuídos via **ROS 2 Jazzy** e automação de infraestrutura. O sistema destaca-se pela sua capacidade de operar em ambientes dinâmicos, utilizando uma fusão de percepção baseada em Deep Learning e controle clássico de precisão para atingir velocidades de cruzeiro competitivas em pista.

## 2. Funcionalidades Implementadas
O sistema opera de forma autónoma através de uma orquestração complexa de modos de operação e controladores de malha fechada:

- **Navegação Reativa Baseada em Visão:** O núcleo do guiamento utiliza uma arquitetura de controle PID (Proporcional, Integral e Derivativo) que processa o erro lateral e o erro de heading extraídos em tempo real de um detector de linhas via OpenCV. Este controlador é capaz de antecipar curvaturas da pista através de feedforward, garantindo trajetórias fluidas e minimizando oscilações em retas.
- **Controle de Velocidade em Malha Fechada:** A tração do veículo é gerida por um controlador PID dedicado que funde a leitura de sensores Hall de alta resolução com a cinemática do motor. Este sistema compensa variações de carga e atrito, permitindo uma resposta linear e previsível da velocidade linear, essencial para a estabilidade do sistema de visão.
- **Percepção e Inteligência de Bordo:** O sistema integra detecção de obstáculos baseada em redes neuronais convolucionais (**YOLOv8**) otimizadas para hardware móvel. Esta camada de inteligência permite a classificação e localização de painéis e obstáculos, permitindo que o controlador reativo execute desvios dinâmicos sem perder a referência da pista.
- **Gestão de Estados e Navegação Cega:** Em situações de perda temporária de referências visuais ou transições em rampas, o robô utiliza fusão sensorial (LiDAR e IMU) para manter o heading através de um Filtro de Kalman Estendido. O sistema transita de forma transparente entre modos de guiamento visual e odometria inercial, garantindo a continuidade da operação.
- **Infraestrutura de Telemetria e Diagnóstico:** Implementação de uma ponte de dados via WebSocket integrada com o **Foxglove Studio**, permitindo a visualização síncrona de streams de vídeo, nuvens de pontos LiDAR e logs de estado interno dos controladores, facilitando a depuração remota e o ajuste fino de ganhos de controle.

## 3. Descrição da Arquitetura Implementada
A arquitetura do sistema foi estruturada para maximizar a modularidade e garantir a execução em tempo real de tarefas críticas:

### 3.1. Arquitetura de Software
O software foi desenvolvido sob o ecossistema ROS 2, utilizando C++ para as camadas de hardware e Python para a lógica de alto nível:
- **Camada de Controle (Hardware Interface):** Implementada em C++, utiliza daemons de DMA para garantir a precisão dos sinais PWM e threads dedicadas para o processamento de interrupções dos encoders, eliminando o jitter induzido pelo sistema operativo.
- **Camada de Percepção:** Utiliza processamento paralelo para inferência de IA e visão computacional tradicional, alimentando o controlador com métricas de erro normalizadas.
- **Camada de Navegação:** Implementa controladores Ackermann que traduzem objetivos cinemáticos em comandos físicos, gerindo a transição suave entre diferentes comportamentos (Seguimento, Desvio, Rampa).

### 3.2. Arquitetura de Hardware
- **Processamento:** Raspberry Pi 4 (8GB) como unidade central de processamento.
- **Sensores:** LiDAR ToF (LDROBOT D500), IMU MPU6050 e sensores Hall de alta sensibilidade.
- **Atuadores:** Driver de potência **BTS7960** para controle robusto do motor DC e servo de alto torque para a direção Ackermann.
- **Estrutura:** Chassi escala 1:10 com geometria de direção otimizada e calibração via regressão polinomial para linearizar a resposta do servo.

## 4. Atividades Realizadas e Distribuição de Tarefas
A implementação do projeto foi dividida de forma a potenciar as competências específicas de cada membro:

- **Nycolas Souza:** Desenvolvimento integral do firmware (C++), implementação da `HardwareInterface` do ROS 2, arquitetura do sistema de visão computacional e lógica dos controladores de navegação.
- **Kira Sousa:** Responsável pelo treinamento e otimização do dataset de IA, gestão das sessões no Roboflow e exportação dos modelos YOLOv8 para inferência NCNN em tempo real.
- **Luan Ribeiro:** Engenharia de hardware e desenho do sistema elétrico. Responsável pela calibração matemática da direção (regressão polinomial) e montagem mecânica do chassi.
- **Lohanne Guedes:** Prototipagem física e integração de laboratório. Liderou a esquematização elétrica, soldadura da eletrônica de bordo e configuração da infraestrutura de rede e hotspot do sistema.

## 5. Automação e Provisionamento
O projeto adota uma filosofia de **Infrastructure as Code**, garantindo que todo o robô pode ser provisionado de forma idempotente:
- **Ansible Playbooks:** Automatizam desde a configuração do kernel para hardware real até a criação de ambientes de desenvolvimento isolados (Distrobox).
- **Deployment:** O sistema permite a compilação cruzada e o deploy contínuo do workspace ROS 2, garantindo agilidade no ciclo de desenvolvimento e testes físicos.

---
*IADE - Universidade Europeia | Engenharia Informática - 2026*
