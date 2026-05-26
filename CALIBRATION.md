# Calibração do Speedy

## Line Detector (`vision.yaml`)

### Ferramenta principal: Foxglove Studio

Painel `Image` apontado para `/speedy_camera/image_raw/compressed` (JPEG; nunca o cru sobre o hotspot) e ativa o tópico `/line_detector/annotations`. O detector desenha em tempo real:

| Elemento | Significado |
|---|---|
| Retângulo azul | Banda analisada |
| Círculos vermelhos | Pontos (inliers) ajustados à **fita esquerda** |
| Círculos laranja | Pontos (inliers) ajustados à **fita direita** |
| Linha amarela | Centro da lane (média das duas fitas) ou, com 1 fita, a reta dessa fita |
| Texto "SEE L" / "SEE R" | Só uma fita visível (esquerda / direita) → publica `visible_side` ±1; o controller entra em REACQUIRE |
| Texto "NO LANE" | Nenhuma fita ajustada |

> O detector **rastreia as duas fitas como duas retas** (tracking temporal). O heading vem do declive de cada fita real, não de um centro por linha — por isso é estável e o caso de uma fita só (ex. parede a ocultar uma fita) é tratado com degradação segura.

Todos os parâmetros são lidos **a cada frame** → alterações têm efeito imediato. Ajusta-os pelo **painel de Parâmetros do Foxglove** (passa pelo `foxglove_bridge`, que está no grafo DDS). O `ros2 param set` por SSH **não funciona** — processos ad-hoc não entram no grafo (CycloneDDS preso ao wlan0).

> ⚠️ **Setup seguro:** o `line_detector` só subscreve a imagem em **AUTO**. Mas em AUTO o `reactive_controller` comanda os motores — antes de calibrar, põe `reactive_controller/navigation_enabled: false` (robô parado) e `obstacle_detector/enabled: false` (liberta CPU, feed fluido).

---

### Ordem de calibração

#### 1. Banda (`band_bottom_frac`, `band_top_frac`)

O retângulo azul deve cobrir apenas o chão à frente — sem capô na base, sem horizonte ou parede no topo.

| Situação | Ajuste |
|---|---|
| Capô/nariz do carro dentro da banda | Aumenta `band_bottom_frac` (ex: 0.0 → 0.10) |
| Banda sobe até paredes/fundo da sala | Reduz `band_top_frac` (ex: 0.50 → 0.35) |
| Poucas linhas de varredura chegam ao chão | Aumenta `band_top_frac` |

#### 2. Tamanho da fita (`min_edge_w_frac`, `max_edge_w_frac`)

O algoritmo encontra trechos escuros em cada linha horizontal e rejeita os que são demasiado finos (ruído) ou demasiado largos (painel/sombra).

1. Para o robô com boa visibilidade da lane
2. Observa se os círculos vermelhos/laranja estão sobre a fita
3. Se aparecem **no painel** (objeto largo): verifica se `max_edge_w_frac` é baixo o suficiente (painel de 60 cm a ~50 cm ocupa ~60–70% da imagem; `0.18` já o rejeita)
4. Se **não aparecem** onde há fita: a fita pode estar abaixo de `min_edge_w_frac` → reduz para `0.004`–`0.005`

#### 3. Largura da lane (`lane_width_min_frac`, `lane_width_max_frac`, `lane_half_init_frac`)

Com o tracking de duas fitas, estes deixaram de rejeitar pares: `lane_width_min/max` só **validam a separação** antes de atualizar a meia-largura por EMA, e `lane_half_init` é o **offset usado quando só há uma fita** (auto-refina depois).

1. Em Foxglove, pausa um frame com os dois círculos visíveis
2. Mede a distância horizontal entre vermelho e laranja em píxeis
3. Divide pela largura total da imagem → valor de referência

```
lane_width_min_frac  ≈ referência × 0.7
lane_width_max_frac  ≈ referência × 1.3
lane_half_init_frac  ≈ referência / 2
```

#### 4. Parâmetros de robustez

| Parâmetro | Efeito | Quando ajustar |
|---|---|---|
| `n_rows` | Mais linhas de varredura = mais amostras para o ajuste | Aumenta (12 → 16) se a linha amarela tremer |
| `blur_kernel` | Suavização antes do threshold | Aumenta se o Otsu detetar ruído de textura no chão |
| `min_valid_rows` | Mínimo de deteções para publicar | Reduz (4 → 3) se o detector emite NaN em curvas |
| `lane_ema_alpha` | Suavização da meia-largura estimada | Aumenta (→ 0.2) se o robô passa por secções sem visibilidade |
| `max_rate_hz` | Cap de CPU | Manter em 15 (câmera a 30 fps; não há ganho acima) |
| `outlier_reject` / `outlier_max_frac` | Descarta pontos fora da reta de **cada fita** | Baixa `outlier_max_frac` (0.06 → 0.04) se uma marca ainda torce uma fita; sobe se corta curvas reais |
| `assoc_gate_frac` | Distância máx. (fração da largura) p/ associar um candidato à fita prevista | Baixa para rejeitar mais agressivamente obstáculos perto das fitas; sobe se perde a fita em curvas apertadas |
| `track_max_miss` | Frames sem ver uma fita até largar o track (e re-bootstrap) | Sobe para tolerar oclusões mais longas; baixa para re-adquirir mais depressa |

> **Obstáculo entre as fitas:** fica fora do `assoc_gate` das duas → excluído por construção, não chega ao ajuste. Um obstáculo *colado* a uma fita ainda pode entrar; aí é o `outlier_max_frac` (resíduo) que o apara. Caso extremo (obstáculo estreito e alto, muitos pontos colineares): fundir o `obstacle_detector` (mascarar as bounding boxes da banda).

---

### Procedimento de teste físico (passo a passo)

Tudo pelo **Foxglove** (painel de Imagem + painel de Parâmetros), sem SSH.

**A — Montagem (uma vez)**

1. Pista real no chão (fitas no espaçamento verdadeiro), robô na posição de arranque, câmera à altura de montagem normal (a perspetiva depende disto).
2. Painel de Parâmetros: `reactive_controller/navigation_enabled: false` (robô parado) e `obstacle_detector/enabled: false` (CPU livre, feed fluido).
3. Entra em **AUTO** (gamepad: segura SELECT+START) — o `line_detector` só subscreve a imagem em AUTO.
4. Painel de Imagem: `/speedy_camera/image_raw/compressed` + anotações `/line_detector/annotations`.

**B — Front-end** (§1 e §2 acima): acerta a banda (retângulo azul só no chão) e o gate de largura (círculos nas fitas, não em ruído/painel). É o que mais depende da tua câmera/luz.

**C — Largura da lane** (§3): mede a separação vermelho↔laranja (cresce do topo→base da banda, por perspetiva). Visual (coords do cursor no Foxglove) ou captura um JPG via `dataset_collector` (MANUAL, botão), faz `scp` e mede num editor. `lane_half_init_frac ≈ (fração no meio) / 2`.

**D — Validar o tracking das duas fitas**

1. **Duas fitas:** dois conjuntos de círculos (vermelho/laranja), cada um com a sua reta, linha amarela ao centro.
2. **Uma fita** (tapa/contorna uma à mão, simula a parede): deve surgir **"SEE L"/"SEE R"** e o `visible_side` ir a ±1. O controller entra em REACQUIRE e esterça p/ o lado da fita tapada (ver §Navegação). Se o track não persistir na oclusão, sobe `track_max_miss`.
3. **Obstáculo entre as fitas:** os círculos **não** devem cair nele nem a linha amarela saltar. Se cair, baixa `assoc_gate_frac`.
4. Empurra o robô pela pista toda (curvas incluídas) e confere os critérios de "Como saber se a regulação está boa".

---

### Sinais de saída

| Tópico | Valor | Interpretação |
|---|---|---|
| `/line_detector/lateral_error` | −1 a +1 | −1 = lane totalmente à esquerda, 0 = centrado, +1 = à direita |
| `/line_detector/heading_error` | −1 a +1 | Inclinação da lane (negativo = curva à esquerda) |
| `/line_detector/visible_side` | −1, 0, +1 | −1 = só fita esquerda visível, +1 = só direita, 0 = ambas/nenhuma |

`lateral_error`/`heading_error` publicam `NaN` quando não há linha detetada. Quando só há uma fita, `visible_side` vai a ±1 → o controller entra em REACQUIRE (esterça p/ recuperar a fita perdida); sem nenhuma fita cai para heading-hold do EKF (BLIND).

### Como saber se a regulação está boa

Critérios de aceitação (verifica todos em Foxglove, robô em AUTO + `navigation_enabled: false`):

1. **Círculos nas fitas:** vermelho/laranja assentam nas fitas reais, não em ruído do chão nem em obstáculos.
2. **Linha amarela centrada:** fica entre as duas fitas e aponta ao longo da pista.
3. **`lateral_error` monótono:** move o robô à mão da esquerda para a direita da pista — o valor varia suavemente de negativo → 0 (centrado) → positivo, sem saltos.
4. **`heading_error` ~0 em reta**, com sinal correto em curva.
5. **Sem "NO LANE" a piscar** quando a pista está claramente visível.
6. **Imune a obstáculos:** põe um obstáculo na pista — a linha amarela e o `lateral_error` **não** devem saltar (rejeição de outliers a funcionar). Se saltam, baixa `outlier_max_frac`.
7. **Estável no tempo:** num gráfico (plot) do `lateral_error`, o traço é suave, não serrilhado. Se treme, sobe `n_rows` ou `blur_kernel`.

**Teste dinâmico final:** empurra o robô devagar pela pista toda, curvas incluídas. Os critérios 1–4 devem manter-se em todo o percurso.

### Reaproveitar uma calibração antiga

Após o redesign para tracking de duas fitas, a maioria dos valores **carrega direto**:

- **Carregam (mesmo significado):** `band_*`, `n_rows`, `blur_kernel`, `min/max_edge_w_frac`, `lane_ema_alpha`, `max_rate_hz`, `outlier_*` (agora aplicado por fita).
- **Mudaram de papel (valor serve de arranque):** `min_valid_rows` (agora mín. de pontos **por fita** — baixa p/ 3 se uma fita aparece pouco); `lane_width_min/max` (só validam a EMA da meia-largura, já não rejeitam pares).
- **Novos (usa defaults):** `assoc_gate_frac: 0.12`, `track_max_miss: 5`. Mantém sempre `assoc_gate_frac < lane_half_init_frac` (senão troca de lados / não rejeita obstáculo central).
- **Atenção downstream:** a escala do `heading_error` mudou (base fixa + declive por-fita) → **re-afina o `k_heading`** na navegação.

---

## Navegação Reativa (`navigation.yaml`)

O `reactive_controller` funde visão + LiDAR + IMU num comando Ackermann em `/bicycle_steering_controller/reference` (`linear.x` = velocidade m/s, `angular.z` = curvatura). Modos: **VISION** (2 fitas; PID), **REACQUIRE** (1 fita; esterça p/ recuperar a perdida, ex. curva de 90°), **BLIND** (sem fita; segura o yaw do EKF), **RAMP** (pitch do IMU), **+DODGE** (desvio do painel somado por cima).

> **Prioridade REACQUIRE > VISION:** com uma só fita, a estimativa lateral não é fiável numa curva apertada, então vale mais virar p/ recuperar as duas fitas do que seguir uma estimativa errada.

### Setup seguro

⚠️ Em AUTO o controlador **comanda os motores**. Calibra por fases:

1. **Bancada** (rodas no ar): observa o esterço a responder sem o robô fugir.
2. **Pista a baixa velocidade**: `v_base: 0.3` até o seguimento provar-se.
3. **Sobe `v_base`** gradualmente (0.3 → 0.4 → 0.5 → 0.6).

Para parar sem matar o nó: `navigation_enabled: false`. Arbitragem MANUAL/AUTO é do supervisor (gamepad, segurar SELECT+START).

### Debug topics (novos — usa-os para afinar)

O controller publica internamente os seguintes tópicos. Abre um painel **Plot** no Foxglove e adiciona todos:

| Tópico | O que mostra |
|---|---|
| `/reactive_controller/steer_raw` | Saída do PID+feedforward **antes** do rate-limiter e clamp |
| `/reactive_controller/steer_out` | Esterço **final** (após rate-limiter e clamp ±`steer_max`) |
| `/reactive_controller/v_cmd` | Velocidade comandada (m/s) |
| `/reactive_controller/i_lat` | Termo integral acumulado (detetar windup) |
| `/reactive_controller/mode` | Modo ativo: `VISION`, `BLIND`, `RAMP`, `VISION+DODGE`, `STOP`, `DISABLED` |
| `/reactive_controller/diag` | Resumo de todos os valores (Raw Messages, para snapshot rápido) |

Plota **juntos** `/line_detector/lateral_error`, `/reactive_controller/steer_raw`, `/reactive_controller/steer_out` → vês imediatamente se o PID responde ao erro, se o rate-limiter está a cortar, e se o sinal tem o sinal correto.

### Ordem de calibração

#### 1. Segurança (antes de andar depressa)

| Parâmetro | Efeito | Como afinar |
|---|---|---|
| `hard_stop_range` | Distância (m) a que o motor desliga | Põe um obstáculo à frente e confirma que para. Sobe se a travagem a `v_base` for curta demais |
| `safety_fov_deg` | Setor frontal (graus) p/ deteção de parede/parada | Largo demais → para com paredes laterais; estreito demais → não vê painel oblíquo |
| `steer_rate_max` | rad/s: limita a taxa do esterço (suaviza) | Baixa se o servo treme/oscila; sobe se a resposta em curva é lenta |

#### 2. Velocidade base

| Parâmetro | Efeito | Como afinar |
|---|---|---|
| `v_base` | m/s em pista livre | Começa em 0.3. Sobe só depois do PID estável |
| `v_min` | m/s perto de parede / cego | Baixo mas suficiente para manter momento |

#### 3. PID de visão — procedimento passo a passo

O `navigation.yaml` arranca com `ki_lat=0`, `kd_lat=0`, `k_heading=0` para isolar cada parte.

**Passo 1 — Verificar o sinal (bancada, robô no ar)**

Muda para AUTO, `navigation_enabled: true`, `v_base: 0.0` (só esterço, sem motor).  
Desloca a câmera para um lado da pista. No plot, o `lateral_error` vai para positivo (por exemplo) e o `steer_raw` deve ir **no mesmo sentido** (para corrigir, esterçar para o lado da fita). Se `steer_raw` for para o lado **errado**, inverte os três ganhos (`kp_lat`, `ki_lat`, `kd_lat` todos com sinal negativo).

**Passo 2 — Afinar `kp_lat` (só proporcional)**

Com `v_base: 0.3` na pista:

- Sobe `kp_lat` de 0.2 em 0.2 até o robô serpentear (oscilar rapidamente à volta da linha central).
- O ponto onde começa a serpentear → divide por ~1.7 → esse é o `kp_lat` de trabalho.
- Confirma: `lateral_error` oscila perto de 0 com amplitude pequena e suave.
- O gráfico de `steer_raw` deve parecer a imagem espelhada do `lateral_error` (com escala kp).

**Passo 3 — Adicionar `k_heading` (curvas)**

Ainda com `ki_lat=0`, `kd_lat=0`. Aumenta `k_heading` de 0.1 em 0.1.  

- Efeito esperado: entra nas curvas mais cedo (menor corte interior).
- Demasiado alto: o `steer_raw` "salta" no início de cada curva antes do erro lateral crescer.
- Ponto de arranque razoável: 0.1–0.2 (o sinal de heading agora é mais estável do que na versão anterior).

**Passo 4 — Adicionar `kd_lat` (amortecimento)**

Só se ainda houver serpentear depois do passo 2. Sobe devagar (0.01–0.03 de cada vez).  

- Confirma no plot: `steer_raw` deixa de oscilar sem `lateral_error` a pedir.
- Alto demais: o `steer_raw` fica "nervoso" (treme com o ruído do detector).

**Passo 5 — `ki_lat` (offset de regime)**

Só se o robô andar consistentemente descentrado (um lado) com `kp` bem afinado.  

- Sobe muito devagar (0.02–0.05 de cada vez). Verifica `i_lat` no plot: deve ser pequeno e estável (não crescer sem parar). Se cresce indefinidamente → `i_max` está alto demais ou `kp` está baixo.

| Parâmetro | Arranque | Quando sobe | Sintoma de excesso |
|---|---|---|---|
| `kp_lat` | 0.3 | Resposta fraca / corta curvas | Serpenteia (oscilação rápida) |
| `k_heading` | 0.0 | Corta muito no interior das curvas | Salta no início da curva antes do erro crescer |
| `kd_lat` | 0.0 | Serpenteia mesmo com kp OK | `steer_raw` nervoso / treme em reta |
| `ki_lat` | 0.0 | Deriva constante para um lado | Oscilação lenta / overshoot |
| `i_max` | 0.3 | — | Manter modesto; anti-windup |

> ⚠️ Se vieste do `line_detector` antigo: a escala do `heading_error` mudou (base fixa + declive por-fita) → **re-afina o `k_heading`**, provavelmente para menos, porque o sinal agora é mais estável.

#### 3b. Re-aquisição em curvas apertadas (`k_reacquire`)

Numa curva de 90° a fita interior sai do FOV; o detector publica `visible_side = ±1` e o controller entra em **REACQUIRE**: aplica um viés de esterço fixo `steer = k_reacquire * visible_side` p/ o lado da fita em falta, virando a curva e trazendo a fita perdida de volta ao FOV.

**Passo 1 — Sinal (bancada, rodas no ar):**
1. AUTO, `navigation_enabled: true`, `v_base: 0.0`.
2. Tapa a fita **esquerda** com a mão → overlay "SEE R", `visible_side = +1`.
3. O esterço deve apontar **para o lado da fita tapada (esquerda)** — a curva está desse lado.
4. Se apontar ao contrário, **troca o sinal de `k_reacquire`**. (Tipicamente é o oposto do `kp_lat`.)

**Passo 2 — Magnitude (pista, baixa velocidade):**
- Sobe `|k_reacquire|` até a curva de 90° fechar sem o carro sair pela tangente.
- Alto demais → vira cedo/curto demais e perde a fita exterior também.
- Pode precisar de chegar perto do `steer_max` (~0.44) para curvas mesmo fechadas.

> No plot, o `/reactive_controller/mode` mostra `REACQ_L`/`REACQ_R` durante a manobra; quando recupera as duas fitas volta a `VISION`. Se ficar a oscilar entre REACQUIRE e BLIND (pisca "NO LANE"), sobe o `track_max_miss` no `line_detector`.

#### 4. Desvio do LiDAR (AVOID — só contorna painel, não navega)

| Parâmetro | Efeito | Como afinar |
|---|---|---|
| `wall_detect_range` | Painel mais perto que isto (m) ativa o desvio | Sobe para reagir mais cedo; desce se desvia de coisas longe demais |
| `avoid_fov_deg` | Setor frontal considerado p/ o desvio | Largo vê painéis oblíquos; estreito foca só à frente |
| `k_avoid` | Ganho do desvio (rad no máx. de proximidade) | Sobe se não desvia o suficiente; alto demais → guinada |
| `dodge_max_frac` | Teto do desvio como fração do `steer_max` | Limita a guinada máxima do desvio |
| `vision_suppress` | Quanto a proximidade reduz a autoridade da visão [0..1] | Sobe se a visão "luta" contra o desvio perto do painel |
| `center_threat_deg` | Painel ~à frente: decide o lado pela metade mais aberta | Janela angular do "está mesmo à frente" |

#### 5. Rampa (detetada pelo pitch do IMU)

| Parâmetro | Efeito | Como afinar |
|---|---|---|
| `ramp_pitch_deg` | Pitch acima disto → modo RAMP | Acima do ruído de pitch em piso plano, abaixo do pitch real da rampa |
| `ramp_speed` | m/s para subir com momento | Sobe se o robô empanca a meio da subida |
| `ramp_steer_scale` | Reduz a autoridade do esterço na rampa | Baixa se guina demais na subida |

#### 6. Heading-hold (cego / rampa)

| Parâmetro | Efeito | Como afinar |
|---|---|---|
| `kp_yaw` | Ganho para segurar o yaw do EKF sem fita | Sobe para hold mais firme; alto demais → oscila à volta do heading |

> `control_hz` (frequência do laço) é lido **só no arranque** — alterá-lo em runtime não tem efeito; precisa de restart.

### Como saber se está bom

1. **Segue centrado:** `lateral_error` oscila perto de 0, sem deriva sustentada para um lado.
2. **Esterço suave:** sem serpentear (oscilação rápida) nem trepidação no `angular.z`.
3. **Curvas limpas:** entra e sai sem cortar nem sair da pista.
4. **Para em obstáculo:** abaixo de `hard_stop_range` o `linear.x` vai a 0.
5. **Desvia o painel** sem perder a pista, e retoma o seguimento depois.
6. **Recupera heading:** sem fita (cego/rampa) mantém a direção e retoma quando a fita volta.
7. **Sobe a rampa:** deteta o pitch, mantém momento, não guina.

**Validação:** plota o `lateral_error` durante uma volta completa — deve manter-se pequeno e suave do início ao fim.

---

## Regressão Polinomial de Direção (`hardware.yaml`)

O servo mapeia ângulo desejado (°) para pulso (ms) via: `pulse = a0 + a1·δ + a2·δ²`

### Procedimento de calibração

1. Lança o `servo_calibrator_node` (pacote `speedy_calibration`)
2. Em modo MANUAL, conduz o robô em linha reta a ~0.2 m/s
3. Para cada ângulo comandado, prime o botão de registo — o nó reporta `measured_angle_deg` via `/servo_calibration/measured_angle_deg`
4. Recolhe medições para **ambos os sentidos** (direita e esquerda) para neutralizar a folga mecânica (~3°)
5. Calcula a média esquerda/direita por ângulo e ajusta uma regressão quadrática (R² > 0.99 é bom)
6. Atualiza `steering_poly_a0`, `steering_poly_a1`, `steering_poly_a2` em `hardware.yaml`

> **Nota:** A folga mecânica (~3°) é a diferença entre medições direita e esquerda. Não é corrigível em software com uma única curva — para compensar por direção usa dois conjuntos de coeficientes.

### Valores de referência atuais

```
a0 = 1.45304200  # centro real do servo (corrige bias de ~2° à esquerda)
a1 = 0.02523100  # ganho linear
a2 = 0.00016789  # correção de saturação nos extremos
```

---

## IMU (`imu.yaml` / `ekf.yaml`)

### Calibração de bias do gyro

O driver calibra automaticamente no arranque (parâmetro `calibrate_samples: 200`, ~400 ms).  
**O robô deve estar completamente imóvel durante o boot** para que o bias seja calculado corretamente.

### Verificação em Foxglove

| Tópico | O que verificar |
|---|---|
| `/imu/mpu6050` → `angular_velocity.z` | Deve ser ~0 rad/s quando parado (após boot imóvel) |
| `/imu/mpu6050` → `linear_acceleration.z` | Deve ser ~9.81 m/s² em superfície plana |
| `/odometry/filtered` → `pose.orientation` | Yaw deve estabilizar após 15–40 s (convergência do DMP) |
