# Contexto para continuar o trabalho (sessões Claude)

Branch de trabalho: `claude/admiring-feynman-xvonec`
Hardware: Arduino DUE + placa GY-80 (ADXL345, L3G4200D, HMC5883L, BMP085), quadricóptero em X.
Código: `QuadController/` (sketch Arduino; `.ino` são concatenados pelo IDE).

> Nada foi testado em voo nem compilado para o DUE (a instalação do
> `arduino-cli` foi bloqueada na sessão). O PID e os drivers GY80 foram
> testados só no PC, com stubs.
> **Testar em bancada, sem hélices, antes de voar.**

---

## 1. Como o controle funciona (entendimento do código)

Não existe controle de posição: não há GPS, o barômetro está comentado em
`read_sensors()` e nada estima x/y. `Roll_Position`, `Pitch_Position` e
`Yaw_Position` são só a **deflexão do stick** já convertida.

O modo é escolhido pelo canal 6 (chave de 3 posições, `QuadController.ino`, bloco `if(Ch6.T_alto<...)`):

| Ch6 (µs) | `anglemode` | `hold_yaw` | Modo |
|---|---|---|---|
| < 1400 | 1 | 0 | **Angle / Stabilize** (usa acelerômetro) |
| 1400–1700 | 0 | 0 | **Rate / Acro** (só giroscópio) |
| 1700–2000 | 1 | 1 | **Angle + Headless** |

- **Rate/Acro**: o stick comanda velocidade angular (×19 → ~±200 °/s em roll/pitch, ×31 → ~±330 °/s em yaw). Um PID de taxa por eixo, entrada `gyro[]`. Solto o stick, o drone fica no ângulo em que estava. O acelerômetro não entra no controle.
- **Angle/Stabilize**: PID em cascata. A malha de ângulo (`Roll_PID`, `Pitch_PID`, `Yaw_PID`, saída ±20000 LSB de taxa) gera o setpoint da malha de taxa (`RollRate_PID`..., saída ±150 µs). O stick vira ângulo (`/15000` rad → máx ≈ ±45°). Solto o stick, o drone volta a nivelar. Com o stick mantido, ele fica inclinado e acelera (deriva no plano).
- **Headless**: igual ao Angle, mas grava a proa ao entrar no modo e gira os comandos de pitch/roll pela diferença (proa inicial − proa atual).
- **Estimação de atitude** (`Sensors.ino`, `Read_Gyro`): filtro complementar, giroscópio integrado com peso 0,9995 e ângulo do acelerômetro com peso 0,0005, a 400 Hz (τ ≈ 5 s). No yaw, o giroscópio é fundido com a bússola (peso 0,01).
- **Mixer em X**: `M = Throttle ± Roll ± Pitch ± Yaw`. Com throttle < 1100, os motores vão a 1000 µs e todos os PIDs são resetados.
- **Escalas**: giroscópio a 500 dps → 17,5 mdps/LSB (constante `0.00030543` rad/LSB). ESC a 400 Hz (`Servo.h` `REFRESH_INTERVAL 2500`).
- **PID** (`PID_v1.cpp`): é a PID do Brett Beauregard modificada. Usa tempo em µs, derivada filtrada no estilo Simulink (N = 284, Ts = 2,5 ms fixo) e por isso `kd = Kd/Ts`, o que explica o `*0.0025` nos ganhos.

---

## 2. O que já foi alterado nesta branch

Commit "Corrige cascata, banda morta do PID e yaw do modo angulo":

1. **Cascata atualizada a cada ciclo** (`QuadController.ino`): `RollRate/PitchRate/YawRate_Setpoint` recebem a saída das malhas de ângulo logo após o `Compute()` delas, a cada ciclo. Antes isso estava dentro do bloco que só roda quando chega um frame do rádio (~50 Hz).
2. **Banda morta do PID** (`PID_v1.cpp`, `DCM.ino`):
   - A condição antiga, `error>error_min || error<error_min`, era sempre verdadeira.
   - Agora `fabs(error) > error_min` **só congela o integrador**; P e D sempre atuam e a saída sempre é atualizada.
   - Tolerâncias reduzidas ao nível de ruído (taxa: 30 LSB ≈ 0,5 °/s; ângulo: 0,002 rad ≈ 0,1°). Os valores antigos (300/500 LSB, 0,02 rad) nunca tiveram efeito, e os ganhos foram ajustados em voo sem banda morta.
   - `Compute()` agora sempre retorna um valor.
   - O construtor inicializa `error_min`, `ITerm` etc.
   - `Reset()` também zera `Filter_DSTATE` e `FilterCoefficient`.
3. **Yaw no modo Angle vira "taxa + segurar proa"** (`QuadController.ino`):
   - Stick fora do centro → `YawRate_Setpoint = Yaw_Position` (mesma escala do Rate).
   - Stick solto e ainda girando acima de `YAW_SETTLE_RATE` (570 LSB ≈ 10 °/s) → taxa 0 (freia); a proa desejada acompanha a atual.
   - Depois de assentar → `Yaw_PID` segura a proa travada.
   - Antes, o stick de yaw comandava uma proa absoluta (`Yaw_Position/4060`).
   - A proa é recapturada na **troca de modo** (detecção via `last_mode`, que também reseta os PIDs de ângulo) e com **throttle baixo** (no chão).
   - Helper novo: `Current_Yaw_Input()`.
4. **Fusão giro/bússola do yaw** (`Sensors.ino`): `gyro_angle[2] += 0.01*wrap(MAG_Heading - gyro_angle[2])`, com a diferença ajustada para [−π, π]. Antes, a média direta fazia a proa estimada pular perto de ±180°.
5. Removidos bytes NUL corrompidos no fim de `QuadController.ino` e `DCM.h`.
6. **Drivers próprios `GY80.h/.cpp`** substituem as bibliotecas modificadas (ver seção 4).
   Também foram removidos `#include "I2Cdev.h"` e `<PWM.h>`, que não eram usados.
7. **Segurança: armar/desarmar, failsafe e limites dos motores** (`Safety.h`, `Safety.ino`, `Mix_Motors()` em `Output.ino`). Todas as constantes estão em `Safety.h`.
   - Estados: `DESARMADO` → `ARMADO` → `FAILSAFE`. O drone liga desarmado (motores em 1000 µs).
   - **Armar:** throttle < 1100 + yaw > 1900 µs por 1 s, com rádio válido e inclinação < 25°.
   - **Desarmar:** throttle < 1100 + yaw < 1100 µs por 1 s, ou 15 s armado com throttle baixo, ou fim do failsafe.
   - **Rádio:** o frame só vale com os 4 pulsos entre 900 e 2100 µs. Os pulsos são copiados para variáveis locais e o código não escreve mais em `Throttle.T_alto`. Sem frame válido por 250 ms → `FAILSAFE`.
   - **Failsafe:** modo Angle nivelado, segurando a proa, sticks e Ch6 ignorados. O throttle desce em rampa de 50 µs/s a partir do último valor; ao chegar a 1100 µs, ou após 10 s, desarma. Se o rádio voltar, termina a descida mesmo assim.
   - **Motores:** desarmado = 1000; armado no chão = 1080 (hélices girando devagar); em voo, cada motor fica em [1100, 2000]. Se as correções não cabem na faixa, são reduzidas proporcionalmente; depois o throttle é deslocado para todos caberem. Anti-windup: com saturação, os integradores das malhas de taxa param (`PID_::HoldIntegrator`).
   - **LED 13:** pisca devagar = desarmado; aceso = armado; pisca rápido = failsafe.
   - O mapeamento dos sticks foi simplificado para `(pulso − 1484) × ganho`, com os mesmos sinais e escalas de antes (pulso limitado a 1000–2000 µs).
   - **Simulação no PC:** `sh tools/sim/run.sh` compila o sketch inteiro com stubs de hardware e roda 19 cenários (armar, voo, saturação, perda de rádio, rádio voltando no failsafe, pulso inválido, desarmar, auto-desarme, armar inclinado). Todos passam.

### Pendências de configuração (precisam do hardware)
- **Throttle de hover**: ainda não medido. Quando souber, ponha em `FS_HOVER_THROTTLE` (`Safety.h`). A descida do failsafe passa a partir de hover − 50 µs em vez do último throttle.
- **Failsafe do receptor**: configurar o receptor para, ao perder sinal, **parar de mandar pulsos** ou mandar throttle < 900 µs. Se ele repetir o último pulso, o firmware nunca detecta a perda.
- **Faixa dos ESCs**: o código assume calibração em 1000–2000 µs (`MOTOR_MAX`). Confirmar, e verificar se 1080 µs (`MOTOR_IDLE`) gira os motores devagar e 1100 µs (`MOTOR_MIN`) não para nenhum deles.
- **Sentido do gesto**: se no seu rádio "yaw para a direita" der pulso baixo, troque `STICK_HIGH`/`STICK_LOW` na lógica de `Update_Safety()`.

---

## 3. Achados ainda NÃO corrigidos (por prioridade)

### Segurança
1–3. ~~Failsafe, armar/desarmar e limites dos motores~~ → **feitos** (seção 2, item 7). Restam as pendências de configuração listadas lá.

### Qualidade do controle
4. **Média móvel de 6 amostras no giroscópio**: ~6 ms de atraso de fase na malha de taxa, o maior limitante de ganho. Trocar por um passa-baixa de 1ª ordem ou biquad (~80 Hz).
5. **Passa-alta do L3G** (`Gyr.setHighPassOnOff(1)`): com OUT_Sel = 00 ele **não entra no dado lido**, então provavelmente não tem efeito (ver seção 4). Pode ser removido para ficar explícito.
6. **Laço sem sincronia**: sensores, PIDs e motores rodam com timers independentes, com jitter de até 2,5 ms. O ideal é um tick fixo de 400 Hz: ler → filtrar → PIDs → motores. Usar I²C a 400 kHz (`Wire.setClock(400000)`), porque a leitura bloqueante da bússola rouba tempo do laço.
7. **Windup no chão**: o I da malha de ângulo (limitado a ±20000) acumula com throttle > 1100 antes de decolar. Limitar esse I bem menos e só integrar acima de um throttle de decolagem.
8. **Derivada sobre o erro** (pico quando o setpoint muda). Hoje o D é muito pequeno; se aumentar, trocar para derivada sobre a medida.

### Estimação
9. **A bússola não compensa a inclinação de verdade**: `magnetom[2]` nunca é preenchido (fica 0). O peso 0,01 a 400 Hz (τ ≈ 0,25 s) confia demais na bússola, que sofre interferência dos motores. Algo como 0,001 seria mais seguro.
10. **Euler integrado direto das taxas do corpo**: ao girar no yaw inclinado, a inclinação migra entre roll e pitch. Quaternion ou DCM resolvem (`DCM.h`/`Math.ino` existem, sem uso).

### Menores
- ~~`Throttle.T_alto=1930` escrito no loop~~, ~~includes sem uso~~ e ~~stick fora da faixa mantendo o valor anterior~~ → resolvidos.
- 7 (windup no chão) ficou **parcialmente** resolvido: armado com throttle < 1100, os PIDs ficam zerados. Mas com throttle acima de 1100 e o drone ainda no chão, o I da malha de ângulo ainda acumula.

---

## 4. Bibliotecas modificadas → substituídas por `GY80.h/.cpp`

O código original usava cópias **modificadas** de `Adafruit_ADXL345_U` e `L3G` (Pololu), que nunca foram versionadas. Elas foram substituídas por drivers próprios no sketch (`QuadController/GY80.h`, `GY80.cpp`), com **os mesmos nomes de métodos**, então `Sensors.ino` quase não mudou (`Gyr` é `GY80_Gyro`, `accel` é `GY80_Accel`; as constantes viraram `GY80_ADXL_RANGE_16_G` e `GY80_ADXL_DATARATE_400_HZ`).

Bibliotecas externas necessárias agora (todas oficiais, sem modificação):
**Adafruit Unified Sensor** e **Adafruit BMP085 Unified** (+ core `arduino:sam` para o DUE).

Registradores escritos (conferidos com teste de I²C simulado no PC):

| Chamada | Registrador | Valor | Observação |
|---|---|---|---|
| `accel.begin()` | POWER_CTL 0x2D | 0x08 | checa DEVID = 0xE5 |
| `setRange(16G)` | DATA_FORMAT 0x31 | 0x0B | FULL_RES → 1 g = 256 contagens |
| `setDataRate(400)` | BW_RATE 0x2C | 0x0C | |
| `Xofset/Yofset/Zofset` | 0x1E/0x1F/0x20 | int8 | 15,6 mg/LSB = 4 contagens (daí o `/4`) |
| `getRaw()` | DATAX0.. 0x32 | 6 bytes | contagens brutas, não m/s² |
| `Gyr.init()` | WHO_AM_I 0x0F | 0xD3 | procura em 0x69 (GY-80) e 0x68 |
| `enableDefault()` + `setOutputDataRate(400)` | CTRL_REG1 0x20 | **0xAF** | 400 Hz, BW = 10 (LPF1 ≈ 50 Hz) |
| `setFilterMode(0b11)` | CTRL_REG2 0x21 | 0x30 | HPM = 11, HPCF padrão |
| `scale(500)` | CTRL_REG4 0x23 | 0x10 | 17,5 mdps/LSB |
| `setHighPassOnOff(1)` | CTRL_REG5 0x24 | 0x10 | HPen = 1, OUT_Sel = 00 → HPF fora do dado |

**Suposições** (o que as bibliotecas originais faziam exatamente não é conhecido):
- **BW do L3G**: segui o `enableDefault()` da Pololu mais recente (0x6F → BW = 10, ~50 Hz a 400 Hz ODR). Versões antigas usavam 0x0F (BW = 00, ~20 Hz). Se o drone ficar mais "nervoso" que antes, testar BW = 00 (CTRL_REG1 = 0x8F).
- **`setFilterMode`** foi interpretado como HPM (CTRL_REG2). Outra leitura possível seria OUT_Sel (CTRL_REG5). Isso colocaria um passa-alta de 30 Hz no dado, e com isso o drone não teria voado, por isso essa leitura foi descartada.
- Se as bibliotecas originais forem encontradas, comparar com a tabela acima. Procurar por `Xofset`/`getRaw` (ADXL) e `setHighPassOnOff`/`setOutputDataRate` (L3G).

**Checagem em bancada sugerida:** com o drone parado, `Print_Acc()` deve dar Z ≈ 256 e X, Y ≈ 0; `Print_Gyr()` deve dar ≈ 0. Girando a 90°/s, o giroscópio deve ler ≈ 5140 LSB (90/0,0175).

## 5. Próximos passos sugeridos

1. Compilar com `arduino-cli` (core `arduino:sam` + as 2 bibliotecas Adafruit) e fazer a checagem em bancada da seção 4.
2. Bancada **sem hélices**: testar armar/desarmar, LED, failsafe (desligar o rádio) e as pendências de configuração da seção 2.
3. Itens 4 e 6 (filtro do giroscópio e laço síncrono a 400 Hz), provavelmente o maior ganho de estabilidade.
4. Revisar 5, 7 e 9; depois considerar 10.
