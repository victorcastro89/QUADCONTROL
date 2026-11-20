# Contexto para continuar o trabalho (sessões Claude)

Branch de trabalho: `claude/admiring-feynman-xvonec`
Hardware: Arduino DUE + placa GY-80 (ADXL345, L3G4200D, HMC5883L, BMP085), quadricóptero em X.
Código: `QuadController/` (sketch Arduino; `.ino` são concatenados pelo IDE).

> Nada foi testado em voo nem compilado para o DUE (faltam as bibliotecas
> modificadas, ver seção 4). O PID corrigido foi testado só no PC, com stubs.
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
5. Removidos ~9 KB de bytes NUL corrompidos no fim de `QuadController.ino`.

---

## 3. Achados ainda NÃO corrigidos (por prioridade)

### Segurança
1. **Não há failsafe.** Sem sinal do rádio, `T_alto` mantém o último valor e o drone continua voando com ele. Seria preciso um timeout por canal (ex.: 100 ms sem pulso → descer ou cortar).
2. **Não há armar/desarmar.** Os motores giram assim que o throttle passa de 1100 µs, inclusive ao ligar com o stick fora do mínimo.
3. **A saída dos motores não tem limite mínimo nem tratamento de saturação.** `Throttle − 450` pode ir abaixo da rotação mínima do ESC (motor parando em voo). Seria preciso limitar entre marcha lenta e máximo e deslocar os 4 motores juntos quando saturar.

### Qualidade do controle
4. **Média móvel de 6 amostras no giroscópio**: ~6 ms de atraso de fase na malha de taxa, o maior limitante de ganho. Trocar por um passa-baixa de 1ª ordem ou biquad (~80 Hz).
5. **Passa-alta do L3G ligado** (`Gyr.setHighPassOnOff(1)`, `Sensors.ino` `Gyro_Init`): apaga rotações lentas, o que atrapalha a integração do ângulo e o I da malha de taxa. Verificar a frequência de corte ou desligar.
6. **Laço sem sincronia**: sensores, PIDs e motores rodam com timers independentes, com jitter de até 2,5 ms. O ideal é um tick fixo de 400 Hz: ler → filtrar → PIDs → motores. Usar I²C a 400 kHz (`Wire.setClock(400000)`), porque a leitura bloqueante da bússola rouba tempo do laço.
7. **Windup no chão**: o I da malha de ângulo (limitado a ±20000) acumula com throttle > 1100 antes de decolar. Limitar esse I bem menos e só integrar acima de um throttle de decolagem.
8. **Derivada sobre o erro** (pico quando o setpoint muda). Hoje o D é muito pequeno; se aumentar, trocar para derivada sobre a medida.

### Estimação
9. **A bússola não compensa a inclinação de verdade**: `magnetom[2]` nunca é preenchido (fica 0). O peso 0,01 a 400 Hz (τ ≈ 0,25 s) confia demais na bússola, que sofre interferência dos motores. Algo como 0,001 seria mais seguro.
10. **Euler integrado direto das taxas do corpo**: ao girar no yaw inclinado, a inclinação migra entre roll e pitch. Quaternion ou DCM resolvem (`DCM.h`/`Math.ino` existem, sem uso).

### Menores
- `Throttle.T_alto=1930` escreve numa variável que a interrupção também altera (é melhor usar uma cópia local).
- `#include "I2Cdev.h"` e `<PWM.h>` parecem não ser usados.
- Fora de 1000–2100 µs, o stick mantém o valor anterior em vez de ir para o centro.

---

## 4. Bibliotecas modificadas (fora do repositório)

O código chama funções que **não existem nas bibliotecas oficiais**. O autor editou as cópias em `Arduino/libraries` da máquina dele e elas não foram versionadas, por isso o sketch não compila numa máquina nova.

- **Adafruit_ADXL345_U**: `getRaw()` (contagens brutas em vez de m/s²) e `Xofset/Yofset/Zofset()` (registradores de offset em hardware OFSX/OFSY/OFSZ, 0x1E–0x20, ~15,6 mg/LSB; daí o `/4` em `set_acc_offset`).
- **L3G (Pololu)**: `setOutputDataRate(400)`, `scale(500)`, `setFilterMode()` e `setHighPassOnOff()`. São escritas nos registradores CTRL_REG1/2/4/5 do L3G4200D.
- `PID_v1` e `Servo` também são modificadas, mas **estão** no repositório (PID renomeada para `PID_`; Servo a 400 Hz para ESC).

**Plano sugerido:** recriar essas funções no próprio sketch (escrita direta nos registradores via `Wire`), usar as bibliotecas oficiais sem alteração e remover os includes sem uso.

---

## 5. Próximos passos sugeridos

1. Tornar o projeto compilável (seção 4) e validar com `arduino-cli` (core `arduino:sam`).
2. Itens de segurança 1–3 (failsafe, armar, limites e saturação dos motores).
3. Itens 4 e 6 (filtro do giroscópio e laço síncrono a 400 Hz), provavelmente o maior ganho de estabilidade.
4. Revisar 5, 7 e 9; depois considerar 10.
