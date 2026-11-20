/*
 * Seguranca: armar/desarmar, failsafe de radio e limites dos motores.
 * Implementacao em Safety.ino (estados) e Output.ino (Mix_Motors).
 * Todos os tempos em ms e pulsos em us.
 */
#ifndef SAFETY_H
#define SAFETY_H

/* ---------- Radio ---------- */
#define RC_PULSE_MIN        900    // pulso abaixo disso = frame invalido
#define RC_PULSE_MAX        2100   // pulso acima disso  = frame invalido
#define RC_TIMEOUT_MS       250    // sem frame valido por esse tempo -> failsafe
#define RC_CENTER           1484   // centro dos sticks (medido no radio)

/* ---------- Gestos (throttle baixo + yaw no extremo) ---------- */
#define THROTTLE_LOW        1100   // abaixo disso: "throttle baixo" / no chao
#define STICK_HIGH          1900   // yaw todo para um lado  -> armar
#define STICK_LOW           1100   // yaw todo para o outro  -> desarmar
#define ARM_HOLD_MS         1000   // tempo segurando o gesto
#define AUTO_DISARM_MS      15000  // armado com throttle baixo por esse tempo -> desarma
#define MAX_ARM_TILT        0.436  // rad (~25 graus): nao arma inclinado

/* ---------- Failsafe ---------- */
// PENDENTE: throttle de hover ainda nao medido. Com 0 a descida parte do ultimo
// throttle valido. Quando souber o valor (us), coloque aqui: a descida passa a
// partir de (hover - FS_HOVER_MARGIN), o que e mais previsivel.
#define FS_HOVER_THROTTLE   0
#define FS_HOVER_MARGIN     50
#define FS_RAMP_US_PER_S    50.0   // velocidade da rampa de descida do throttle
#define FS_MAX_MS           10000  // tempo maximo em failsafe antes de desarmar

/* ---------- Motores (faixa calibrada dos ESCs: 1000-2000 us) ---------- */
#define MOTOR_OFF           1000   // desarmado
#define MOTOR_IDLE          1080   // armado no chao: helices girando devagar
#define MOTOR_MIN           1100   // minimo em voo (nenhum motor para)
#define MOTOR_MAX           2000   // maximo do ESC
#define THROTTLE_MAX_IN     1930   // limite do stick de throttle (folga p/ correcoes)

enum FlightState { STATE_DISARMED, STATE_ARMED, STATE_FAILSAFE };

extern FlightState flight_state;
extern unsigned long rc_last_frame;
extern bool rc_ever_valid;
extern int rc_throttle;      // copia local do throttle do ultimo frame valido
extern int rc_yaw_raw;       // pulso de yaw do ultimo frame valido (para os gestos)
extern double fs_throttle;   // throttle usado durante o failsafe
extern bool motors_saturated;

bool RC_Pulse_Valid(int us);
void Update_Safety();
void Update_LED();
void Reset_Controllers();
void Mix_Motors();

#endif
