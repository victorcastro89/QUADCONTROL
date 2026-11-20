/*
 * Maquina de estados de seguranca (constantes e explicacao em Safety.h):
 *
 *   DESARMADO --(throttle baixo + yaw > STICK_HIGH por 1 s)--> ARMADO
 *   ARMADO    --(throttle baixo + yaw < STICK_LOW por 1 s)---> DESARMADO
 *   ARMADO    --(throttle baixo por AUTO_DISARM_MS)----------> DESARMADO
 *   ARMADO    --(sem frame valido por RC_TIMEOUT_MS)---------> FAILSAFE
 *   FAILSAFE  --(throttle chegou a MOTOR_MIN ou FS_MAX_MS)---> DESARMADO
 *
 * Em FAILSAFE o drone fica no modo Angle, nivelado, segurando a proa, com o
 * throttle descendo em rampa. Mesmo se o radio voltar, ele termina a descida e
 * desarma; para voar de novo e preciso armar com o gesto.
 */

FlightState flight_state = STATE_DISARMED;
unsigned long rc_last_frame = 0;
bool rc_ever_valid = false;
int rc_throttle = MOTOR_OFF;
int rc_yaw_raw = RC_CENTER;
double fs_throttle = MOTOR_OFF;
bool motors_saturated = false;

static bool gesture_active = false;
static unsigned long gesture_start = 0;
static bool low_throttle_active = false;
static unsigned long low_throttle_start = 0;
static unsigned long fs_start = 0;
static unsigned long fs_last_update = 0;

bool RC_Pulse_Valid(int us){
  return us >= RC_PULSE_MIN && us <= RC_PULSE_MAX;
}

static bool RC_Ok(unsigned long now){
  return rc_ever_valid && (now - rc_last_frame) < RC_TIMEOUT_MS;
}

// Zera todos os PIDs e trava a proa atual (usado no chao e ao armar/desarmar).
void Reset_Controllers(){
  RollRate_PID.Reset();
  PitchRate_PID.Reset();
  YawRate_PID.Reset();
  Roll_PID.Reset();
  Pitch_PID.Reset();
  Yaw_PID.Reset();
  Yaw_Setpoint = Yaw_Input;
}

// Conta quanto tempo uma condicao (gesto ou throttle baixo) se mantem.
static bool Held_For(bool cond, bool &active, unsigned long &start,
                     unsigned long now, unsigned long ms){
  if(!cond){ active = false; return false; }
  if(!active){ active = true; start = now; }
  return (now - start) >= ms;
}

static void Arm(){
  Reset_Controllers();
  flight_state = STATE_ARMED;
  gesture_active = false;
  low_throttle_active = false;
  Serial.println("ARMADO");
}

static void Disarm(){
  Reset_Controllers();
  flight_state = STATE_DISARMED;
  gesture_active = false;
  low_throttle_active = false;
  Serial.println("DESARMADO");
}

static void Enter_Failsafe(unsigned long now){
  flight_state = STATE_FAILSAFE;
  fs_start = now;
  fs_last_update = now;
  fs_throttle = rc_throttle;
  if(FS_HOVER_THROTTLE > 0 && fs_throttle > FS_HOVER_THROTTLE - FS_HOVER_MARGIN)
    fs_throttle = FS_HOVER_THROTTLE - FS_HOVER_MARGIN;
  if(fs_throttle > THROTTLE_MAX_IN) fs_throttle = THROTTLE_MAX_IN;

  // Forca o modo Angle sem headless, com troca de modo "sem tranco".
  anglemode = 1;
  hold_yaw = 0;
  last_mode = 1;
  Roll_PID.Reset();
  Pitch_PID.Reset();
  Yaw_PID.Reset();
  Yaw_Setpoint = Yaw_Input;
  Serial.println("FAILSAFE");
}

void Update_Safety(){
  unsigned long now = millis();
  bool throttle_low = rc_throttle < THROTTLE_LOW;

  switch(flight_state){
    case STATE_DISARMED: {
      bool level = fabs(Roll_Input) < MAX_ARM_TILT && fabs(Pitch_Input) < MAX_ARM_TILT;
      bool arm_gesture = RC_Ok(now) && throttle_low && rc_yaw_raw > STICK_HIGH && level;
      if(Held_For(arm_gesture, gesture_active, gesture_start, now, ARM_HOLD_MS))
        Arm();
      break;
    }

    case STATE_ARMED:
      if(!RC_Ok(now)){
        Enter_Failsafe(now);
        break;
      }
      if(Held_For(throttle_low && rc_yaw_raw < STICK_LOW,
                  gesture_active, gesture_start, now, ARM_HOLD_MS)){
        Disarm();
        break;
      }
      if(Held_For(throttle_low, low_throttle_active, low_throttle_start,
                  now, AUTO_DISARM_MS))
        Disarm();
      break;

    case STATE_FAILSAFE: {
      // Nivelado, segurando a proa, sticks ignorados.
      anglemode = 1;
      hold_yaw = 0;
      Roll_Setpoint = 0;
      Pitch_Setpoint = 0;
      Yaw_Position = 0;
      yaw_stick_active = false;

      fs_throttle -= FS_RAMP_US_PER_S * (now - fs_last_update) / 1000.0;
      fs_last_update = now;
      if(fs_throttle <= MOTOR_MIN || (now - fs_start) >= FS_MAX_MS)
        Disarm();
      break;
    }
  }
}

// LED 13: pisca devagar = desarmado; aceso = armado; pisca rapido = failsafe.
void Update_LED(){
  static unsigned long last_toggle = 0;
  static bool led_on = false;
  unsigned long now = millis();

  if(flight_state == STATE_ARMED){
    led_on = true;
  }
  else{
    unsigned long period = (flight_state == STATE_FAILSAFE) ? 100 : 500;
    if(now - last_toggle >= period){
      led_on = !led_on;
      last_toggle = now;
    }
  }
  digitalWrite(STATUS_LED_PIN, led_on ? HIGH : LOW);
}
