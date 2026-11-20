#include "build/sketch.cpp"
unsigned long fake_us=0; int led_state=0; FakeSerial Serial; FakeWire Wire; Tc tc0,tc2; Tc*TC0=&tc0; Tc*TC2=&tc2;
int rc_thr=1000, rc_r=1484, rc_p=1484, rc_y=1484; bool radio=true; unsigned long next_frame=0;
const char* st(){ return flight_state==STATE_DISARMED?"DESARMADO":flight_state==STATE_ARMED?"ARMADO":"FAILSAFE"; }
void run_ms(int ms){ for(int i=0;i<ms*2;i++){ fake_us+=500;
  if(radio && fake_us>=next_frame){ next_frame=fake_us+20000; Throttle.T_alto=rc_thr; Roll.T_alto=rc_r; Pitch.T_alto=rc_p; Yaw.T_alto=rc_y; Ch6.T_alto=1200;
    Throttle.Flag=Roll.Flag=Pitch.Flag=Yaw.Flag=1; }
  loop(); } }
void show(const char*t){ printf("%-48s estado=%-9s motores=%d %d %d %d led=%d\n",t,st(),Motor1.us,Motor2.us,Motor3.us,Motor4.us,led_state); }
int main(){
  Wire.regs[0x53][0x00]=0xE5; Wire.regs[0x69][0x0F]=0xD3;
  setup(); Serial.quiet=false;
  run_ms(500); show("1. ligado, throttle baixo");
  rc_thr=1500; run_ms(500); show("2. throttle 1500 desarmado (nao pode girar)");
  rc_thr=1000; rc_y=2000; run_ms(600); show("3. gesto de armar por 0,6 s (ainda nao)");
  run_ms(600); show("4. gesto de armar por 1,2 s");
  rc_y=1484; run_ms(200); show("5. armado no chao");
  rc_thr=1500; run_ms(1000); show("6. voando com throttle 1500");
  // saturacao: injeta correcoes grandes direto no mixer
  rc_throttle=1930; RollRate_Output=150; PitchRate_Output=150; YawRate_Output=150; Mix_Motors();
  printf("%-48s motores=%d %d %d %d saturado=%d\n","7. sat.: thr 1930, R=P=Y=+150",M1_out,M2_out,M3_out,M4_out,motors_saturated);
  rc_throttle=1100; RollRate_Output=150; PitchRate_Output=0; YawRate_Output=0; Mix_Motors();
  printf("%-48s motores=%d %d %d %d saturado=%d\n","8. sat.: thr 1100, R=+150",M1_out,M2_out,M3_out,M4_out,motors_saturated);
  rc_throttle=1500; RollRate_Output=150; PitchRate_Output=150; YawRate_Output=-150; Mix_Motors();
  printf("%-48s motores=%d %d %d %d saturado=%d\n","8b. thr 1500, R=P=150 Y=-150 (cabe)",M1_out,M2_out,M3_out,M4_out,motors_saturated);
  rc_thr=1500; run_ms(500);
  radio=false; run_ms(200); show("9. sem radio por 0,2 s (ainda armado)");
  run_ms(200); show("10. sem radio por 0,4 s");
  run_ms(4000); show("11. failsafe ha ~4 s (descendo)");
  radio=true; rc_thr=1800; rc_r=2000; run_ms(1000); show("12. radio voltou, sticks mexidos (ignora)");
  run_ms(4000); show("13. fim da descida");
  rc_thr=1000; rc_r=1484; run_ms(500); show("14. depois do failsafe");
  rc_y=2000; run_ms(1200); rc_y=1484; run_ms(100); show("15. armado de novo");
  rc_y=1000; run_ms(1200); rc_y=1484; run_ms(100); show("16. gesto de desarmar");
  rc_y=2000; run_ms(1200); rc_y=1484; run_ms(15500); show("17. armado no chao por 15,5 s (auto-desarme)");
  rc_y=2000; run_ms(1200); rc_y=1484; rc_thr=1500; run_ms(500);
  rc_thr=800; run_ms(400); show("18. throttle invalido (800 us) por 0,4 s");
  rc_thr=1000; run_ms(9000); rc_y=2000; gyro_angle[1]=0.6; run_ms(1500); show("19. tentar armar inclinado 34 graus");
}
