// Testa o tick fixo do loop(): 400 ticks/s, dt medido e realinhamento apos atraso.
#include "build/sketch.cpp"
unsigned long fake_us=0; int led_state=0; FakeSerial Serial; FakeWire Wire; Tc tc0,tc2; Tc*TC0=&tc0; Tc*TC2=&tc2;
int fails=0;
void check(bool ok,const char*msg){ printf("%s %s\n", ok?"OK  ":"FALHA", msg); if(!ok) fails++; }
int main(){
  Wire.regs[0x53][0x00]=0xE5; Wire.regs[0x69][0x0F]=0xD3;
  setup();
  // aquecimento (o 1o tick sai alguns us depois da agenda; o 2o compensa)
  for(int i=0;i<100;i++){ fake_us+=100; loop(); }
  // 1 s de loop() chamado a cada 100 us, janela [t0, t0+1 s)
  unsigned long ticks=0, prev=last_tick_start, t0=fake_us;
  long dtmin=1<<30, dtmax=0;
  while(true){ fake_us+=100; if(fake_us - t0 >= 1000000) break; loop();
    if(last_tick_start!=prev){ ticks++; prev=last_tick_start; if(tick_dt<dtmin)dtmin=tick_dt; if(tick_dt>dtmax)dtmax=tick_dt; } }
  printf("     ticks em 1 s = %lu, dt min/max = %ld/%ld us, estouros = %lu\n", ticks, dtmin, dtmax, tick_overruns);
  check(ticks>=399 && ticks<=400, "400 ticks por segundo (+-1 pela fase da janela)");
  check(dtmin==2500 && dtmax==2500, "dt de 2500 us em todos os ticks");
  check(tick_overruns==0, "sem estouros com laco rapido");
  // um travamento de 6 ms (ex.: I2C lento): conta um estouro e realinha
  fake_us+=6000; loop(); unsigned long after=last_tick_start;
  check(tick_overruns==1, "travamento de 6 ms conta 1 estouro");
  check(tick_dt>=6000, "dt medido reflete o atraso (integracao do angulo correta)");
  ticks=0; prev=after; t0=fake_us;
  while(true){ fake_us+=100; if(fake_us - t0 >= 100000) break; loop(); if(last_tick_start!=prev){ticks++; prev=last_tick_start;} }
  printf("     ticks em 100 ms depois do atraso = %lu\n", ticks);
  check(ticks>=39 && ticks<=40, "depois do atraso volta a 400 Hz sem rajada de ticks");
  return fails;
}
