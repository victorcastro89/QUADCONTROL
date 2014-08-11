#include <Arduino.h>


extern struct capture {
  volatile int Rising,Falling,LastFalling,LastRising;
  volatile int T_alto,T_baixo,Delta,Freq;
  volatile bool Flag,IRQ_Disabled;
    
}Roll,Pitch,Yaw,Throttle,Ch6;


 void startTimer_TC0(Tc *tc, uint32_t channel, IRQn_Type irq);
 void startTimer_TC6(Tc *tc, uint32_t channel, IRQn_Type irq);
 void startTimer_TC7(Tc *tc, uint32_t channel, IRQn_Type irq);
 void startTimer_TC8(Tc *tc, uint32_t channel, IRQn_Type irq);



