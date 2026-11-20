#include <Arduino.h>
struct capture { volatile int Rising,Falling,LastFalling,LastRising; volatile int T_alto,T_baixo,Delta,Freq; volatile bool Flag,IRQ_Disabled; };
extern struct capture Roll,Pitch,Yaw,Throttle,Ch6;
inline void startTimer_TC0(Tc*,uint32_t,IRQn_Type){} inline void startTimer_TC6(Tc*,uint32_t,IRQn_Type){}
inline void startTimer_TC7(Tc*,uint32_t,IRQn_Type){} inline void startTimer_TC8(Tc*,uint32_t,IRQn_Type){}
