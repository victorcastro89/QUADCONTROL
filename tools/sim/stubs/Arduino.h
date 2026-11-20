#pragma once
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cstring>
using std::fabs;
typedef bool boolean; typedef uint8_t byte;
extern unsigned long fake_us;
inline unsigned long micros(){return fake_us;}
inline unsigned long millis(){return fake_us/1000;}
inline void delay(unsigned long ms){fake_us+=ms*1000;}
template<class T,class A,class B> T constrain(T x,A a,B b){return x<a?a:(x>b?b:x);}
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define RISING 1
#define FALLING 2
extern int led_state;
inline void pinMode(int,int){} inline void digitalWrite(int,int v){led_state=v;}
inline void attachInterrupt(int, void(*)(), int){}
struct FakeSerial{ bool quiet=true; template<class T> void print(T){} template<class T> void println(T v){ if(!quiet) log(v);} void println(){}
  void log(const char*s){ printf("    [serial] %s\n",s);} template<class T> void log(T){} void begin(long){} };
extern FakeSerial Serial;
struct TcCh{ uint32_t TC_IER, TC_IDR; }; struct Tc{ TcCh TC_CHANNEL[3]; }; typedef int IRQn_Type; extern Tc* TC0; extern Tc* TC2;
#define TC6_IRQn 0
#define TC7_IRQn 0
#define TC8_IRQn 0
#define TC0_IRQn 0
#define TC_CMR_WAVE 0
#define TC_CMR_WAVSEL_UP_RC 0
#define TC_CMR_TCCLKS_TIMER_CLOCK4 0
#define TC_IER_CPCS 0
#define VARIANT_MCK 84000000
#define ARDUINO 100

inline void pmc_set_writeprotect(bool){} inline void pmc_enable_periph_clk(uint32_t){}
inline void TC_Configure(Tc*,uint32_t,uint32_t){} inline void TC_SetRA(Tc*,uint32_t,uint32_t){} inline void TC_SetRC(Tc*,uint32_t,uint32_t){} inline void TC_Start(Tc*,uint32_t){}
inline void NVIC_EnableIRQ(int){}
