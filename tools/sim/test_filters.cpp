// Resposta em frequencia dos filtros do giroscopio (GyroFilter.h), a 400 Hz.
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include "GyroFilter.h"
const double FS=400, PI=3.14159265358979;
template<class F> void resp(F f, double hz, double &gain_db, double &delay_ms){
  double a=1000, sre=0, sim=0, xre=0, xim=0; int n0=2000, n=8000;
  for(int i=0;i<n0+n;i++){ double t=i/FS, x=a*sin(2*PI*hz*t); double y=f(x);
    if(i>=n0){ sre+=y*sin(2*PI*hz*t); sim+=y*cos(2*PI*hz*t); } }
  double amp=2*sqrt(sre*sre+sim*sim)/n; double ph=atan2(sim,sre); // fase relativa ao seno
  gain_db=20*log10(amp/a); delay_ms=-ph/(2*PI*hz)*1000;
}
template<class F> void row(const char*name, F make){
  printf("%-34s", name);
  double hz[]={10,30,67,100,133}; for(double h:hz){ auto f=make(); double g,d; resp(f,h,g,d); if(h==10) printf(" atraso@10Hz=%5.2f ms |", d); printf(" %4.0fHz:%6.1f dB", h, g);} printf("\n");
}
int main(){
  row("media movel 6 (padrao, original)", []{ auto m=new MovingAverage(); return [m](double x){ return (double)m->apply((int)lround(x)); }; });
  for(double fc: {30.0, 70.0}){
    char n1[64], n2[64]; snprintf(n1,64,"PT1 %.0f Hz",fc); snprintf(n2,64,"biquad Butterworth %.0f Hz",fc);
    row(n1,[fc]{ auto p=new PT1(); p->set(fc,FS); return [p](double x){ return (double)p->apply(x); }; });
    row(n2,[fc]{ auto b=new Biquad(); b->setLowPass(fc,FS); return [b](double x){ return (double)b->apply(x); }; });
  }
  row("biquad 70 Hz + notch 133 Hz Q=2",[]{ auto b=new Biquad(); auto nt=new Biquad(); b->setLowPass(70,FS); nt->setNotch(133,FS,2); return [b,nt](double x){ return (double)b->apply(nt->apply(x)); }; });
  // media movel identica ao codigo original (soma/6 inteira)
  MovingAverage m; int buf[6]={0}, sum=0, idx=0, bad=0; srand(1);
  for(int i=0;i<100000;i++){ int x=rand()%4001-2000; sum+=-buf[idx]+x; buf[idx]=x; idx=(idx==5)?0:idx+1; if(m.apply(x)!=sum/6) bad++; }
  printf("%s media movel identica a original em 100000 amostras aleatorias\n", bad?"FALHA":"OK  ");
  return bad;
}
