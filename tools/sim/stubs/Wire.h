#pragma once
#include <cstdint>
struct FakeWire {
  uint8_t regs[128][64]={}; uint8_t addr=0, ptr=0; int n=0; bool first=true, autoinc=false;
  int avail=0; void begin(){} void setClock(unsigned long){}
  void beginTransmission(uint8_t a){addr=a; first=true;}
  void write(uint8_t v){ if(first){ptr=v&0x7F; autoinc=v&0x80; first=false;} else { regs[addr][ptr]=v; ptr++; } }
  int endTransmission(){return 0;}
  uint8_t requestFrom(uint8_t a, uint8_t k){addr=a; avail=k; return k;}
  int available(){return avail;}
  int read(){ avail--; return regs[addr][ptr++]; }
};
extern FakeWire Wire;
