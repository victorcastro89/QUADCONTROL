/*
 * Drivers minimos para a placa GY-80 (ADXL345 + L3G4200D), via Wire.
 *
 * Substituem as copias MODIFICADAS de Adafruit_ADXL345_U e L3G (Pololu) que
 * existiam so no computador original. Os nomes dos metodos sao os mesmos que
 * o codigo ja usava (getRaw, Xofset, setOutputDataRate, scale, ...), mas cada
 * um agora e uma escrita/leitura explicita de registrador do datasheet.
 *
 * Os valores exatos que as bibliotecas originais escreviam nao sao conhecidos;
 * as escolhas feitas estao documentadas em GY80.cpp (procure "SUPOSICAO").
 */
#ifndef GY80_H
#define GY80_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>   // so pelo tipo sensors_event_t (biblioteca oficial)

/* ------------------------------------------------------------------ */
/* ADXL345 - acelerometro                                              */
/* ------------------------------------------------------------------ */

// Valores do campo "range" do registrador DATA_FORMAT (0x31)
#define GY80_ADXL_RANGE_2_G   0x00
#define GY80_ADXL_RANGE_4_G   0x01
#define GY80_ADXL_RANGE_8_G   0x02
#define GY80_ADXL_RANGE_16_G  0x03

// Valores do registrador BW_RATE (0x2C)
#define GY80_ADXL_DATARATE_100_HZ  0x0A
#define GY80_ADXL_DATARATE_200_HZ  0x0B
#define GY80_ADXL_DATARATE_400_HZ  0x0C
#define GY80_ADXL_DATARATE_800_HZ  0x0D

class GY80_Accel {
public:
  GY80_Accel(uint8_t address = 0x53) : _addr(address) {}

  // Confere o ID (0xE5) e liga o modo de medicao. Retorna false se nao achar.
  bool begin();
  // Faixa de medicao com FULL_RES ligado: 3,9 mg/LSB em qualquer faixa (1 g = 256).
  void setRange(uint8_t range);
  void setDataRate(uint8_t rate);

  // Leitura BRUTA (contagens, 256 = 1 g) nos campos event->acceleration.x/y/z.
  // Diferente do getEvent() oficial, que converte para m/s^2.
  void getRaw(sensors_event_t *event);
  void getRaw(int16_t *x, int16_t *y, int16_t *z);

  // Offsets em hardware (OFSX/OFSY/OFSZ, 15,6 mg/LSB = 4 contagens brutas).
  void Xofset(int8_t v) { writeReg(0x1E, (uint8_t)v); }
  void Yofset(int8_t v) { writeReg(0x1F, (uint8_t)v); }
  void Zofset(int8_t v) { writeReg(0x20, (uint8_t)v); }

  void    writeReg(uint8_t reg, uint8_t value);
  uint8_t readReg(uint8_t reg);

private:
  uint8_t _addr;
};

/* ------------------------------------------------------------------ */
/* L3G4200D - giroscopio                                               */
/* ------------------------------------------------------------------ */

class GY80_Gyro {
public:
  struct vector { int16_t x, y, z; };
  vector g;   // ultima leitura bruta (mesmo nome do Pololu L3G: Gyr.g.x ...)

  GY80_Gyro() : _addr(0) { g.x = g.y = g.z = 0; }

  // Procura o L3G4200D em 0x69 (padrao da GY-80) e 0x68. Retorna false se nao achar.
  bool init();
  // Liga o sensor nos 3 eixos (equivalente ao enableDefault do Pololu).
  void enableDefault();
  // Campo HPM (modo do passa-alta) em CTRL_REG2, bits 5:4.
  void setFilterMode(uint8_t hpm);
  // Bit HPen em CTRL_REG5. Ver observacao em GY80.cpp sobre o efeito real.
  void setHighPassOnOff(bool on);
  // Taxa de saida: 100, 200, 400 ou 800 Hz (campo DR de CTRL_REG1).
  void setOutputDataRate(int hz);
  // Fundo de escala: 250, 500 ou 2000 graus/s (campo FS de CTRL_REG4).
  void scale(int dps);
  // Le os 3 eixos para g.x, g.y, g.z.
  void read();

  void    writeReg(uint8_t reg, uint8_t value);
  uint8_t readReg(uint8_t reg);

private:
  uint8_t _addr;
};

#endif
