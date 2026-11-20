#include "GY80.h"

/* Registradores usados (datasheets ADXL345 rev. E e L3G4200D rev. 3) */
#define ADXL_REG_DEVID        0x00   // deve ler 0xE5
#define ADXL_REG_BW_RATE      0x2C
#define ADXL_REG_POWER_CTL    0x2D
#define ADXL_REG_DATA_FORMAT  0x31
#define ADXL_REG_DATAX0       0x32   // X0 X1 Y0 Y1 Z0 Z1 (little-endian)

#define L3G_WHO_AM_I          0x0F   // L3G4200D = 0xD3
#define L3G_CTRL_REG1         0x20   // DR[7:6] BW[5:4] PD Zen Yen Xen
#define L3G_CTRL_REG2         0x21   // HPM[5:4] HPCF[3:0]
#define L3G_CTRL_REG4         0x23   // BDU BLE FS[5:4] - ST SIM
#define L3G_CTRL_REG5         0x24   // BOOT FIFO_EN - HPen INT1_Sel OUT_Sel[1:0]
#define L3G_OUT_X_L           0x28
#define L3G_AUTO_INCREMENT    0x80   // bit 7 do endereco: leitura sequencial

/* ------------------------------------------------------------------ */
/* Acesso I2C comum                                                    */
/* ------------------------------------------------------------------ */

static void i2cWrite(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

static uint8_t i2cRead(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

// Le n bytes a partir de reg. Retorna false se faltou algum.
static bool i2cReadBlock(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(addr, n);
  uint8_t i = 0;
  while (Wire.available() && i < n) buf[i++] = Wire.read();
  return i == n;
}

/* ------------------------------------------------------------------ */
/* ADXL345                                                             */
/* ------------------------------------------------------------------ */

void    GY80_Accel::writeReg(uint8_t reg, uint8_t value) { i2cWrite(_addr, reg, value); }
uint8_t GY80_Accel::readReg(uint8_t reg)                  { return i2cRead(_addr, reg); }

bool GY80_Accel::begin() {
  Wire.begin();
  if (readReg(ADXL_REG_DEVID) != 0xE5) return false;
  writeReg(ADXL_REG_POWER_CTL, 0x08);   // Measure = 1 (igual a biblioteca Adafruit)
  return true;
}

void GY80_Accel::setRange(uint8_t range) {
  // Igual ao Adafruit setRange(): troca so os bits de faixa e liga FULL_RES (bit 3).
  // Com FULL_RES a escala e 3,9 mg/LSB em qualquer faixa: 1 g = 256 contagens,
  // valor que o codigo assume (calibra_acc subtrai 256 do eixo Z).
  uint8_t format = readReg(ADXL_REG_DATA_FORMAT);
  format &= ~0x0F;
  format |= (range & 0x03) | 0x08;
  writeReg(ADXL_REG_DATA_FORMAT, format);
}

void GY80_Accel::setDataRate(uint8_t rate) {
  writeReg(ADXL_REG_BW_RATE, rate & 0x0F);
}

void GY80_Accel::getRaw(int16_t *x, int16_t *y, int16_t *z) {
  uint8_t b[6];
  if (!i2cReadBlock(_addr, ADXL_REG_DATAX0, b, 6)) return;  // mantem a leitura anterior
  *x = (int16_t)((b[1] << 8) | b[0]);
  *y = (int16_t)((b[3] << 8) | b[2]);
  *z = (int16_t)((b[5] << 8) | b[4]);
}

void GY80_Accel::getRaw(sensors_event_t *event) {
  int16_t x = (int16_t)event->acceleration.x;
  int16_t y = (int16_t)event->acceleration.y;
  int16_t z = (int16_t)event->acceleration.z;
  getRaw(&x, &y, &z);
  event->acceleration.x = x;
  event->acceleration.y = y;
  event->acceleration.z = z;
}

/* ------------------------------------------------------------------ */
/* L3G4200D                                                            */
/* ------------------------------------------------------------------ */

void    GY80_Gyro::writeReg(uint8_t reg, uint8_t value) { i2cWrite(_addr, reg, value); }
uint8_t GY80_Gyro::readReg(uint8_t reg)                  { return i2cRead(_addr, reg); }

bool GY80_Gyro::init() {
  Wire.begin();
  // Mesma busca do Pololu L3G::init() para o L3G4200D: SDO alto (0x69, GY-80) ou baixo (0x68).
  const uint8_t candidates[2] = { 0x69, 0x68 };
  for (uint8_t i = 0; i < 2; i++) {
    _addr = candidates[i];
    if (readReg(L3G_WHO_AM_I) == 0xD3) return true;
  }
  _addr = 0;
  return false;
}

void GY80_Gyro::enableDefault() {
  // SUPOSICAO: igual ao enableDefault() do Pololu L3G (versao com autodeteccao):
  //   CTRL_REG4 = 0x00 -> +-250 dps
  //   CTRL_REG1 = 0x6F -> DR = 01 (200 Hz), BW = 10, PD = 1, X/Y/Z ligados
  // Versoes mais antigas da Pololu escreviam CTRL_REG1 = 0x0F (BW = 00). O BW
  // so define o corte do passa-baixa interno; a escala e a taxa sao
  // sobrescritas logo depois por scale(500) e setOutputDataRate(400).
  writeReg(L3G_CTRL_REG4, 0x00);
  writeReg(L3G_CTRL_REG1, 0x6F);
}

void GY80_Gyro::setFilterMode(uint8_t hpm) {
  // SUPOSICAO: "filter mode" = HPM, modo do passa-alta (CTRL_REG2 bits 5:4).
  // 0b11 = "autoreset on interrupt". O corte (HPCF, bits 3:0) fica no padrao.
  uint8_t r = readReg(L3G_CTRL_REG2);
  r = (r & ~0x30) | ((hpm & 0x03) << 4);
  writeReg(L3G_CTRL_REG2, r);
}

void GY80_Gyro::setHighPassOnOff(bool on) {
  // Liga/desliga HPen (CTRL_REG5 bit 4). OUT_Sel (bits 1:0) fica em 00, e
  // nesse caso o registrador de saida recebe o dado so do LPF1: o passa-alta
  // NAO entra no dado lido (apenas no caminho de interrupcao). Com HPCF padrao
  // e ODR 400 Hz o corte do passa-alta seria 30 Hz; se ele estivesse no dado,
  // o controle de taxa e o filtro complementar nao teriam funcionado em voo.
  uint8_t r = readReg(L3G_CTRL_REG5);
  if (on) r |= 0x10; else r &= ~0x10;
  writeReg(L3G_CTRL_REG5, r);
}

void GY80_Gyro::setOutputDataRate(int hz) {
  // Troca so o campo DR (bits 7:6) e preserva BW, PD e eixos.
  uint8_t dr;
  switch (hz) {
    case 100: dr = 0; break;
    case 200: dr = 1; break;
    case 400: dr = 2; break;
    case 800: dr = 3; break;
    default:  return;
  }
  uint8_t r = readReg(L3G_CTRL_REG1);
  r = (r & ~0xC0) | (dr << 6);
  writeReg(L3G_CTRL_REG1, r);
}

void GY80_Gyro::scale(int dps) {
  // FS (bits 5:4): 00 = 250 dps (8,75 mdps/LSB), 01 = 500 dps (17,5 mdps/LSB),
  // 10/11 = 2000 dps (70 mdps/LSB). O codigo usa 17,5 mdps/LSB -> scale(500).
  uint8_t fs;
  switch (dps) {
    case 250:  fs = 0; break;
    case 500:  fs = 1; break;
    case 2000: fs = 2; break;
    default:   return;
  }
  uint8_t r = readReg(L3G_CTRL_REG4);
  r = (r & ~0x30) | (fs << 4);
  writeReg(L3G_CTRL_REG4, r);
}

void GY80_Gyro::read() {
  // Igual ao Pololu L3G::read(): 6 bytes a partir de OUT_X_L com auto-incremento.
  uint8_t b[6];
  if (!i2cReadBlock(_addr, L3G_OUT_X_L | L3G_AUTO_INCREMENT, b, 6)) return;  // mantem a anterior
  g.x = (int16_t)((b[1] << 8) | b[0]);
  g.y = (int16_t)((b[3] << 8) | b[2]);
  g.z = (int16_t)((b[5] << 8) | b[4]);
}
