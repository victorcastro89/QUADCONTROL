/*
 * Filtro do giroscopio, selecionavel por constante (item 4 / etapa 4a).
 *
 * O PADRAO e a media movel de 6 amostras, identica a de antes (inclusive a
 * divisao inteira). Os outros tipos existem para comparar depois de medir o
 * espectro de vibracao (GYRO_LOG + espectro_gyro.py).
 *
 *   Filtro (corte ~30 Hz)     atraso em baixa freq.   atenuacao em 100 Hz
 *   media movel de 6          6,25 ms                 forte (zeros 67/133 Hz)
 *   PT1 (1a ordem)            1/(2*pi*fc)             fraca
 *   biquad Butterworth        sqrt(2)/(2*pi*fc)       boa (-40 dB/decada)
 *
 * O ganho real vem de SUBIR o corte (60-80 Hz) se a vibracao permitir, com um
 * notch opcional na frequencia das helices.
 */
#ifndef GYRO_FILTER_H
#define GYRO_FILTER_H

#define GYRO_FILTER_MOVING_AVG  0
#define GYRO_FILTER_PT1         1
#define GYRO_FILTER_BIQUAD      2

/* ---------- Configuracao ---------- */
#define GYRO_FILTER_TYPE   GYRO_FILTER_MOVING_AVG  // padrao: igual ao original
#define GYRO_SAMPLE_HZ     400.0   // taxa do laco (tick de 2500 us)
#define GYRO_MA_SIZE       6       // amostras da media movel
#define GYRO_LPF_HZ        70.0    // corte do PT1 / biquad
#define GYRO_NOTCH_HZ      0.0     // 0 = notch desligado (vale p/ qualquer tipo)
#define GYRO_NOTCH_Q       2.0     // largura do notch (maior = mais estreito)

/* Biquad generico, forma direta II transposta. */
class Biquad {
public:
  Biquad() { setPassThrough(); }
  void setPassThrough();
  void setLowPass(float fc, float fs, float q = 0.70710678f);
  void setNotch(float f0, float fs, float q);
  void reset() { z1 = z2 = 0; }
  float apply(float x) {
    float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
private:
  float b0, b1, b2, a1, a2, z1, z2;
};

/* Media movel inteira, igual ao codigo original (soma / N com divisao inteira). */
class MovingAverage {
public:
  MovingAverage() { reset(); }
  void reset() { sum = 0; idx = 0; for (int i = 0; i < GYRO_MA_SIZE; i++) buf[i] = 0; }
  int apply(int x) {
    sum += x - buf[idx];
    buf[idx] = x;
    idx = (idx + 1) % GYRO_MA_SIZE;
    return sum / GYRO_MA_SIZE;
  }
private:
  int buf[GYRO_MA_SIZE];
  int sum, idx;
};

/* Filtro de 1a ordem (PT1). */
class PT1 {
public:
  PT1() : k(1), y(0) {}
  void set(float fc, float fs) {
    float rc = 1.0f / (2.0f * 3.14159265f * fc);
    float dt = 1.0f / fs;
    k = dt / (rc + dt);
  }
  void reset() { y = 0; }
  float apply(float x) { y += k * (x - y); return y; }
private:
  float k, y;
};

/* Um filtro por eixo, com o tipo escolhido acima + notch opcional. */
class GyroAxisFilter {
public:
  GyroAxisFilter();
  void reset();
  float apply(int raw);
private:
  MovingAverage ma;
  PT1 pt1;
  Biquad lpf;
  Biquad notch;
};

#endif
