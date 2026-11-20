#include <math.h>
#include "GyroFilter.h"

/* Coeficientes pelo "Audio EQ Cookbook" (R. Bristow-Johnson). */

void Biquad::setPassThrough() {
  b0 = 1; b1 = b2 = a1 = a2 = 0;
  z1 = z2 = 0;
}

void Biquad::setLowPass(float fc, float fs, float q) {
  float w0 = 2.0f * 3.14159265f * fc / fs;
  float alpha = sinf(w0) / (2.0f * q);
  float c = cosf(w0);
  float a0 = 1.0f + alpha;
  b0 = (1.0f - c) / 2.0f / a0;
  b1 = (1.0f - c) / a0;
  b2 = b0;
  a1 = -2.0f * c / a0;
  a2 = (1.0f - alpha) / a0;
  reset();
}

void Biquad::setNotch(float f0, float fs, float q) {
  float w0 = 2.0f * 3.14159265f * f0 / fs;
  float alpha = sinf(w0) / (2.0f * q);
  float c = cosf(w0);
  float a0 = 1.0f + alpha;
  b0 = 1.0f / a0;
  b1 = -2.0f * c / a0;
  b2 = b0;
  a1 = b1;
  a2 = (1.0f - alpha) / a0;
  reset();
}

GyroAxisFilter::GyroAxisFilter() {
  pt1.set(GYRO_LPF_HZ, GYRO_SAMPLE_HZ);
  lpf.setLowPass(GYRO_LPF_HZ, GYRO_SAMPLE_HZ);
  if (GYRO_NOTCH_HZ > 0)
    notch.setNotch(GYRO_NOTCH_HZ, GYRO_SAMPLE_HZ, GYRO_NOTCH_Q);
}

void GyroAxisFilter::reset() {
  ma.reset();
  pt1.reset();
  lpf.reset();
  notch.reset();
}

float GyroAxisFilter::apply(int raw) {
#if GYRO_FILTER_TYPE == GYRO_FILTER_MOVING_AVG
  // Igual ao original: media inteira; o notch (se ligado) vem antes.
  int x = (GYRO_NOTCH_HZ > 0) ? (int)lroundf(notch.apply((float)raw)) : raw;
  return (float)ma.apply(x);
#elif GYRO_FILTER_TYPE == GYRO_FILTER_PT1
  return pt1.apply(notch.apply((float)raw));
#else
  return lpf.apply(notch.apply((float)raw));
#endif
}
