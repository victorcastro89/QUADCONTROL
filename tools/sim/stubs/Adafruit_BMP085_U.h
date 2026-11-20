#pragma once
#include "Adafruit_Sensor.h"
#define BMP085_MODE_ULTRALOWPOWER 0
struct Adafruit_BMP085_Unified{ Adafruit_BMP085_Unified(int){} bool begin(int){return true;} void getEvent(sensors_event_t*){} void getTemperature(float*){} float pressureToAltitude(float,float,float){return 0;} };
