#pragma once
struct Servo{ int us=0; void attach(int){} void writeMicroseconds(int v){us=v;} };
