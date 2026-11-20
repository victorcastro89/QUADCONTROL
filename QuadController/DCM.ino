/* This file is part of the Razor AHRS Firmware */

// DCM algorithm


// Chamado uma vez por tick (2500 us) pelo loop(); delta_gyro = dt medido do tick.
// Giroscopio e acelerometro a cada tick; bussola a cada MAG_EVERY_TICKS (50 Hz).
// (O barometro continua desligado; a versao antiga esta no historico do git.)
void read_sensors() {
  static int mag_count = 0;
  delta_gyro = tick_dt;
  Read_Gyro();
  delta_gyro_aux = delta_gyro;
  Read_Accel();
  if(++mag_count >= MAG_EVERY_TICKS){
    mag_count = 0;
    Read_Magn();
    Compass_Heading();
  }
}

// Modo de registro p/ analise de vibracao (GYRO_LOG = 1): pela USB nativa do DUE,
// uma linha cada: 'A', tempo em us, 3 brutos, 3 filtrados.
// Ler com espectro_gyro.py (Python: pip install pyserial numpy matplotlib).
void Log_Gyro(){
#if GYRO_LOG
  if(!SerialUSB) return;
  SerialUSB.println('A');
  SerialUSB.println((long)micros());
  for(int i=0;i<3;i++) SerialUSB.println(gyro_raw[i]);
  for(int i=0;i<3;i++) SerialUSB.println(gyro[i]);
#endif
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
  /*
	// Compensate accelerometer error
	accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
	accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
	accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;
*/
	// Compensate magnetometer error
	#if CALIBRATION__MAGN_USE_EXTENDED == true
	for (int i = 0; i < 3; i++)
	magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
	Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
	#else
	magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
	magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
	magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
	#endif

	// Compensate gyroscope error
	//gyro[0] -= GYRO_AVERAGE_OFFSET_X;
	//gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
	//gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}

void Pid_Init(){
  
  
  Pitch_PID.SetMode(AUTOMATIC);
  Pitch_PID.SetSampleTime(2500);
  Pitch_PID.SetOutputLimits(-20000, 20000);
  Pitch_PID.ErrorTolerance(0.002);  // rad (~0,1 grau): so congela o I

  Roll_PID.SetMode(AUTOMATIC);
  Roll_PID.SetSampleTime(2500);
  Roll_PID.SetOutputLimits(-20000, 20000);
  Roll_PID.ErrorTolerance(0.002);   // rad (~0,1 grau)

  Yaw_PID.SetMode(AUTOMATIC);
  Yaw_PID.SetSampleTime(2400);
  Yaw_PID.SetOutputLimits(- 20000,  20000);
  Yaw_PID.ErrorTolerance(0.002);    // rad (~0,1 grau)
  PitchRate_PID.SetMode(AUTOMATIC);
  PitchRate_PID.SetSampleTime(2500);
  PitchRate_PID.SetOutputLimits(-150 , 150);
  PitchRate_PID.ErrorTolerance(30);  // LSB do giro (~0,5 grau/s)

  RollRate_PID.SetMode(AUTOMATIC);
  RollRate_PID.SetSampleTime(2500);
  RollRate_PID.SetOutputLimits(-150, 150);
  RollRate_PID.ErrorTolerance(30);   // LSB do giro (~0,5 grau/s)

  YawRate_PID.SetMode(AUTOMATIC);
  YawRate_PID.SetSampleTime(2500);
  YawRate_PID.SetOutputLimits(-150, 150);
  YawRate_PID.ErrorTolerance(30);    // LSB do giro (~0,5 grau/s)
}
