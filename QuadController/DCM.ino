/* This file is part of the Razor AHRS Firmware */

// DCM algorithm


void read_sensors() {
   delta_gyro =micros()-Last_TimeGyro;
  if(delta_gyro >=2500){
    
       //  Serial.println(micros()-Last_TimeGyro);                                     
	Read_Gyro();                  
        Last_TimeGyro = micros();
      delta_gyro_aux=delta_gyro;
       
  }
  delta_acc = micros()-Last_TimeAcc;
   if(delta_acc>=2500){
	Read_Accel();                      
                          
	Last_TimeAcc = micros();
delta_acc_aux=delta_acc;
}
  delta_mag=millis()-Last_TimeMag;
   if(delta_mag>=20){
        Read_Magn();                     
	Last_TimeMag = millis();
        Compass_Heading();
        
       //Compass_compensate();
  
        delta_mag_aux=delta_mag;
}
/*
delta_bmp=millis()-Last_Timebmp;
   if(delta_bmp>=6){
        Read_bpm();              
	Last_Timebmp= millis();
        bpm_cont++;
        
        Soma_pressure +=pressure;
        Soma_temperature +=  temperature;
 if(bpm_cont==10){
   
   pressure=Soma_pressure/10;
  temperature = Soma_temperature/10;
     float seaLevelPressure = 1009.25;
  
    altitude =bmp.pressureToAltitude(seaLevelPressure,
                                        pressure,
                                        temperature);

     Print_Bpm();
    Soma_pressure=0;
    Soma_temperature=0;
    bpm_cont=0;
 }

   
}
 */
//Pitch_Input = gyro_angle[1]*0.96 + Pitch_angle*0.04;
//Roll_Input = gyro_angle[0]*0.96 + Roll_angle*0.04;
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
  Pitch_PID.ErrorTolerance(0.02);

  Roll_PID.SetMode(AUTOMATIC);
  Roll_PID.SetSampleTime(2500);
  Roll_PID.SetOutputLimits(-20000, 20000);
  Roll_PID.ErrorTolerance(0.02);

  Yaw_PID.SetMode(AUTOMATIC);
  Yaw_PID.SetSampleTime(2400);
  Yaw_PID.SetOutputLimits(- 20000,  20000);
    Yaw_PID.ErrorTolerance(0.02);
  PitchRate_PID.SetMode(AUTOMATIC);
  PitchRate_PID.SetSampleTime(2500);
  PitchRate_PID.SetOutputLimits(-150 , 150);
  PitchRate_PID.ErrorTolerance(300);

  RollRate_PID.SetMode(AUTOMATIC);
  RollRate_PID.SetSampleTime(2500);
  RollRate_PID.SetOutputLimits(-150, 150);
  RollRate_PID.ErrorTolerance(300);

  YawRate_PID.SetMode(AUTOMATIC);
  YawRate_PID.SetSampleTime(2500);
  YawRate_PID.SetOutputLimits(-150, 150);
  YawRate_PID.ErrorTolerance(500);
}
