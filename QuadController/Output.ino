void Print_Acc(){
  
    Serial.print("#A-");  Serial.print('=');
  Serial.print(accel_[0]); Serial.print(",");
  Serial.print(accel_[1]); Serial.print(",");
  Serial.print(accel_[2]); Serial.println();
}
void Print_Gyr(){
  
    Serial.print("#G-");  Serial.print('=');
  Serial.print(gyro[0]); Serial.print(",");
  Serial.print(gyro[1]); Serial.print(",");
  Serial.print(gyro[2]); Serial.println();
}

void Print_Gyr_Offset (void){
  
     Serial.println (" GYRO OFFSET");
 Serial.print (" gx= ");Serial.println (GxOffset);
 Serial.print (" gy= ");Serial.println (GyOffset);
 Serial.print (" gz= ");Serial.println (GzOffset); 
 Serial.println();
  
};
void Print_Acc_Offset (void){
  
     Serial.println (" ACC OFFSET");
     Serial.println("");
    Serial.print(" Ofset X: "); Serial.print( mediax);
    Serial.print(" Ofset  Y: "); Serial.print( mediay);
    Serial.print(" Ofset  Z: "); Serial.print( mediaz);
    Serial.print(" Round X: "); Serial.print( -round(mediax));
    Serial.println("");
  
};
void Print_angle_acc (void){
  
  Serial.print("#ACC Angle P - R");  Serial.print('=');
  Serial.print(TO_DEG(Pitch_angle_acc)); Serial.print(",");
  Serial.print(TO_DEG(Roll_angle_acc)); Serial.print(",");
 
 Serial.println();
  
}
void Print_Bpm(void){
    Serial.print("Pressure:    ");
    Serial.print(pressure);
    Serial.println(" hPa");
   Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");
    Serial.print("Altitude:    "); 
    Serial.print(altitude); 
    Serial.println(" m");
    Serial.println("");
}
void Print_angle_gyro (void){
  
  Serial.print("#GYRO Angle P - R");  Serial.print('=');
  Serial.print(TO_DEG(gyro_angle[0])); Serial.print(",");
  Serial.print(TO_DEG(gyro_angle[1])); Serial.print(",");
  
    //Serial.print(RollRate_Input); Serial.print(",");
  //Serial.print( PitchRate_Input); Serial.print(",");

 Serial.println();
  
}
void Print_angle_fused (void){
  
  Serial.print("#FUSED Angle P - R - Y");  Serial.print('=');
  Serial.print(TO_DEG(Pitch_Input)); Serial.print(",");
  Serial.print(TO_DEG(Roll_Input)); Serial.print(",");
  Serial.print(TO_DEG(Yaw_Input)); Serial.print(","); 
 Serial.println();
  
}
void Compass_calibrate (void){
  
if(compass_flag){
   Serial.println('A');   // send a capital A
   Serial.println((long)micros());
   Serial.println((float)magnetom[0]); 
   Serial.println((float)magnetom[1]); 
 compass_flag=0;  
}
}
void Acc_gyr_fusion(){
    Serial.println('A');   // send a capital A
   Serial.println((long)micros());
   Serial.println((float)Pitch_angle_acc); 
   Serial.println((float)gyro_pitch ); 
   Serial.println((float)gyro_angle[0]);
   Serial.println((float)Roll_angle_acc); 
   Serial.println((float)gyro_roll ); 
   Serial.println((float)gyro_angle[1]);
}
void Compass_reading (void){
  
if(compass_flag){
   Serial.println('A');   // send a capital A
   Serial.println((long)micros());
   Serial.println((float)magnetom[0]); 
   Serial.println((float)magnetom[1]); 
 compass_flag=0;  
}
}


/*
 * Saida dos motores (constantes em Safety.h). Mixer em X:
 *   M1 = T + R + P - Y    M2 = T - R + P + Y
 *   M3 = T - R - P - Y    M4 = T + R - P + Y
 * Em voo cada motor fica em [MOTOR_MIN, MOTOR_MAX]. Prioridade para a atitude:
 * se as correcoes nao cabem na faixa, elas sao reduzidas na mesma proporcao;
 * depois o throttle e deslocado para que todos os motores caibam.
 */
void Mix_Motors(){
  double m[4];
  bool saturated = false;

  if(flight_state == STATE_DISARMED){
    for(int i=0;i<4;i++) m[i] = MOTOR_OFF;
    Reset_Controllers();
  }
  else{
    double thr = (flight_state == STATE_FAILSAFE) ? fs_throttle : rc_throttle;

    if(thr < THROTTLE_LOW){
      // Armado no chao: helices girando devagar, PIDs zerados.
      for(int i=0;i<4;i++) m[i] = MOTOR_IDLE;
      Reset_Controllers();
    }
    else{
      if(thr > THROTTLE_MAX_IN) thr = THROTTLE_MAX_IN;

      double c[4];
      c[0] =  RollRate_Output + PitchRate_Output - YawRate_Output;
      c[1] = -RollRate_Output + PitchRate_Output + YawRate_Output;
      c[2] = -RollRate_Output - PitchRate_Output - YawRate_Output;
      c[3] =  RollRate_Output - PitchRate_Output + YawRate_Output;

      double cmax = c[0], cmin = c[0];
      for(int i=1;i<4;i++){
        if(c[i] > cmax) cmax = c[i];
        if(c[i] < cmin) cmin = c[i];
      }

      // 1) Correcoes maiores que a faixa util: reduz todas proporcionalmente.
      double range = MOTOR_MAX - MOTOR_MIN;
      double spread = cmax - cmin;
      if(spread > range){
        double k = range / spread;
        for(int i=0;i<4;i++) c[i] *= k;
        cmax *= k;
        cmin *= k;
        saturated = true;
      }

      // 2) Desloca o throttle para todos os motores caberem em [MIN, MAX].
      if(thr + cmax > MOTOR_MAX){ thr = MOTOR_MAX - cmax; saturated = true; }
      if(thr + cmin < MOTOR_MIN){ thr = MOTOR_MIN - cmin; saturated = true; }

      for(int i=0;i<4;i++) m[i] = thr + c[i];
    }
  }

  // Anti-windup: com saida saturada os integradores de taxa nao acumulam.
  motors_saturated = saturated;
  RollRate_PID.HoldIntegrator(saturated);
  PitchRate_PID.HoldIntegrator(saturated);
  YawRate_PID.HoldIntegrator(saturated);

  M1_out = (int)(m[0] + 0.5);
  M2_out = (int)(m[1] + 0.5);
  M3_out = (int)(m[2] + 0.5);
  M4_out = (int)(m[3] + 0.5);
  Motor1.writeMicroseconds(M1_out);
  Motor2.writeMicroseconds(M2_out);
  Motor3.writeMicroseconds(M3_out);
  Motor4.writeMicroseconds(M4_out);
}
