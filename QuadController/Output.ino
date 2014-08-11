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

