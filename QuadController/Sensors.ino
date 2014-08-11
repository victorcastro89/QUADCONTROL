/* This file is part of the Razor AHRS Firmware */

// I2C code to read the sensors

// Sensor I2C addresses
#define ACCEL_ADDRESS ((int16_t) 0x53) // 0x53 = 0xA6 / 2
#define MAGN_ADDRESS  ((int16_t) 0x1E) // 0x1E = 0x3C / 2
#define GYRO_ADDRESS  ((int16_t) 0x68) // 0x68 = 0xD0 / 2

// Arduino backward compatibility macros
#if ARDUINO >= 100
  #define WIRE_SEND(b) Wire.write((byte) b) 
  #define WIRE_RECEIVE() Wire.read() 
#else
  #define WIRE_SEND(b) Wire.send(b)
  #define WIRE_RECEIVE() Wire.receive() 
#endif

void I2C_Init()
{
  Wire.begin();
}
void  bmp_init(void){
if(!bmp.begin(BMP085_MODE_ULTRALOWPOWER))
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
}
 
void Accel_Init()
{
   if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }
  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_400_HZ);
 // calibra_acc();
set_acc_offset();

}
void set_acc_offset(){
      accel.Xofset(0);
      accel.Yofset(0);
      accel.Zofset(0);
      
      accel.Xofset( -round((float)23.24/4));
      accel.Yofset( -round((float)7/4));
      accel.Zofset( -round((float)-36.77/4));
      
}
void calibra_acc(void){
  
 int i;
  double Somax=0;
  double Somay=0;
  double Somaz=0;
      accel.Xofset(0);
      accel.Yofset(0);
      accel.Zofset(0);
 
   for (i=0;i<1000;i++){
   accel.getRaw(&event);
   double x=  event.acceleration.x;
   double y=  event.acceleration.y;
   double z=  event.acceleration.z;
   Somax += x ;
   Somay += y ;
   Somaz += z ;
  
//Ofset X: 44.88 Ofset  Y: -14.16 Ofset  Z: -32.00 
// Ofset X: -19.12 Ofset  Y: 17.77 Ofset  Z: 0.00 
   delay(3);
   }
   
   
   mediax =(double)Somax/1000;
    mediay =(double) Somay/1000;
    mediaz =  (double)(Somaz/1000) -256;
  //if(mediaz>9.8066) mediaz=9.8066;
   Serial.println("");
    Serial.print(" Ofset X: "); Serial.print( mediax);
    Serial.print(" Ofset  Y: "); Serial.print( mediay);
    Serial.print(" Ofset  Z: "); Serial.print( mediaz);
    Serial.print(" Round X: "); Serial.print( -round(mediax));
    Serial.println("");
      accel.Xofset( -round(mediax/4));
      accel.Yofset( -round(mediay/4));
      accel.Zofset( -round(mediaz/4));

}
void Read_bpm(void){
  sensors_event_t event;
  bmp.getEvent(&event);
  pressure =event.pressure;
  bmp.getTemperature(&temperature);
      
  

}
// Reads x, y and z accelerometer registers
void Read_Accel()
{
  accel.getRaw(&event);
    

    soma_acc[0] += -buffer_acc[0][indice_acc]; 
    soma_acc[1] += -buffer_acc[1][indice_acc]; 
    soma_acc[2] += -buffer_acc[2][indice_acc]; 
    soma_acc[0] += +(int)event.acceleration.x; 
    soma_acc[1] += +(int)event.acceleration.y; 
    soma_acc[2] += +(int)event.acceleration.z; 
    
    buffer_acc[0][indice_acc] = (int)event.acceleration.x;
    buffer_acc[1][indice_acc] = (int)event.acceleration.y;
    buffer_acc[2][indice_acc] = (int)event.acceleration.z;
    
    if(indice_acc==(tamanho_acc-1))
    indice_acc=0;
    else 
    indice_acc++;
    
    accel_[0] =-soma_acc[0]/tamanho_acc ; 
    accel_[1] =-soma_acc[1]/tamanho_acc ; 
    accel_[2] =soma_acc[2]/tamanho_acc ; 
    
  float x2 =accel_[0]*accel_[0];
  float y2 =accel_[1]*accel_[1];
  float z2 =accel_[2]*accel_[2];
  Roll_angle_acc =( atan2(-accel_[0],accel_[2]) );
  Pitch_angle_acc = -(atan2(accel_[1],sqrt( x2 + z2)) );

}

void Magn_Init()
{
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x02); 
  WIRE_SEND(0x00);  // Set continuous mode (default 10Hz)
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x00);
  WIRE_SEND(0b00011000);  // Set 50Hz
  Wire.endTransmission();
  delay(5);
}

void Read_Magn()
{
  int16_t i = 0;
  byte buff[6];
 
  Wire.beginTransmission(MAGN_ADDRESS); 
  WIRE_SEND(0x03);  // Send address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(MAGN_ADDRESS); 
  Wire.requestFrom(MAGN_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6)  // All bytes received?
  {
    magnetom_raw[0] = (int16_t)(((int16_t) buff[0]) << 8) | buff[1];                                           
    magnetom_raw[1] =(int16_t) (1 * ((((int16_t) buff[4]) << 8) | buff[5]));                                     
    magnetom_raw[2] = (int16_t)(1 * ((((int16_t) buff[2]) << 8) | buff[3]));   
}
/*
	for (int i = 0; i < 3; i++)
	magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
	Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
	 */
 Mag_filter[0] = ((magnetom_raw[0] + 2*L_magnetom_raw[0] + LL_magnetom_raw[0])/4) + L_Mag_filter[0] - LL_Mag_filter[0]/4;
 Mag_filter[1] = ((magnetom_raw[1] + 2*L_magnetom_raw[1] + LL_magnetom_raw[1])/4) + L_Mag_filter[1] - LL_Mag_filter[1]/4;
 
  LL_Mag_filter[0] =  L_Mag_filter[0];
  L_Mag_filter[0] = Mag_filter[0];
 
 LL_Mag_filter[1] =  L_Mag_filter[1];
 L_Mag_filter[1] = Mag_filter[1];

  LL_magnetom_raw[0] =  L_magnetom_raw[0] ;
 L_magnetom_raw[0] = magnetom_raw[0] ;

 LL_magnetom_raw[1] =  L_magnetom_raw[1] ;
 L_magnetom_raw[1] = magnetom_raw[1] ;

 /*
 L_ magnetom_raw[2] = magnetom_raw[2] ;
 LL_ magnetom_raw[2] =  L_ magnetom_raw[2] ;
  magnetom[0] =   Mag_filter[0]* 1.0398 -  23.3956;
 magnetom[1] =   Mag_filter[1]*  0.9617 + 804.9586;
 */

 magnetom[0] =   Mag_filter[0]*1.0151 - 36.0366;
 magnetom[1] =   Mag_filter[1]*0.9851 + 841.2837;
compass_flag=1;
}

void Gyro_Init()
{
    if (!Gyr.init())
  {
   // Serial.println("Failed to autodetect gyro type!");
    while (1);
  }
  Gyr.enableDefault();
    Gyr.setFilterMode(0b11);
    Gyr.setHighPassOnOff(1);
    Gyr.setOutputDataRate(400);
    Gyr.scale(500);
    calibra_gyr();
    
}
 void calibra_gyr(){
   
   long sumx=0,sumy=0,sumz=0;
   int i;
   for(i=0;i<1000;i++){
     Read_Gyro();
   sumx+= Gyr.g.x ;
   sumy+= Gyr.g.y ;
   sumz+= Gyr.g.z ;
   delay(4);
   }
   GxOffset =  sumx/1000;
   GyOffset =  sumy/1000;
   GzOffset =  sumz/1000;
 
 
 
 
 

  }
// Reads x, y and z gyroscope registers
void Read_Gyro()
{

  Gyr.read();
 

  double delta_t = (double)delta_gyro/1000000;
    soma_gyr[0] += -buffer_gyr[0][indice_gyr]; 
    soma_gyr[1] += -buffer_gyr[1][indice_gyr]; 
    soma_gyr[2] += -buffer_gyr[2][indice_gyr]; 
    soma_gyr[0] += +(int)(Gyr.g.x - GxOffset); 
    soma_gyr[1] += +(int)(Gyr.g.y - GyOffset); 
    soma_gyr[2] += +(int)(Gyr.g.z - GzOffset); 
    
    buffer_gyr[0][indice_gyr] = (int)(Gyr.g.x - GxOffset);
    buffer_gyr[1][indice_gyr] = (int)(Gyr.g.y - GyOffset);
    buffer_gyr[2][indice_gyr] = (int)(Gyr.g.z - GzOffset);
    
    if(indice_gyr==(tamanho_gyr-1))
    indice_gyr=0;
    else 
    indice_gyr++;
    
    gyro[0] = soma_gyr[0]/tamanho_gyr;
    gyro[1] = soma_gyr[1]/tamanho_gyr;
    gyro[2] = soma_gyr[2]/tamanho_gyr;
  
    PitchRate_Input = -gyro[0];//  *  0.2738 + 0.7262*PitchRate_Input_;
    RollRate_Input = -gyro[1];// * 0.2738 + 0.7262*RollRate_Input_;
    YawRate_Input =  -gyro[2];//  * 0.2738 + 0.7262*YawRate_Input_;
    gyro_angle[1] += 0.00030543261909*RollRate_Input* delta_t; //roll rad/s 0.085 sensivity
    gyro_angle[0] += 0.00030543261909*-PitchRate_Input* delta_t; //pitch rad/s
    gyro_angle[2] += 0.00030543261909*-YawRate_Input* delta_t; //pitch rad/s
    gyro_pitch += 0.00030543261909*-PitchRate_Input* delta_t;
    gyro_roll += 0.00030543261909*RollRate_Input* delta_t;
    gyro_angle[0] = gyro_angle[0]*0.9995 + Pitch_angle_acc*0.0005 ;
    gyro_angle[1] = gyro_angle[1]*0.9995+ Roll_angle_acc*0.0005 ;
    gyro_angle[2] =   gyro_angle[2]*0.99 + MAG_Heading*0.01 ;
    Yaw_Input = - (gyro_angle[2] -Yaw_offset);
    Pitch_Input = -gyro_angle[0];
    Roll_Input = gyro_angle[1];
   //PitchRate_Input_=PitchRate_Input;
   //RollRate_Input_=RollRate_Input;
   //YawRate_Input_=YawRate_Input;
}

