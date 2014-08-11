
#include "I2Cdev.h"
#include <L3G.h>
#include <Wire.h>
#include <PWM.h>
#include "DCM.h"

#include "Capture_Functions.h"
#include "PID_v1.h"
#include "Servo.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP085_U.h>
   
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
int compass_flag=0;
L3G Gyr;
long aux;
float gyro_pitch,gyro_roll ;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(1);
sensors_event_t event; 
  
struct capture Roll;
struct capture Pitch;
struct capture Yaw;
struct capture Throttle;
struct capture Ch6; 
int RX_INT_PIN = 26;

Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;

bool CaptureInt_Disabled;

long Loop_Timer,lt,endtime,cycleTime,Last_TimeGyro,Last_TimeAcc,Last_TimeMag,previousTime,previousPidTime,PidTime,timer,timer_old,delta_gyro,delta_acc,delta_mag,delta_gyro_aux,delta_acc_aux,delta_mag_aux ;
long delta_bmp,Last_Timebmp;
float GxOffset,GyOffset,GzOffset;
float roll,pitch,yaw;

float accel_[3]; 
float accel__[3]; 
float accel___[3]; 
float accel____[3]; 
double mediax,mediay,mediaz;

float pressure,altitude,temperature,Soma_pressure,Soma_temperature;

float Pitch_angle_acc,Roll_angle_acc;
double gyro_angle[3];
double gyro_angle_raw[3];
float gyro[3];
float cos_yaw,sin_yaw;
float magnetom[3];
int magnetom_raw[3] ;
int L_magnetom_raw[3]; 
int LL_magnetom_raw[3]; int  Mag_filter[3]; int  L_Mag_filter[3];int  LL_Mag_filter[3];
float MAG_Heading;float MAG_Heading1;
float magnetom_min[3];
float magnetom_max[3];
float magnetom_tmp[3];

   double PitchRate_Input_,PitchRate_Input,RollRate_Input_,RollRate_Input,YawRate_Input_,YawRate_Input;
   double Pitch_Input,Roll_Input;
int led_value;

#define tamanho_acc 8
int soma_acc[3];
int buffer_acc[3][tamanho_acc];
int indice_acc;

#define tamanho_gyr 6
int soma_gyr[3];
int buffer_gyr[3][tamanho_gyr];
int indice_gyr;

int M1_out=1000;
int M2_out=1000;
int M3_out=1000;
int M4_out=1000;

double Yaw_Position,Roll_Position,Pitch_Position;
double Pitch_Setpoint;
double Pitch_Output;
double Roll_Setpoint;
double Roll_Output;
double Yaw_Setpoint, Yaw_Input;
double PitchRate_Setpoint;
double PitchRate_Output;
double RollRate_Setpoint;
double RollRate_Output;
double YawRate_Setpoint;

int bpm_cont=0;
double YawRate_Output,Yaw_Output;
//double kp = 0.02;
//double ki =0.008;
//double kp=0.02759;
//double ki= 0.1852;
//double kd = 0.0005610*0.0025;
//double kp = 0.0142659150; // ok tf2  
//double ki=  0.00408305;
//otimo
/*
double kp =0.022;
double ki=0.11;
double kd =0;
*/
double kp = 0.034944213;
double ki=0.2639735;
double kd=0.0025*0.00094357;
/* Very good
double kp =0.029654878;
double ki=0.35437;
double kd =0.000442071*0.0025;
*/
/* OKKK filtro 162
double kp =0.020;
double ki=0.1;
double kd =0.0001*0.0025;
*/
//Tbm ok
/*
double kp =0.03;
double ki=0.015918910;
double kd = 0.0;
/* 
double kp = 0.02656;
double ki = 0.004462;
double kd =0;
*/
PID_ PitchRate_PID(&PitchRate_Input, &PitchRate_Output, &PitchRate_Setpoint, kp,ki, kd, DIRECT)   ;
PID_ RollRate_PID(&RollRate_Input, &RollRate_Output, &RollRate_Setpoint, kp, ki, kd, DIRECT);
PID_ YawRate_PID(&YawRate_Input, &YawRate_Output, &YawRate_Setpoint,3*kp,ki,kd, DIRECT);

double k =25939;
double i =1588;
double d =625*0.0025;
PID_ Pitch_PID(&Pitch_Input, &Pitch_Output, &Pitch_Setpoint, k, i, d, DIRECT)   ;
PID_ Roll_PID(&Roll_Input, &Roll_Output, &Roll_Setpoint, k, i, d, DIRECT);
PID_ Yaw_PID(&Yaw_Input, &Yaw_Output, &Yaw_Setpoint, k/3, i/3, 0, DIRECT);

#define Loop_Time 1
    float Yaw_offset = 0;
    int hold_yaw=0;

int anglemode=0;
int Yaw_offset_flag=0;
void setup()
{
int j;
for (j=0;j<3;j++){
 soma_acc[j]=0; 
  soma_gyr[j]=0;
}
PitchRate_Input_ = 0;RollRate_Input_ = 0;YawRate_Input_ = 0;
  RollRate_Setpoint = 0;
  PitchRate_Setpoint = 0;
  YawRate_Setpoint = 0;
  Roll_Setpoint = 0;
  Pitch_Setpoint = 0;
  Yaw_Setpoint = 0;
  
  Motor1.attach(16);
  Motor2.attach(14);
  Motor3.attach(17);
  Motor4.attach(50);
  Pid_Init();
  startTimer_TC6(TC2, 0, TC6_IRQn); //Pino 5 ROW Canal 1
  startTimer_TC7(TC2, 1, TC7_IRQn); //Pino 3 Throttle Canal 3
  startTimer_TC8(TC2, 2, TC8_IRQn); //Pino 11 YAW Canal 4
  startTimer_TC0(TC0, 0, TC0_IRQn); //Pino 2  Pitch Canal 2
  attachInterrupt(RX_INT_PIN,rxGoesHigh,RISING);
  Serial.begin(57600);
  Serial.println("LIGUEI");
  delay(100);

  // Init status LED
  pinMode (STATUS_LED_PIN, OUTPUT);
 

  // Init sensors 
  delay(50);  // Give sensors enough time to start
   Serial.println("LIGUEI");
    Serial.println("LIGUEI");
  //I2C_Init();

  Accel_Init();
    bmp_init();
  Magn_Init();
  Gyro_Init();
  
  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
  
  
 //startTimer(TC2, 2, TC8_IRQn, 800);
 
}



void loop() {

   read_sensors();
/* 
  Loop_Timer = micros();

timer = millis();
  if (CaptureInt_Disabled == 1 &&  (timer - timer_old) >= 12) {
    NVIC_EnableIRQ(TC6_IRQn) ;
    NVIC_EnableIRQ(TC7_IRQn) ;
    NVIC_EnableIRQ(TC8_IRQn) ;
    NVIC_EnableIRQ(TC0_IRQn) ;
    timer_old = timer;

  }
*/
  if (Pitch.Flag && Yaw.Flag && Roll.Flag && Throttle.Flag) {
    
    /*
     NVIC_DisableIRQ(TC6_IRQn) ;
     NVIC_DisableIRQ(TC7_IRQn) ;
     NVIC_DisableIRQ(TC8_IRQn) ;
     NVIC_DisableIRQ(TC0_IRQn) ;
     */
    if (Yaw.T_alto < 1484 && Yaw.T_alto>1000 ) 
      Yaw_Position = (1484 - Yaw.T_alto)*-31;
    else if (Yaw.T_alto < 2100 && Yaw.T_alto>1000)
      Yaw_Position = (Yaw.T_alto - 1484  ) *31;

    if (Roll.T_alto < 1484 && Roll.T_alto>1000 )
      Roll_Position = (1484 - Roll.T_alto)*19;
    else if (Roll.T_alto < 2100 && Roll.T_alto>1000)
      Roll_Position = (Roll.T_alto - 1484  ) * -19;

    if (Pitch.T_alto < 1484 && Pitch.T_alto>1000 )
      Pitch_Position = (1484 - Pitch.T_alto) *-19;
    else if (Pitch.T_alto < 2100 && Pitch.T_alto>1000)
     Pitch_Position = (Pitch.T_alto - 1484  ) *19;


    if(Ch6.T_alto<1400){
      anglemode = 1;
      hold_yaw =0;
      Yaw_offset_flag=1;
  }
    else if(Ch6.T_alto<1700){
       anglemode = 0;
      hold_yaw =0;
      Yaw_offset_flag=1;
    }
        else if(Ch6.T_alto<2000){
       anglemode = 1;
      hold_yaw =1;
   if(Yaw_offset_flag==1){
   Yaw_offset = gyro_angle[2] ;
   cos_yaw = cos(gyro_angle[2]);
   sin_yaw = sin(gyro_angle[2]);
   }
     Yaw_offset_flag = 0;
    }
    
      if(Yaw_Position < 125 && Yaw_Position > -125  )
    Yaw_Position=0;
       if(Roll_Position < 125 && Roll_Position > -125)
    Roll_Position=0;
        if(Pitch_Position <125 && Pitch_Position > -125)
    Pitch_Position=0;
        
        
     if(hold_yaw){
        double pitchx = Pitch_Position*cos_yaw - Roll_Position*sin_yaw;
        double rollx = Pitch_Position*sin_yaw + Roll_Position*cos_yaw;  
    Pitch_Position = +(double)pitchx*cos(gyro_angle[2]) +(double)rollx*sin( gyro_angle[2]);
    Roll_Position = -(double)pitchx *sin(gyro_angle[2]) +(double)rollx*cos(gyro_angle[2]);
   
  }
  
  if(anglemode){
  Roll_Setpoint = Roll_Position/15000;
  Pitch_Setpoint = Pitch_Position/15000 ;
  Yaw_Setpoint = Yaw_Position/4060;
  RollRate_Setpoint = Roll_Output;
  PitchRate_Setpoint = Pitch_Output;
  YawRate_Setpoint = Yaw_Output;
  }
  else{
    
  RollRate_Setpoint = Roll_Position;
  PitchRate_Setpoint = Pitch_Position;
  YawRate_Setpoint = Yaw_Position;
  
  
  }


    //CaptureInt_Disabled = 1;
    Pitch.Flag = 0;
    Roll.Flag = 0;
    Yaw.Flag = 0;
    Throttle.Flag = 0;
  }
  
  
  if(anglemode){
      Pitch_PID.Compute();
       Roll_PID.Compute();
       Yaw_PID.Compute();
  }

  if(PitchRate_PID.Compute()){
  PidTime = micros()-previousTime;
  previousPidTime =micros();
  }
  RollRate_PID.Compute() ;
  YawRate_PID.Compute() ;

  if (Throttle.T_alto >= 1100) {
    if(Throttle.T_alto >= 1930) Throttle.T_alto=1930;
    M1_out = Throttle.T_alto  + RollRate_Output + PitchRate_Output  - YawRate_Output;

    M2_out = Throttle.T_alto  - RollRate_Output  + PitchRate_Output + YawRate_Output;

    M3_out = Throttle.T_alto  - RollRate_Output - PitchRate_Output - YawRate_Output;

    M4_out = Throttle.T_alto  + RollRate_Output  - PitchRate_Output + YawRate_Output;
  }
  else {
    M1_out = 1000;
    M2_out = 1000;
    M3_out = 1000;
    M4_out = 1000;
    RollRate_PID.Reset();
    PitchRate_PID.Reset();
    YawRate_PID.Reset();
    Roll_PID.Reset();
    Pitch_PID.Reset();
    Yaw_PID.Reset();

  }

  Motor1.writeMicroseconds(M1_out);
  Motor2.writeMicroseconds(M2_out);
  Motor3.writeMicroseconds(M3_out);
  Motor4.writeMicroseconds(M4_out);

#if 1
  long t2 = micros();
  
  long deltat=(t2 - lt); 
  
  if ( deltat> 500000) {
     // Compass_calibrate();
     //Print_Bpm();
  // Acc_gyr_fusion();
    /*
    Serial.println((int)RollRate_Input );
    Serial.println((int)PitchRate_Input );
    Serial.println((int)YawRate_Input );
    Serial.println(RollRate_Setpoint);
    Serial.println(PitchRate_Setpoint);
    Serial.println(YawRate_Setpoint);

  Serial.println('A');   // send a capital A
    Serial.println((long)micros());
    Serial.println(Roll_Input);
    Serial.println(Pitch_Input);
    Serial.println(Yaw_Input);
    Serial.println(Roll_Setpoint);
    Serial.println(Pitch_Setpoint);
    Serial.println(Yaw_Setpoint);
 
    */
      //Print_angle_fused();
       Serial.print("Yaw Set =");
        Serial.println(Yaw_Setpoint);
          Serial.print("Yaw in =" );
          Serial.println(Yaw_Input);
            Serial.print("Yaw out =" );
            Serial.println(Yaw_Output);
                 Serial.print("MAG=" );
            Serial.println(MAG_Heading);
      //Print_angle_acc();
      //Print_Acc_Offset();
   //Print_Gyr();
    //
  // Print_angle_gyro();
   // Print_angle_acc();
   // Print_Acc_Offset();
   // Print_Acc();
   //Serial.print(" Compass Heading = ");
//Serial.println(TO_DEG(MAG_Heading));
/* 
Serial.print(" Compass Heading = ");
Serial.println(TO_DEG(MAG_Heading));
       Serial.print( " acc Time= ");
    Serial.println( delta_acc_aux);
    */
//Print_angle_fused();
//Serial.print(" PITCH  SET");
//Serial.println(Pitch_Setpoint);
    /*
  Serial.print("Roll acc");Serial.println((float)Roll_angle);
 Serial.print("Roll gyr");Serial.println((float)gyro_angle[0]);
  Serial.println();
  Serial.print("Pitch acc");Serial.println((float)Pitch_angle);
 Serial.print("Pitch Gyr");Serial.println((float)gyro_angle[1]);

  Serial.println();
      Serial.print("PITCH INPUT");
    Serial.println(Pitch_Input);
         Serial.print("ROLL INPUT");
    Serial.println(Roll_Input);
  */
  /*
     Serial.println("");
    Serial.print(" Ofset X: "); Serial.print( mediax);
    Serial.print(" Ofset  Y: "); Serial.print( mediay);
    Serial.print(" Ofset  Z: "); Serial.print( mediaz);
    Serial.print(" Round X: "); Serial.print( -round(mediax/4));
    Serial.println("");
    */
    

  
  //Serial.println((float)YawRate_Setpoint);
 // Serial.println((float)YawRate_Input);
   /*

 */
 led_value=~led_value;
 digitalWrite(STATUS_LED_PIN, led_value);

    /*
    //Serial.println( ( Loop_Time - endtime) );
    Serial.print( " gyro Time= ");
    Serial.println( delta_gyro_aux);
        Serial.print( " acc Time= ");
    Serial.println( delta_acc_aux);
      Serial.print( " mag Time= ");
    Serial.println( delta_mag_aux);
  //  Serial.print( " PID Time= ");
   // Serial.println( PidTime );   
*/

    lt = t2;


  }
  #endif



endtime = micros();





}




void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
        pmc_set_writeprotect(false);
        pmc_enable_periph_clk((uint32_t)irq);
        TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
        uint32_t rc = VARIANT_MCK/128/frequency; //128 because we selected TIMER_CLOCK4 above
        TC_SetRA(tc, channel, rc/2); //50% high, 50% low
        TC_SetRC(tc, channel, rc);
        TC_Start(tc, channel);
        tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
        tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
        NVIC_EnableIRQ(irq);
}


void rxGoesHigh(){
 
  Ch6.Rising=micros();
  
  attachInterrupt(RX_INT_PIN,rxGoesLow,FALLING);
}
void rxGoesLow(){
/* As soon as we get into this function we measure the current time (with micros) and 
subtract the previous time (rxPrev). By doing that we will get the length of the pulse,
from start to the end. We end this process by re-attaching the throttle pin to a RISING 
interrupt, which will repeat the process again... */
  Ch6.T_alto=micros()-Ch6.Rising; 
  Ch6.Flag = 1; 
  attachInterrupt(RX_INT_PIN,rxGoesHigh,RISING);
  
}

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
