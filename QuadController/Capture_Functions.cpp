
#include <Arduino.h>
#include "Capture_Functions.h"






void TC6_Handler(void)
{
  //reads the interrupt. necessary to clear the interrupt flag.
  const uint32_t status=TC_GetStatus(TC2, 0);

 
  //input capture
  const bool inputcaptureA=status & TC_SR_LDRAS;
  const bool inputcaptureB=status & TC_SR_LDRBS;
  
  if(inputcaptureB) {
   Roll.Rising= TC2->TC_CHANNEL[0].TC_RB;
 //   Serial.print(" rasing=");
  //  Serial.print(rising);
  }
    //read LDRA. If we dont, we will get overflow (TC_SR_LOVR
    if(inputcaptureA) {
  Roll.Falling= TC2->TC_CHANNEL[0].TC_RA;
  //  Serial.print(" falling=");
   // Serial.print(falling);
  }  
  
  Roll.T_alto= (Roll.Falling-Roll.Rising)/42;
 // Roll.T_baixo = Roll.Rising -Roll.LastFalling;
  //Roll.Delta = Roll.Rising-Roll.LastRising;
/*
  Serial.print(" Alto =");
  Serial.print((double)alto*0.02380952380952);
   
  Serial.print(" Freq =");
  Serial.print((float)42000000/delta);
  Serial.println("");
  */
  //Roll.LastRising = Roll.Rising;
 // Roll.LastFalling=  Roll.Falling;
  Roll.Flag = 1;
}


  void TC0_Handler()
{
  //reads the interrupt. necessary to clear the interrupt flag.
  const uint32_t status=TC_GetStatus(TC0, 0);

 
  //input capture
  const bool inputcaptureA=status & TC_SR_LDRAS;
  const bool inputcaptureB=status & TC_SR_LDRBS;





  //read LDRA. If we dont, we will get overflow (TC_SR_LOVRS)
  if(inputcaptureB) {
   Pitch.Rising= TC0->TC_CHANNEL[0].TC_RB;
 //   Serial.print(" rasing=");
  //  Serial.print(rising);
  }
    if(inputcaptureA) {
  Pitch.Falling= TC0->TC_CHANNEL[0].TC_RA;
  //  Serial.print(" falling=");
   // Serial.print(falling);
  }  
  
  Pitch.T_alto= (Pitch.Falling-Pitch.Rising)/42;
  //Pitch.T_baixo = Pitch.Rising -Pitch.LastFalling;
  //Pitch.Delta = Pitch.Rising-Pitch.LastRising;
/*
  Serial.print(" Alto =");
  Serial.print((double)alto*0.02380952380952);
   
  Serial.print(" Freq =");
  Serial.print((float)42000000/delta);
  Serial.println("");
  */
 // Pitch.LastRising = Pitch.Rising;
  //Pitch.LastFalling=  Pitch.Falling;
  Pitch.Flag = 1;
}



void TC8_Handler()
{
  //reads the interrupt. necessary to clear the interrupt flag.
  const uint32_t status=TC_GetStatus(TC2, 2);

 
  //input capture
  const bool inputcaptureA=status & TC_SR_LDRAS;
  const bool inputcaptureB=status & TC_SR_LDRBS;



  //read LDRA. If we dont, we will get overflow (TC_SR_LOVRS)
  if(inputcaptureB) {
   Yaw.Rising= TC2->TC_CHANNEL[2].TC_RB;
 //   Serial.print(" rasing=");
  //  Serial.print(rising);
  }
  
    if(inputcaptureA) {
  Yaw.Falling= TC2->TC_CHANNEL[2].TC_RA;
  //  Serial.print(" falling=");
   // Serial.print(falling);
  }  
  
  Yaw.T_alto= (Yaw.Falling-Yaw.Rising)/42;
  //Yaw.T_baixo = Yaw.Rising -Yaw.LastFalling;
 // Yaw.Delta = Yaw.Rising-Yaw.LastRising;
/*
  Serial.print(" Alto =");
  Serial.print((double)alto*0.02380952380952);
   
  Serial.print(" Freq =");
  Serial.print((float)42000000/delta);
  Serial.println("");
  */
 // Yaw.LastRising = Yaw.Rising;
 // Yaw.LastFalling=  Yaw.Falling;
   Yaw.Flag = 1;
}


void TC7_Handler()
{
  //reads the interrupt. necessary to clear the interrupt flag.
  const uint32_t status=TC_GetStatus(TC2, 1);

 
  //input capture
  const bool inputcaptureA=status & TC_SR_LDRAS;
  const bool inputcaptureB=status & TC_SR_LDRBS;




  //read LDRA. If we dont, we will get overflow (TC_SR_LOVRS)
  if(inputcaptureA) {
   Throttle.Falling= TC2->TC_CHANNEL[1].TC_RA;
    //Serial.print(" ra1=");
    //Serial.print(falling);
  }
  //read LDRB. If we dont, we will get overflow (TC_SR_LOVRS)
  if(inputcaptureB) {
   Throttle.Rising= TC2->TC_CHANNEL[1].TC_RB;
   // Serial.print(" rising=");
  //  Serial.print(rising);
  }  


  Throttle.T_alto=  (Throttle.Falling-Throttle.Rising)/42;
  //Throttle.T_baixo = Throttle.Rising -Throttle.LastFalling;
  //Throttle.Delta = Throttle.Rising-Throttle.LastRising;
/*
  Serial.print(" Alto =");
  Serial.print((double)alto*0.02380952380952);
   
  Serial.print(" Freq =");
  Serial.print((float)42000000/delta);
  Serial.println("");
  */
//  Throttle.LastRising = Throttle.Rising;
 // Throttle.LastFalling=  Throttle.Falling;
   Throttle.Flag = 1;
}


void startTimer_TC6(Tc *tc, uint32_t channel, IRQn_Type irq) {

  //see 37.7.9
  REG_TC2_WPMR=0x54494D00;

  //enable configuring the io registers. see 32.7.42
  REG_PIOC_WPMR=0x50494F00;


  //we need to configure the pin to be controlled by the right peripheral.
  //pin 5 is port C. PIOC_PDR is defined in hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/instance/instance_pioc.h
  //and PIO_PDR_P25 is defined in hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/component/component_pio.h 
  //this disables the pio from controlling the pin. see 32.7.2

  REG_PIOC_PDR |= PIO_PDR_P25;

  //next thing is to assign the io line to the peripheral. See 32.7.24.
  //we need to know which peripheral we should use. Read table 37-4 in section 37.5.1.
  //TIOA6 is peripheral B, so we want to set that bit to 1.
  //REG_PIOC_ABSR is defined in hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/instance/instance_pioc.h
  //PIO_ABSR_P25 is defined in hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/component/component_pio.h
  REG_PIOC_ABSR |= PIO_ABSR_P25;


  //allow configuring the clock.
  pmc_set_writeprotect(false);

  /*
  Every peripheral in the SAM3X is off by default (to save power) and
  should be turned on. 
  */
    pmc_enable_periph_clk(ID_TC6);

  /*
  configure the timer. All this is about setting TC_CMRx, see 37.7.10 in atmel pdf.
  We use CLOCK1 at 42 MHz to get the best possible resolution.
  We want input capture on TIOA6 (pin 5). Nothing else should be necessary, BUT there is a caveat:
  As mentioned in 37.6.8, we only get the value loaded in RA if not loaded since the last trigger,
  or RB has been loaded. Since I do not want to trigger as that sets the timer value to 0, I
  instead let register B be loaded when the pulse is going low.
  */
  TC_Configure(tc, channel,  TC_CMR_TCCLKS_TIMER_CLOCK1 |TC_CMR_LDRA_FALLING |TC_CMR_LDRB_RISING     );



  //set the interrupt flags. We want interrupt on overflow and TIOA6 (pin 5) going high.
  const uint32_t flags=TC_IER_COVFS  | TC_IER_LDRAS;
  tc->TC_CHANNEL[channel].TC_IER=flags;
  tc->TC_CHANNEL[channel].TC_IDR=~flags;//assume IER and IDR are equally defined.

  NVIC_EnableIRQ(irq);

  //read away the status.
  // TC_GetStatus(tc, channel);

  //start the timer
  TC_Start(tc,channel);
}
void startTimer_TC7(Tc *tc, uint32_t channel, IRQn_Type irq) {

  //see 37.7.9
  REG_TC2_WPMR=0x54494D00;

  //enable configuring the io registers. see 32.7.42
  REG_PIOC_WPMR=0x50494F00;


  //we need to configure the pin to be controlled by the right peripheral.
  //pin 5 is port C. PIOC_PDR is defined in hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/instance/instance_pioc.h
  //and PIO_PDR_P25 is defined in hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/component/component_pio.h 
  //this disables the pio from controlling the pin. see 32.7.2

 // REG_PIOC_PDR |= PIO_PDR_P25;

  //next thing is to assign the io line to the peripheral. See 32.7.24.
  //we need to know which peripheral we should use. Read table 37-4 in section 37.5.1.
  //TIOA6 is peripheral B, so we want to set that bit to 1.
  //REG_PIOC_ABSR is defined in hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/instance/instance_pioc.h
  //PIO_ABSR_P25 is defined in hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/component/component_pio.h
 // REG_PIOC_ABSR |= PIO_ABSR_P25;

REG_PIOC_PDR |= PIO_PDR_P29;
REG_PIOC_ABSR |= PIO_ABSR_P29;
  //allow configuring the clock.
  pmc_set_writeprotect(false);

  /*
  Every peripheral in the SAM3X is off by default (to save power) and
  should be turned on. 
  */
    pmc_enable_periph_clk(ID_TC7);

  /*
  configure the timer. All this is about setting TC_CMRx, see 37.7.10 in atmel pdf.
  We use CLOCK1 at 42 MHz to get the best possible resolution.
  We want input capture on TIOA6 (pin 5). Nothing else should be necessary, BUT there is a caveat:
  As mentioned in 37.6.8, we only get the value loaded in RA if not loaded since the last trigger,
  or RB has been loaded. Since I do not want to trigger as that sets the timer value to 0, I
  instead let register B be loaded when the pulse is going low.
  */
  TC_Configure(tc, channel,  TC_CMR_TCCLKS_TIMER_CLOCK1 |TC_CMR_LDRA_FALLING   |TC_CMR_LDRB_RISING    );



  //set the interrupt flags. We want interrupt on overflow and TIOA6 (pin 5) going high.
  const uint32_t flags=TC_IER_COVFS  | TC_IER_LDRAS;
  tc->TC_CHANNEL[channel].TC_IER=flags;
  tc->TC_CHANNEL[channel].TC_IDR=~flags;//assume IER and IDR are equally defined.

  NVIC_EnableIRQ(irq);

  //read away the status.
  // TC_GetStatus(tc, channel);

  //start the timer
  TC_Start(tc,channel);
}

void startTimer_TC8(Tc *tc, uint32_t channel, IRQn_Type irq) {

  //see 37.7.9
  REG_TC2_WPMR=0x54494D00;

  //enable configuring the io registers. see 32.7.42
  REG_PIOD_WPMR=0x50494F00;


  //we need to configure the pin to be controlled by the right peripheral.
  //pin 5 is port C. PIOC_PDR is defined in hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/instance/instance_pioc.h
  //and PIO_PDR_P25 is defined in hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/component/component_pio.h 
  //this disables the pio from controlling the pin. see 32.7.2

 // REG_PIOC_PDR |= PIO_PDR_P25;

  //next thing is to assign the io line to the peripheral. See 32.7.24.
  //we need to know which peripheral we should use. Read table 37-4 in section 37.5.1.
  //TIOA6 is peripheral B, so we want to set that bit to 1.
  //REG_PIOC_ABSR is defined in hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/instance/instance_pioc.h
  //PIO_ABSR_P25 is defined in hardware/arduino/sam/system/CMSIS/Device/ATMEL/sam3xa/include/component/component_pio.h
 // REG_PIOC_ABSR |= PIO_ABSR_P25;


    REG_PIOD_PER |= PIO_PDR_P7;
    REG_PIOD_ABSR |= PIO_ABSR_P7;
  //allow configuring the clock.
  pmc_set_writeprotect(false);

  /*
  Every peripheral in the SAM3X is off by default (to save power) and
  should be turned on. 
  */
    pmc_enable_periph_clk(ID_TC8);

  /*
  configure the timer. All this is about setting TC_CMRx, see 37.7.10 in atmel pdf.
  We use CLOCK1 at 42 MHz to get the best possible resolution.
  We want input capture on TIOA6 (pin 5). Nothing else should be necessary, BUT there is a caveat:
  As mentioned in 37.6.8, we only get the value loaded in RA if not loaded since the last trigger,
  or RB has been loaded. Since I do not want to trigger as that sets the timer value to 0, I
  instead let register B be loaded when the pulse is going low.
  */
  TC_Configure(tc, channel,  TC_CMR_TCCLKS_TIMER_CLOCK1 |TC_CMR_LDRA_FALLING   |TC_CMR_LDRB_RISING   );



  //set the interrupt flags. We want interrupt on overflow and TIOA6 (pin 5) going high.
  const uint32_t flags=TC_IER_COVFS  | TC_IER_LDRAS;
  tc->TC_CHANNEL[channel].TC_IER=flags;
  tc->TC_CHANNEL[channel].TC_IDR=~flags;//assume IER and IDR are equally defined.

  NVIC_EnableIRQ(irq);

  //read away the status.
   //TC_GetStatus(tc, channel);

  //start the timer
  TC_Start(tc,channel);
}



 void startTimer_TC0(Tc *tc, uint32_t channel, IRQn_Type irq) {

  //see 37.7.9
  REG_TC0_WPMR=0x54494D00;

  //enable configuring the io registers. see 32.7.42
  REG_PIOB_WPMR=0x50494F00;

  REG_PIOB_PER |= PIO_PDR_P25; //PINO 5
  REG_PIOB_ABSR |= PIO_ABSR_P25; //PINO 5
  
  //allow configuring the clock.
  pmc_set_writeprotect(false);

  /*
  Every peripheral in the SAM3X is off by default (to save power) and
  should be turned on. 
  */
    pmc_enable_periph_clk(ID_TC0);

  /*
  configure the timer. All this is about setting TC_CMRx, see 37.7.10 in atmel pdf.
  We use CLOCK1 at 42 MHz to get the best possible resolution.
  We want input capture on TIOA6 (pin 5). Nothing else should be necessary, BUT there is a caveat:
  As mentioned in 37.6.8, we only get the value loaded in RA if not loaded since the last trigger,
  or RB has been loaded. Since I do not want to trigger as that sets the timer value to 0, I
  instead let register B be loaded when the pulse is going low.
  */
  TC_Configure(tc, channel,  TC_CMR_TCCLKS_TIMER_CLOCK1 |TC_CMR_LDRA_FALLING   |TC_CMR_LDRB_RISING   );



  //set the interrupt flags. We want interrupt on overflow and TIOA6 (pin 5) going high.
  const uint32_t flags=TC_IER_COVFS  | TC_IER_LDRAS;
  tc->TC_CHANNEL[channel].TC_IER=flags;
  tc->TC_CHANNEL[channel].TC_IDR=~flags;//assume IER and IDR are equally defined.

  NVIC_EnableIRQ(irq);

  //read away the status.
   //TC_GetStatus(tc, channel);

  //start the timer
  TC_Start(tc,channel);
}


