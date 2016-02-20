/* This sketch is an example use of the POWERSTEP01 library with one shield  X-NUCLEO-IHM01A1*/
/* It drives one stepper motor */

#include <powerstep01.h>
#include <SPI.h>

POWERSTEP01 PS01;
  
void setup()
{
	pinMode(13, OUTPUT);
	int32_t pos;
	uint16_t mySpeed;
	delay(5000);
	digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(1000);              // wait for a second
	digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
	delay(300);
	digitalWrite(13, HIGH);


//----- Init
  /* Start the library to use 1 shield */
  /* The POWERSTEP01 registers are set with the predefined values */
  /* from file POWERSTEP01_target_config.h*/
  PS01.Begin(1);

  /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
  PS01.AttachFlagInterrupt(MyFlagInterruptHandler);

//----- Move of 16000 steps in the FW direction

  /* Move shield 0 of 16000 steps in the FORWARD direction*/
  PS01.Move(0, FORWARD, 16000);
  
  /* Wait for the motor of shield 0 ends moving */
  PS01.WaitWhileActive(0);

  /* Wait for 2 seconds */
  delay(3000);

//----- Move of 16000 steps in the BW direction

  /* Move shield 0 of 16000 steps in the BACKWARD direction*/
  PS01.Move(0, BACKWARD, 16000);

  /* Wait for the motor of shield 0 ends moving */
  PS01.WaitWhileActive(0);

   /* Set the current position of shield 0 to be the Home position */
  PS01.SetHome(0);
  
  /* Wait for 2 seconds */
  delay(2000);

//----- Go to position -6400

  /* Request shield 0 to go to position -6400 */
  PS01.GoTo(0,-6400);  
  
  /* Wait for the motor ends moving */
  PS01.WaitWhileActive(0);

  /* Get current position of shield 0*/
  pos = PS01.GetPosition(0);

   /* Set the current position of shield 0 to be the Mark position */
  PS01.SetMark(0);

/* Wait for 2 seconds */
  delay(2000);
  
//----- Go Home

  /* Request shield 0 to go to Home */
  PS01.GoHome(0);  
  PS01.WaitWhileActive(0);

  /* Get current position of shield 0 */
  pos = PS01.GetPosition(0);
  
  /* Wait for 2 seconds */
  delay(2000);

//----- Go to position 6400

  /* Request shield 0 to go to position 6400 */
  PS01.GoTo(0,6400);  
  
  /* Wait for the motor of shield 0 ends moving */
  PS01.WaitWhileActive(0);

  /* Get current position of shield 0*/
  pos = PS01.GetPosition(0);

  /* Wait for 2 seconds */
  delay(2000);
  
//----- Go Mark which was set previously after go to -6400

  /* Request shield 0 to go to Mark position */
  PS01.GoMark(0);  
  
  /* Wait for the motor of shield 0 ends moving */
  PS01.WaitWhileActive(0);

  /* Get current position of shield 0 */
  pos = PS01.GetPosition(0);

  /* Wait for 2 seconds */
  delay(2000);

//----- Run the motor BACKWARD

  /* Request shield 0 to run BACKWARD */
   PS01.Run(0,BACKWARD);       
   delay(5000);

   /* Get current speed of shield 0 */
   mySpeed = PS01.GetCurrentSpeed(0);

//----- Increase the speed while running

  /* Increase speed of shield 0 to 2400 step/s */
  PS01.SetMaxSpeed(0,2400);
  delay(5000);

   /* Get current speed of shield 0 */
   mySpeed = PS01.GetCurrentSpeed(0);

//----- Decrease the speed while running

  /* Decrease speed of shield 0 to 1200 step/s */
  PS01.SetMaxSpeed(0,1200);
  delay(5000);

  /* Get current speed */
  mySpeed = PS01.GetCurrentSpeed(0);

//----- Increase acceleration while running

  /* Increase acceleration of shield 0 to 480 step/s^2 */
  PS01.SetAcceleration(0,480);
  delay(5000);

  /* Increase speed of shield 0 to 2400 step/s */
 PS01.SetMaxSpeed(0,2400);
 delay(5000);

 /* Get current speed of shield 0 */
 mySpeed = PS01.GetCurrentSpeed(0);

//----- Increase deceleration while running

  /* Increase deceleration of shield 0 to 480 step/s^2 */
  PS01.SetDeceleration(0,480);
  delay(5000);

  /* Decrease speed of shield 0 to 1200 step/s */
  PS01.SetMaxSpeed(0,1200);
  delay(5000);

  /* Get current speed */
  mySpeed = PS01.GetCurrentSpeed(0);

//----- Soft stopped required while running

  /* Request soft stop of shield 0 */
  PS01.SoftStop(0);

  /* Wait for the motor of shield 0 ends moving */  
  PS01.WaitWhileActive(0);

  /* Wait for 2 seconds */
  delay(2000);

//----- Run stopped by hardstop

  /* Request shield 0 to run in FORWARD direction */
  PS01.Run(0,FORWARD);       
  delay(5000);
  
  /* Request shield 0 to immediatly stop */
  PS01.HardStop(0);
  PS01.WaitWhileActive(0);

  /* Wait for 2 seconds */
  delay(2000);
  
//----- GOTO stopped by softstop

 /* Request shield 0 to go to position 20000  */
  PS01.GoTo(0,20000);  
  delay(5000);

  /* Request shield 0 to perform a soft stop */
  PS01.SoftStop(0);
  PS01.WaitWhileActive(0);

  /* Wait for 2 seconds */
  delay(2000);

//----- Read inexistent register to test MyFlagInterruptHandler

  /* Try to read an inexistent register */
  /* the flag interrupt should be raised */
  /* and the MyFlagInterruptHandler function called */
  PS01.CmdGetParam(0,(POWERSTEP01_Registers_t)0x1F);
  delay(500);

//----- Change step mode to full step mode

  /* Select full step mode for shield 0 */
  PS01.SelectStepMode(0,POWERSTEP01_STEP_SEL_1);

  /* Set speed and acceleration to be consistent with full step mode */
  PS01.SetMaxSpeed(0,100);
  PS01.SetMinSpeed(0,50);
  PS01.SetAcceleration(0,10);
  PS01.SetDeceleration(0,10);

  /* Request shield 0 to go position 200 */
  PS01.GoTo(0,200);  

  /* Wait for the motor of shield 0 ends moving */
  PS01.WaitWhileActive(0);

  /* Get current position */
  pos = PS01.GetPosition(0);

  /* Wait for 2 seconds */
  delay(2000);
  
//----- Restore 1/16 microstepping mode

  /* Reset shield 0 to 1/16 microstepping mode */
  PS01.SelectStepMode(0,POWERSTEP01_STEP_SEL_1_16);

  /* Update speed, acceleration, deceleration for 1/16 microstepping mode*/
  PS01.SetMaxSpeed(0,1600);
  PS01.SetMinSpeed(0,800);
  PS01.SetAcceleration(0,160);
  PS01.SetDeceleration(0,160);

}

void loop()
{
  /* Request shield 0 to go position -6400 */
  PS01.GoTo(0,-6400);

  /* Wait for the motor of shield 0 ends moving */
  PS01.WaitWhileActive(0);

  /* Request shield 0 to go position 6400 */
  PS01.GoTo(0,6400);

  /* Wait for the motor of shield 0 ends moving */
  PS01.WaitWhileActive(0);  
}

void MyFlagInterruptHandler(void)
{
  /* Get the value of the status register via the POWERSTEP01 command GET_STATUS */
  uint16_t statusRegister = PS01.CmdGetStatus(0);

  /* Check HIZ flag: if set, power brigdes are disabled */
  if ((statusRegister & POWERSTEP01_STATUS_HIZ) == POWERSTEP01_STATUS_HIZ)
  {
    // HIZ state
  }

  /* Check direction bit */
  if ((statusRegister & POWERSTEP01_STATUS_DIR) == POWERSTEP01_STATUS_DIR)
  {
    // Forward direction is set
  }  
  else
  {
    // Backward direction is set
  }  

  /* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
  /* This often occures when a command is sent to the POWERSTEP01 */
  /* while it is in HIZ state */
  if ((statusRegister & POWERSTEP01_STATUS_NOTPERF_CMD) == POWERSTEP01_STATUS_NOTPERF_CMD)
  {
       // Command received by SPI can't be performed
  }  

  /* Check WRONG_CMD flag: if set, the command does not exist */
  if ((statusRegister & POWERSTEP01_STATUS_WRONG_CMD) == POWERSTEP01_STATUS_WRONG_CMD)
  {
     //command received by SPI does not exist 
  }  

  /* Check UVLO flag: if not set, there is an undervoltage lock-out */
  if ((statusRegister & POWERSTEP01_STATUS_UVLO) == 0)
  {
     //undervoltage lock-out 
  }  

  /* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
  if ((statusRegister & POWERSTEP01_STATUS_TH_WRN) == 0)
  {
    //thermal warning threshold is reached
  }    

  /* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
  if ((statusRegister & POWERSTEP01_STATUS_TH_SD) == 0)
  {
    //thermal shut down threshold is reached * 
  }    

  /* Check OCD  flag: if not set, there is an overcurrent detection */
  if ((statusRegister & POWERSTEP01_STATUS_OCD) == 0)
  {
    //overcurrent detection 
  }      
}
