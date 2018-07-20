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
	 uint8_t SSP[]={53};
   	pinMode(53, OUTPUT);
  	digitalWrite(53, HIGH);


//----- Init
  /* Start the library to use 1 shield */
  /* The POWERSTEP01 registers are set with the predefined values */
  /* from file POWERSTEP01_target_config.h*/
  PS01.Begin(1,SSP);

  /* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
  PS01.AttachFlagInterrupt(MyFlagInterruptHandler);

  
  /* Wait for the motor of shield 0 ends moving */
  PS01.WaitWhileActive(0);

  /* Wait for 2 seconds */
  delay(3000);

//----- Move of 16000 steps in the BW direction



  /* Wait for the motor of shield 0 ends moving */
  PS01.WaitWhileActive(0);

   /* Set the current position of shield 0 to be the Home position */
  PS01.CmdSetHome(0);
  
  /* Wait for 2 seconds */
  delay(2000);

//----- Go to position -6400

  /* Request shield 0 to go to position -6400 */
  PS01.CmdGoTo(0,-6400);
  
  /* Wait for the motor ends moving */
  PS01.WaitWhileActive(0);

  /* Get current position of shield 0*/
  pos = PS01.GetPosition(0);

   /* Set the current position of shield 0 to be the Mark position */
  PS01.CmdSetMark(0);

/* Wait for 2 seconds */
  delay(2000);
  
//----- Go Home

  /* Request shield 0 to go to Home */
  PS01.CmdGoHome(0);
  PS01.WaitWhileActive(0);

  /* Get current position of shield 0 */
  pos = PS01.GetPosition(0);
  
  /* Wait for 2 seconds */
  delay(2000);

//----- Go to position 6400

  /* Request shield 0 to go to position 6400 */
  PS01.CmdGoTo(0,6400);
  
  /* Wait for the motor of shield 0 ends moving */
  PS01.WaitWhileActive(0);

  /* Get current position of shield 0*/
  pos = PS01.GetPosition(0);

  /* Wait for 2 seconds */
  delay(2000);
  
//----- Go Mark which was set previously after go to -6400

  /* Request shield 0 to go to Mark position */
  PS01.CmdGoMark(0);
  
  /* Wait for the motor of shield 0 ends moving */
  PS01.WaitWhileActive(0);

  /* Get current position of shield 0 */
  pos = PS01.GetPosition(0);

  /* Wait for 2 seconds */
  delay(2000);

//----- Soft stopped required while running

  /* Request soft stop of shield 0 */
  PS01.CmdSoftStop(0);

  /* Wait for the motor of shield 0 ends moving */  
  PS01.WaitWhileActive(0);

  /* Wait for 2 seconds */
  delay(2000);

//----- Run stopped by hardstop

  
  /* Request shield 0 to immediatly stop */
  PS01.CmdHardStop(0);
  PS01.WaitWhileActive(0);

  /* Wait for 2 seconds */
  delay(2000);
  
//----- GOTO stopped by softstop

 /* Request shield 0 to go to position 20000  */
  PS01.CmdGoTo(0,20000);
  delay(5000);

  /* Request shield 0 to perform a soft stop */
  PS01.CmdSoftStop(0);
  PS01.WaitWhileActive(0);

  /* Wait for 2 seconds */
  delay(2000);

//----- Read inexistent register to test MyFlagInterruptHandler

  /* Try to read an inexistent register */
  /* the flag interrupt should be raised */
  /* and the MyFlagInterruptHandler function called */
  PS01.CmdGetParam(0,(powerstep01_Registers_t)0x1F);
  delay(500);

//----- Change step mode to full step mode

  /* Select full step mode for shield 0 */
  PS01.SelectStepMode(0,STEP_MODE_FULL);


  /* Request shield 0 to go position 200 */
  PS01.CmdGoTo(0,200);

  /* Wait for the motor of shield 0 ends moving */
  PS01.WaitWhileActive(0);

  /* Get current position */
  pos = PS01.GetPosition(0);

  /* Wait for 2 seconds */
  delay(2000);
  
//----- Restore 1/16 microstepping mode

  /* Reset shield 0 to 1/16 microstepping mode */
  PS01.SelectStepMode(0,STEP_MODE_1_16);


}

void loop()
{
  /* Request shield 0 to go position -6400 */
  PS01.CmdGoTo(0,-6400);

  /* Wait for the motor of shield 0 ends moving */
  PS01.WaitWhileActive(0);

  /* Request shield 0 to go position 6400 */
  PS01.CmdGoTo(0,6400);

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



  /* Check UVLO flag: if not set, there is an undervoltage lock-out */
  if ((statusRegister & POWERSTEP01_STATUS_UVLO) == 0)
  {
     //undervoltage lock-out 
  }  


  /* Check OCD  flag: if not set, there is an overcurrent detection */
  if ((statusRegister & POWERSTEP01_STATUS_OCD) == 0)
  {
    //overcurrent detection 
  }      
}
