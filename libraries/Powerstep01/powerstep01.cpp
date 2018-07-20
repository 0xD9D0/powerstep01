/******************************************************//**
 * @file    POWERSTEP01.cpp
 * @version V1.0
 * @date    March 3, 2014
 * @brief   POWERSTEP01 library for arduino
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 **********************************************************/ 

#include <powerstep01.h>
#include <SPI.h>

#ifdef _DEBUG_POWERSTEP01
/// Log buffer
char POWERSTEP01StrOut[DEBUG_BUFFER_SIZE];
#endif


/** @addtogroup BSP
  * @{
  */

/** @addtogroup POWERSTEP01
  * @{
  */

/* Private constants ---------------------------------------------------------*/

/** @addtogroup Powerstep01_Private_Constants
  * @{
  */
/// Error while initialising the SPI
#define POWERSTEP01_ERROR_0   (0x8000)
/// Error: Bad SPI transaction
#define POWERSTEP01_ERROR_1   (0x8001)

/**
  * @}
  */


/// Function pointer to flag interrupt call back
volatile void (*POWERSTEP01::flagInterruptCallback)(void);
/// Function pointer to busy interrupt call back
volatile void (*POWERSTEP01::busyInterruptCallback)(void);
volatile uint8_t POWERSTEP01::numberOfDevices;
static volatile uint8_t POWERSTEP01::_SSPins[MAX_NUMBER_OF_DEVICES];
uint8_t POWERSTEP01::spiTxBursts[POWERSTEP01_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
uint8_t POWERSTEP01::spiRxBursts[POWERSTEP01_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
volatile bool POWERSTEP01::spiPreemtionByIsr = false;
volatile bool POWERSTEP01::isrFlag = false;
volatile class POWERSTEP01* POWERSTEP01::instancePtr = NULL;
  
/******************************************************//**
 * @brief  Constructor
 * @param  None
 * @retval None
 **********************************************************/ 
POWERSTEP01::POWERSTEP01()
{
    instancePtr = this;
}

/******************************************************//**
 * @brief  Attaches a user callback to the flag Interrupt
 * The call back will be then called each time the status 
 * flag pin will be pulled down due to the occurrence of 
 * a programmed alarms ( OCD, thermal pre-warning or 
 * shutdown, UVLO, wrong command, non-performable command)
 * @param[in] callback Name of the callback to attach 
 * to the Flag Interrupt
 * @retval None
 **********************************************************/
void POWERSTEP01::AttachFlagInterrupt(void (*callback)(void))
{
  flagInterruptCallback = (volatile void (*)())callback;
}

/******************************************************//**
 * @brief Starts the POWERSTEP01 library
 * @param[in] nbDEVICEs Number of POWERSTEP01 DEVICEs to use (from 1 to 3)
 * @retval None
 **********************************************************/
void POWERSTEP01::Begin(uint8_t nbDEVICEs, uint8_t SSPin[])
{
  numberOfDevices = nbDEVICEs;

  // start the SPI library:
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  
  // flag pin
  pinMode(POWERSTEP01_FLAG_Pin, INPUT_PULLUP);
  attachInterrupt(0, FlagInterruptHandler, FALLING);
  
  //reset pin
  pinMode(POWERSTEP01_Reset_Pin, OUTPUT);
  
  
  /* Standby-reset deactivation */
  ReleaseReset();
  
  
  /* Disable POWERSTEP01 powerstage */
  for (uint32_t i = 0; i < nbDEVICEs; i++)
  {

    /* Get Status to clear flags after start up */
    CmdGetStatus(i);
	_SSPins[i] = SSPin[i];
  }
}



/******************************************************//**
 * @brief Returns the FW version of the library
 * @param None
 * @retval POWERSTEP01_FW_VERSION
 **********************************************************/
uint8_t POWERSTEP01::GetFwVersion(void)
{
  return (POWERSTEP01_FW_VERSION);
}

/******************************************************//**
 * @brief  Returns the mark position  of the specified DEVICE
 * @param[in] DEVICEId (from 0 to 2)
 * @retval Mark register value converted in a 32b signed integer 
 **********************************************************/
int32_t POWERSTEP01::GetMark(uint8_t DEVICEId)
{
  return ConvertPosition(CmdGetParam(DEVICEId,POWERSTEP01_MARK));
}



/******************************************************//**
 * @brief  Returns the ABS_POSITION of the specified DEVICE
 * @param[in] DEVICEId (from 0 to 2)
 * @retval ABS_POSITION register value converted in a 32b signed integer
 **********************************************************/
int32_t POWERSTEP01::GetPosition(uint8_t DEVICEId)
{
  return ConvertPosition(CmdGetParam(DEVICEId,POWERSTEP01_ABS_POS));
}


/******************************************************//**
 * @brief  Requests the motor to move to the home position (ABS_POSITION = 0)
 * @param[in] DEVICEId (from 0 to 2)
 * @retval None
 **********************************************************/
void POWERSTEP01::CmdGoHome(uint8_t deviceId)
{
	   SendCommand(deviceId, POWERSTEP01_GO_HOME, 0);
} 
  
/******************************************************//**
 * @brief  Requests the motor to move to the mark position 
 * @param[in] DEVICEId (from 0 to 2)
 * @retval None
 **********************************************************/
void POWERSTEP01::CmdGoMark(uint8_t deviceId)
{
  SendCommand(deviceId, POWERSTEP01_GO_MARK, 0);
}

/******************************************************//**
 * @brief  Requests the motor to move to the specified position 
 * @param[in] DEVICEId (from 0 to 2)
 * @param[in] targetPosition absolute position in steps
 * @retval None
 **********************************************************/
void POWERSTEP01::CmdGoTo(uint8_t deviceId, int32_t abs_pos)
{
  SendCommand(deviceId, POWERSTEP01_GO_TO, abs_pos);
}

/******************************************************//**
 * @brief  Immediatly stops the motor and disable the power bridge
 * @param[in] DEVICEId (from 0 to 2)
 * @retval None
 **********************************************************/
void POWERSTEP01::CmdHardStop(uint8_t deviceId)
{
	SendCommand(deviceId, POWERSTEP01_HARD_STOP, 0);

}

/******************************************************//**
 * @brief  Moves the motor of the specified number of steps
 * @param[in] DEVICEId (from 0 to 2)
 * @param[in] direction FORWARD or BACKWARD
 * @param[in] stepCount Number of steps to perform
 * @retval None
 **********************************************************/
void POWERSTEP01::CmdMove(uint8_t deviceId, motorDir_t direction, uint32_t n_step)
{
  SendCommand(deviceId,
                          (uint8_t)POWERSTEP01_MOVE |
                          (uint8_t)direction, n_step);
}


/******************************************************//**
 * @brief Issues PowerStep01 Run command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] direction Movement direction (FORWARD, BACKWARD)
 * @param[in] speed in steps/s
 * @retval None
 *********************************************************/
void POWERSTEP01::CmdRun(uint8_t deviceId, motorDir_t direction, uint32_t speed)
{
  SendCommand(deviceId, (uint8_t)POWERSTEP01_RUN | (uint8_t)direction, speed);
}




/******************************************************//**
 * @brief  Set current position to be the Home position (ABS pos set to 0)
 * @param[in] DEVICEId (from 0 to 2)
 * @retval None
 **********************************************************/
void POWERSTEP01::CmdSetHome(uint8_t DEVICEId)
{
  CmdSetParam(DEVICEId, POWERSTEP01_ABS_POS, 0);
}
 
/******************************************************//**
 * @brief  Sets current position to be the Mark position 
 * @param[in] DEVICEId (from 0 to 2)
 * @retval None
 **********************************************************/
void POWERSTEP01::CmdSetMark(uint8_t DEVICEId)
{
  uint32_t mark = CmdGetParam(DEVICEId,POWERSTEP01_ABS_POS);
  CmdSetParam(DEVICEId,POWERSTEP01_MARK, mark);
}



/******************************************************//**
 * @brief Issues PowerStep01 Soft Stop command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void POWERSTEP01::CmdSoftStop(uint8_t deviceId)
{
  SendCommand(deviceId, POWERSTEP01_SOFT_STOP, 0);
}

/******************************************************//**
 * @brief  Locks until the DEVICE state becomes Inactive
 * @param[in] DEVICEId (from 0 to 2)
 * @retval None
 **********************************************************/
void POWERSTEP01::WaitWhileActive(uint8_t deviceId)
{
	/* Wait while motor is running */
	while (IsDeviceBusy(deviceId) != 0);
}

/******************************************************//**
 * @brief Checks if the specified device is busy
 * by reading the Busy flag bit ot its status Register
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval true if device is busy, false zero
 *********************************************************/
bool POWERSTEP01::IsDeviceBusy(uint8_t deviceId)
{
  if(!(CmdGetStatus(deviceId) & POWERSTEP01_STATUS_BUSY))
  {
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

/******************************************************//**
 * @brief  Issues the GetParam command to the POWERSTEP01 of the specified DEVICE
 * @param[in] DEVICEId (from 0 to 2)
 * @param[in] param Register adress (POWERSTEP01_ABS_POS, POWERSTEP01_MARK,...)
 * @retval Register value
 **********************************************************/
uint32_t POWERSTEP01::CmdGetParam(uint8_t DEVICEId, powerstep01_Registers_t param)
{
  uint32_t i;
  uint32_t spiRxData;
  uint8_t maxArgumentNbBytes = 0;
  uint8_t spiIndex = numberOfDevices - DEVICEId - 1;
  bool itDisable = false;  
  
  do
  {
    spiPreemtionByIsr = false;
    if (itDisable)
    {
      /* re-enable interrupts if disable in previous iteration */
      interrupts();
      itDisable = false;
    }
  
    for (i = 0; i < numberOfDevices; i++)
    {
      spiTxBursts[0][i] = POWERSTEP01_NOP;
      spiTxBursts[1][i] = POWERSTEP01_NOP;
      spiTxBursts[2][i] = POWERSTEP01_NOP;
      spiTxBursts[3][i] = POWERSTEP01_NOP;
      spiRxBursts[1][i] = 0;
      spiRxBursts[2][i] = 0;
      spiRxBursts[3][i] = 0;    
    }
    switch (param)
    {
      case POWERSTEP01_ABS_POS: ;
      case POWERSTEP01_MARK:
        spiTxBursts[0][spiIndex] = ((uint8_t)POWERSTEP01_GET_PARAM )| (param);
        maxArgumentNbBytes = 3;
        break;
      case POWERSTEP01_EL_POS: ;
      case POWERSTEP01_CONFIG: ;
      case POWERSTEP01_STATUS:
        spiTxBursts[1][spiIndex] = ((uint8_t)POWERSTEP01_GET_PARAM )| (param);
        maxArgumentNbBytes = 2;
        break;
      default:
        spiTxBursts[2][spiIndex] = ((uint8_t)POWERSTEP01_GET_PARAM )| (param);
        maxArgumentNbBytes = 1;
    }
    
    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    noInterrupts();
    itDisable = true;
  } while (spiPreemtionByIsr); // check pre-emption by ISR
    
  for (i = POWERSTEP01_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes;
       i < POWERSTEP01_CMD_ARG_MAX_NB_BYTES;
       i++)
  {
     WriteBytes(&spiTxBursts[i][0],
                          &spiRxBursts[i][0],DEVICEId);
  }
  
  spiRxData = ((uint32_t)spiRxBursts[1][spiIndex] << 16)|
              (spiRxBursts[2][spiIndex] << 8) |
              (spiRxBursts[3][spiIndex]);
  
  /* re-enable interrupts after SPI transfers*/
  interrupts();
    
  return (spiRxData);
}

/******************************************************//**
 * @brief  Issues the GetStatus command to the POWERSTEP01 of the specified DEVICE
 * @param[in] DEVICEId (from 0 to 2)
 * @retval Status Register value
 * @note Once the GetStatus command is performed, the flags of the status register
 * are reset. This is not the case when the status register is read with the
 * GetParam command (via the functions ReadStatusRegister or CmdGetParam).
 **********************************************************/
uint16_t POWERSTEP01::CmdGetStatus(uint8_t DEVICEId)
{
  uint32_t i;
  uint16_t status;
  uint8_t spiIndex = numberOfDevices - DEVICEId - 1;
  bool itDisable = false;  
  
  do
  {
    spiPreemtionByIsr = false;
    if (itDisable)
    {
      /* re-enable interrupts if disable in previous iteration */
      interrupts();
      itDisable = false;
    }

    for (i = 0; i < numberOfDevices; i++)
    {
       spiTxBursts[0][i] = POWERSTEP01_NOP;
       spiTxBursts[1][i] = POWERSTEP01_NOP;
       spiTxBursts[2][i] = POWERSTEP01_NOP;
       spiRxBursts[1][i] = 0;
       spiRxBursts[2][i] = 0;
    }
    spiTxBursts[0][spiIndex] = POWERSTEP01_GET_STATUS;

    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    noInterrupts();
    itDisable = true;
  } while (spiPreemtionByIsr); // check pre-emption by ISR

  for (i = 0; i < POWERSTEP01_CMD_ARG_NB_BYTES_GET_STATUS + POWERSTEP01_RSP_NB_BYTES_GET_STATUS; i++)
  {
     WriteBytes(&spiTxBursts[i][0], &spiRxBursts[i][0],DEVICEId);
  }
  status = (spiRxBursts[1][spiIndex] << 8) | (spiRxBursts[2][spiIndex]);
  
  /* re-enable interrupts after SPI transfers*/
  interrupts();
  
  return (status);
}

/******************************************************//**
 * @brief  Issues the Nop command to the POWERSTEP01 of the specified DEVICE
 * @param[in] DEVICEId (from 0 to 2)
 * @retval None
 **********************************************************/
void POWERSTEP01::CmdNop(uint8_t DEVICEId)
{
  SendCommand(DEVICEId, POWERSTEP01_NOP);
}

/******************************************************//**
 * @brief  Issues the SetParam command to the POWERSTEP01 of the specified DEVICE
 * @param[in] DEVICEId (from 0 to 2)
 * @param[in] param Register adress (POWERSTEP01_ABS_POS, POWERSTEP01_MARK,...)
 * @param[in] value Value to set in the register
 * @retval None
 **********************************************************/
void POWERSTEP01::CmdSetParam(uint8_t DEVICEId,
                        uint32_t param,
                        uint32_t value)
{
  uint32_t i;
  uint8_t maxArgumentNbBytes = 0;
  uint8_t spiIndex = numberOfDevices - DEVICEId - 1;
  bool itDisable = false;  
  do
  {
    spiPreemtionByIsr = false;
    if (itDisable)
    {
      /* re-enable interrupts if disable in previous iteration */
      interrupts();
      itDisable = false;
    }
    for (i = 0; i < numberOfDevices; i++)
    {
      spiTxBursts[0][i] = POWERSTEP01_NOP;
      spiTxBursts[1][i] = POWERSTEP01_NOP;
      spiTxBursts[2][i] = POWERSTEP01_NOP;
      spiTxBursts[3][i] = POWERSTEP01_NOP;
    }
    switch (param)
  {
    case POWERSTEP01_ABS_POS: ;
    case POWERSTEP01_MARK:
        spiTxBursts[0][spiIndex] = ((uint8_t)POWERSTEP01_SET_PARAM )| (param);
        spiTxBursts[1][spiIndex] = (uint8_t)(value >> 16);
        spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
        maxArgumentNbBytes = 3;
        break;
    case POWERSTEP01_EL_POS:
    case POWERSTEP01_ACC:
    case POWERSTEP01_DEC:
    case POWERSTEP01_MAX_SPEED:
    case POWERSTEP01_MIN_SPEED:
    case POWERSTEP01_FS_SPD:
    case POWERSTEP01_INT_SPD:
    case POWERSTEP01_CONFIG:
    case POWERSTEP01_GATECFG1:
       spiTxBursts[1][spiIndex] = ((uint8_t)POWERSTEP01_SET_PARAM )| (param);
       spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
       maxArgumentNbBytes = 2;
       break;
    default:
        spiTxBursts[2][spiIndex] = ((uint8_t)POWERSTEP01_SET_PARAM )| (param);
        maxArgumentNbBytes = 1;
    }
    spiTxBursts[3][spiIndex] = (uint8_t)(value);
    
    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    noInterrupts();
    itDisable = true;
  } while (spiPreemtionByIsr); // check pre-emption by ISR
 
  /* SPI transfer */
  for (i = POWERSTEP01_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes;
       i < POWERSTEP01_CMD_ARG_MAX_NB_BYTES;
       i++)
  {
     WriteBytes(&spiTxBursts[i][0],&spiRxBursts[i][0],DEVICEId);
  }
  /* re-enable interrupts after SPI transfers*/
  interrupts();
}

/******************************************************//**
 * @brief  Reads the Status Register value
 * @param[in] DEVICEId (from 0 to 2)
 * @retval Status register valued
 * @note The status register flags are not cleared 
 * at the difference with CmdGetStatus()
 **********************************************************/
uint16_t POWERSTEP01::ReadStatusRegister(uint8_t DEVICEId)
{
  return (CmdGetParam(DEVICEId,POWERSTEP01_STATUS));
}

/******************************************************//**
 * @brief  Releases the POWERSTEP01 reset (pin set to High) of all DEVICEs
 * @param  None
 * @retval None
 **********************************************************/
void POWERSTEP01::ReleaseReset(void)
{ 
  digitalWrite(POWERSTEP01_Reset_Pin, HIGH);
}

/******************************************************//**
 * @brief  Resets the POWERSTEP01 (reset pin set to low) of all DEVICEs
 * @param  None
 * @retval None
 **********************************************************/
void POWERSTEP01::Reset(void)
{
  digitalWrite(POWERSTEP01_Reset_Pin, LOW);
}

/******************************************************//**
 * @brief  Set the stepping mode 
 * @param[in] DEVICEId (from 0 to 2)
 * @param[in] stepMod from full step to 1/16 microstep as specified in enum POWERSTEP01_STEP_SEL_t
 * @retval None
 **********************************************************/
void POWERSTEP01::SelectStepMode(uint8_t deviceId, motorStepMode_t stepMode)
{
  uint8_t stepModeRegister;
  powerstep01_StepSel_t powerstep01StepMode;
  
  switch (stepMode)
  {
    case STEP_MODE_FULL:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1;
      break;
    case STEP_MODE_HALF:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_2;
      break;
    case STEP_MODE_1_4:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_4;
      break;
    case STEP_MODE_1_8:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_8;
      break;
    case STEP_MODE_1_16:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_16;
      break;
    case STEP_MODE_1_32:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_32;
      break;
    case STEP_MODE_1_64:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_64;
      break;
    case STEP_MODE_1_128:
    default:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_128;
      break;
  }
  
  /* Set the powerstep01 in HiZ state */
  CmdHardHiZ(deviceId);

  /* Read Step mode register and clear STEP_SEL field */
  stepModeRegister = (uint8_t)(0xF8 & CmdGetParam(deviceId,POWERSTEP01_STEP_MODE)) ;
  
  /* Apply new step mode */
  CmdSetParam(deviceId, POWERSTEP01_STEP_MODE, stepModeRegister | (uint8_t)powerstep01StepMode);

  /* Reset abs pos register */
  CmdResetPos(deviceId);
}

/******************************************************//**
 * @brief Issues PowerStep01 Hard HiZ command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void POWERSTEP01::CmdHardHiZ(uint8_t deviceId)
{
  SendCommand(deviceId, POWERSTEP01_HARD_HIZ, 0);
}
// free is an alias for CmdHardHiz
void POWERSTEP01::free(uint8_t deviceId)
{
SendCommand(deviceId, POWERSTEP01_HARD_HIZ, 0);
}

/******************************************************//**
 * @brief Issues PowerStep01 Reset Pos command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void POWERSTEP01::CmdResetPos(uint8_t deviceId)
{
  SendCommand(deviceId, POWERSTEP01_RESET_POS, 0);
}
/******************************************************//**
 * @brief  Waits for the specify delay in milliseconds
 * @param[in] msDelay delay in milliseconds
 * @retval None
 * @note Should only be used for 3 DEVICEs configuration.
 * Else, prefer the standard Arduino function delay().
 **********************************************************/

void POWERSTEP01::WaitMs(uint16_t msDelay)
{
  uint16_t i;
  for (i = 0; i < msDelay ; i++)
  {
    WaitUs(1000);
  }
}

/******************************************************//**
 * @brief  Waits for the specify delay in microseconds
 * @param[in] usDelay delay in microseconds
 * @retval None
 * @note Should be only used for 3 DEVICEs configuration.
 * Else, prefer the standard Arduino function delayMicroseconds().
 * Besides, this function is a copy of delayMicroseconds inside
 * the POWERSTEP01 library to avoid dependencies conflicts
 * (a redefinition of ISR(TIMER0_OVF_vect)). 
 **********************************************************/
void POWERSTEP01::WaitUs(uint16_t usDelay)
{
	// calling avrlib's delay_us() function with low values (e.g. 1 or
	// 2 microseconds) gives delays longer than desired.
	//delay_us(us);
#if F_CPU >= 20000000L
	// for the 20 MHz clock on rare Arduino boards

	// for a one-microsecond delay, simply wait 2 cycle and return. The overhead
	// of the function call yields a delay of exactly a one microsecond.
	__asm__ __volatile__ (
		"nop" "\n\t"
		"nop"); //just waiting 2 cycle
	if (--usDelay == 0)
		return;

	// the following loop takes a 1/5 of a microsecond (4 cycles)
	// per iteration, so execute it five times for each microsecond of
	// delay requested.
	usDelay = (usDelay<<2) + usDelay; // x5 us

	// account for the time taken in the preceeding commands.
	usDelay -= 2;

#elif F_CPU >= 16000000L
	// for the 16 MHz clock on most Arduino boards

	// for a one-microsecond delay, simply return.  the overhead
	// of the function call yields a delay of approximately 1 1/8 us.
	if (--usDelay == 0)
		return;

	// the following loop takes a quarter of a microsecond (4 cycles)
	// per iteration, so execute it four times for each microsecond of
	// delay requested.
	usDelay <<= 2;

	// account for the time taken in the preceeding commands.
	usDelay -= 2;
#else
	// for the 8 MHz internal clock on the ATmega168

	// for a one- or two-microsecond delay, simply return.  the overhead of
	// the function calls takes more than two microseconds.  can't just
	// subtract two, since us is unsigned; we'd overflow.
	if (--usDelay == 0)
		return;
	if (--usDelay == 0)
		return;

	// the following loop takes half of a microsecond (4 cycles)
	// per iteration, so execute it twice for each microsecond of
	// delay requested.
	usDelay <<= 1;
    
	// partially compensate for the time taken by the preceeding commands.
	// we can't subtract any more than this or we'd overflow w/ small delays.
	usDelay--;
#endif

	// busy wait
	__asm__ __volatile__ (
		"1: sbiw %0,1" "\n\t" // 2 cycles
		"brne 1b" : "=w" (usDelay) : "0" (usDelay) // 2 cycles
	);
}  
                  
/******************************************************//**
 * @brief  Gets the pointer to the POWERSTEP01 instance
 * @param  None
 * @retval Pointer to the instance of POWERSTEP01
  **********************************************************/
class POWERSTEP01* POWERSTEP01::GetInstancePtr(void)
{
  return (class POWERSTEP01*)instancePtr;
}




/******************************************************//**
 * @brief  Converts the ABS_POSITION register value to a 32b signed integer
 * @param[in] abs_position_reg value of the ABS_POSITION register
 * @retval operation_result 32b signed integer corresponding to the absolute position 
 **********************************************************/
int32_t POWERSTEP01::ConvertPosition(uint32_t abs_position_reg)
{
	int32_t operation_result;

  if (abs_position_reg & POWERSTEP01_ABS_POS_SIGN_BIT_MASK)
  {
		/* Negative register value */
		abs_position_reg = ~abs_position_reg;
		abs_position_reg += 1;

		operation_result = (int32_t) (abs_position_reg & POWERSTEP01_ABS_POS_VALUE_MASK);
		operation_result = -operation_result;
  } 
  else 
  {
		operation_result = (int32_t) abs_position_reg;
	}
	return operation_result;
}

/******************************************************//**
 * @brief  Handlers of the flag interrupt which calls the user callback (if defined)
 * @param None
 * @retval None
 **********************************************************/
void POWERSTEP01::FlagInterruptHandler(void)
{
  if (flagInterruptCallback != NULL)
  {
    /* Set isr flag */
    isrFlag = true;
    
    flagInterruptCallback();
    
    /* Reset isr flag */
    isrFlag = false;   
  }
}



/******************************************************//**
 * @brief  Sends a command to a given device Id via the SPI
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] param Command to send (all Powerstep01 commmands
 * except POWERSTEP01_SET_PARAM, POWERSTEP01_GET_PARAM,
 * POWERSTEP01_GET_STATUS)
 * @param[in] value arguments to send on 32 bits
 * @retval None
 **********************************************************/
void POWERSTEP01::SendCommand(uint8_t deviceId, uint8_t param, uint32_t value)
{
  if (numberOfDevices > deviceId)
  {
    uint32_t loop;
    uint8_t maxArgumentNbBytes = 0;
    uint8_t spiIndex = numberOfDevices - deviceId - 1;
    bool itDisable = false;
    do
    {
        spiPreemtionByIsr = false;
        if (itDisable)
        {
          /* re-enable interrupts if disable in previous iteration */
          interrupts();
          itDisable = false;
        }

  
    for (loop = 0; loop < numberOfDevices; loop++)
    {
        spiTxBursts[0][loop] = POWERSTEP01_NOP;
        spiTxBursts[1][loop] = POWERSTEP01_NOP;
        spiTxBursts[2][loop] = POWERSTEP01_NOP;
        spiTxBursts[3][loop] = POWERSTEP01_NOP;
    }
    switch (param & DAISY_CHAIN_COMMAND_MASK)
    {
      case POWERSTEP01_GO_TO:
      case POWERSTEP01_GO_TO_DIR:
               value = value & POWERSTEP01_ABS_POS_VALUE_MASK;
      case POWERSTEP01_RUN:
      case POWERSTEP01_MOVE:
      case POWERSTEP01_GO_UNTIL:
      case POWERSTEP01_GO_UNTIL_ACT_CPY:
               spiTxBursts[0][spiIndex] = param;
               spiTxBursts[1][spiIndex] = (uint8_t)(value >> 16);
               spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
               spiTxBursts[3][spiIndex] = (uint8_t)(value);
               maxArgumentNbBytes = 3;
               break;
      default:
               spiTxBursts[0][spiIndex] = POWERSTEP01_NOP;
               spiTxBursts[1][spiIndex] = POWERSTEP01_NOP;
               spiTxBursts[2][spiIndex] = POWERSTEP01_NOP;
               spiTxBursts[3][spiIndex] = param;
    }
    spiTxBursts[3][spiIndex] = (uint8_t)(value);
    
    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    noInterrupts();
    itDisable = true;
  } while (spiPreemtionByIsr); // check pre-emption by ISR
    for (loop = POWERSTEP01_CMD_ARG_MAX_NB_BYTES - 1 - maxArgumentNbBytes;
         loop < POWERSTEP01_CMD_ARG_MAX_NB_BYTES;
         loop++)
    {
       WriteBytes(&spiTxBursts[loop][0], &spiRxBursts[loop][0],deviceId);
    }
  }
}
/******************************************************//**
 * @brief  Sets the registers of the POWERSTEP01 to their predefined values
 * from POWERSTEP01_target_config.h
 * @param[in] DEVICEId (from 0 to 2)
 * @retval None
 **********************************************************/
void POWERSTEP01::SetRegisterToPredefinedValues(uint8_t deviceId)
{
	  powerstep01_CmVm_t cmVm;

	  CmdSetParam(deviceId, POWERSTEP01_ABS_POS, 0);
	  CmdSetParam(deviceId, POWERSTEP01_EL_POS, 0);
	  CmdSetParam(deviceId, POWERSTEP01_MARK, 0);

	  switch (deviceId)
	  {
	    case 0:
	      cmVm = POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_0;
	      CmdSetParam(deviceId, POWERSTEP01_ACC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_ACC_DEVICE_0));
	      CmdSetParam(deviceId, POWERSTEP01_DEC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_DEC_DEVICE_0));
	      CmdSetParam(deviceId, POWERSTEP01_MAX_SPEED, MaxSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_0));
	      CmdSetParam(deviceId, POWERSTEP01_MIN_SPEED, POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_0|
	                                                            MinSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_0));
	      CmdSetParam(deviceId, POWERSTEP01_FS_SPD,    POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_0|
	                                                            FSSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_0));
	      CmdSetParam(deviceId, POWERSTEP01_INT_SPD,   IntSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_0)); //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_K_THERM,   KTherm_to_Par(POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_0));       //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_OCD_TH,    (uint8_t)POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_0);
	      CmdSetParam(deviceId, POWERSTEP01_STALL_TH,  StallTh_to_Par(POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_0));      //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_STEP_MODE, (uint8_t)POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_0 |
	                                                            (uint8_t)POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_0|
	                                                            (uint8_t)POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_0);
	      CmdSetParam(deviceId, POWERSTEP01_ALARM_EN,  POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_0);
	      CmdSetParam(deviceId, POWERSTEP01_GATECFG1,  (uint16_t)POWERSTEP01_CONF_PARAM_IGATE_DEVICE_0 |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TCC_DEVICE_0   |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_0|
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_0);
	      CmdSetParam(deviceId, POWERSTEP01_GATECFG2,  (uint16_t)POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_0 |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TDT_DEVICE_0);

	      // Voltage mode
	      if (cmVm == POWERSTEP01_CM_VM_VOLTAGE)
	      {
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_HOLD,  Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_0));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_RUN,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_0));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_ACC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_0));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_DEC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_0));
	        CmdSetParam(deviceId, POWERSTEP01_ST_SLP,     BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_0));
	        CmdSetParam(deviceId, POWERSTEP01_FN_SLP_ACC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_0));
	        CmdSetParam(deviceId, POWERSTEP01_FN_SLP_DEC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_0));
	        CmdSetParam(deviceId, POWERSTEP01_CONFIG,     (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_0 |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_0       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_0       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_0         |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_0       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_0        |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_0       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_0);
	      }
	      else
	      {
	        // Current mode
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_HOLD, Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_0));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_RUN,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_0));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_ACC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_0));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_DEC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_0));
	        CmdSetParam(deviceId, POWERSTEP01_T_FAST,    (uint8_t)POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_0 |
	                                                              (uint8_t)POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_0);
	        CmdSetParam(deviceId, POWERSTEP01_TON_MIN,   Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_0));
	        CmdSetParam(deviceId, POWERSTEP01_TOFF_MIN,  Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_0));

	        CmdSetParam(deviceId, POWERSTEP01_CONFIG,    (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_0 |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_0       |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_0        |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_0         |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_0       |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_0        |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TSW_DEVICE_0           |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_PRED_EN_DEVICE_0);
	      }
	      break;
	   case 1:
	      cmVm = POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_1;
	      CmdSetParam(deviceId, POWERSTEP01_ACC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_ACC_DEVICE_1));
	      CmdSetParam(deviceId, POWERSTEP01_DEC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_DEC_DEVICE_1));
	      CmdSetParam(deviceId, POWERSTEP01_MAX_SPEED, MaxSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_1));
	      CmdSetParam(deviceId, POWERSTEP01_MIN_SPEED, POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_1|
	                                                            MinSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_1));
	      CmdSetParam(deviceId, POWERSTEP01_FS_SPD,    POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_1|
	                                                            FSSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_1));
	      CmdSetParam(deviceId, POWERSTEP01_INT_SPD,   IntSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_1)); //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_K_THERM,   KTherm_to_Par(POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_1));       //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_OCD_TH,    (uint8_t)POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_1);
	      CmdSetParam(deviceId, POWERSTEP01_STALL_TH,  StallTh_to_Par(POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_1));      //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_STEP_MODE, (uint8_t)POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_1 |
	                                                            (uint8_t)POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_1|
	                                                            (uint8_t)POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_1);
	      CmdSetParam(deviceId, POWERSTEP01_ALARM_EN,  POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_1);
	      CmdSetParam(deviceId, POWERSTEP01_GATECFG1,  (uint16_t)POWERSTEP01_CONF_PARAM_IGATE_DEVICE_1 |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TCC_DEVICE_1   |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_1|
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_1);
	      CmdSetParam(deviceId, POWERSTEP01_GATECFG2,  (uint16_t)POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_1 |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TDT_DEVICE_1);

	      // Voltage mode
	      if (cmVm == POWERSTEP01_CM_VM_VOLTAGE)
	      {
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_HOLD,  Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_1));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_RUN,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_1));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_ACC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_1));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_DEC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_1));
	        CmdSetParam(deviceId, POWERSTEP01_ST_SLP,     BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_1));
	        CmdSetParam(deviceId, POWERSTEP01_FN_SLP_ACC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_1));
	        CmdSetParam(deviceId, POWERSTEP01_FN_SLP_DEC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_1));
	        CmdSetParam(deviceId, POWERSTEP01_CONFIG,     (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_1 |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_1       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_1       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_1         |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_1       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_1        |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_1       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_1);
	      }
	      else
	      {
	        // Current mode
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_HOLD, Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_1));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_RUN,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_1));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_ACC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_1));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_DEC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_1));
	        CmdSetParam(deviceId, POWERSTEP01_T_FAST,    (uint8_t)POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_1 |
	                                                              (uint8_t)POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_1);
	        CmdSetParam(deviceId, POWERSTEP01_TON_MIN,   Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_1));
	        CmdSetParam(deviceId, POWERSTEP01_TOFF_MIN,  Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_1));

	        CmdSetParam(deviceId, POWERSTEP01_CONFIG,    (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_1 |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_1       |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_1        |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_1         |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_1       |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_1        |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TSW_DEVICE_1           |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_PRED_EN_DEVICE_1);
	      }
	      break;
	   case 2:
	      cmVm = POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_2;
	      CmdSetParam(deviceId, POWERSTEP01_ACC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_ACC_DEVICE_2));
	      CmdSetParam(deviceId, POWERSTEP01_DEC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_DEC_DEVICE_2));
	      CmdSetParam(deviceId, POWERSTEP01_MAX_SPEED, MaxSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_2));
	      CmdSetParam(deviceId, POWERSTEP01_MIN_SPEED, POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_2|
	                                                            MinSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_2));
	      CmdSetParam(deviceId, POWERSTEP01_FS_SPD,    POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_2|
	                                                            FSSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_2));
	      CmdSetParam(deviceId, POWERSTEP01_INT_SPD,   IntSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_2)); //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_K_THERM,   KTherm_to_Par(POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_2));       //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_OCD_TH,    (uint8_t)POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_2);
	      CmdSetParam(deviceId, POWERSTEP01_STALL_TH,  StallTh_to_Par(POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_2));      //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_STEP_MODE, (uint8_t)POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_2 |
	                                                            (uint8_t)POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_2|
	                                                            (uint8_t)POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_2);
	      CmdSetParam(deviceId, POWERSTEP01_ALARM_EN,  POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_2);
	      CmdSetParam(deviceId, POWERSTEP01_GATECFG1,  (uint16_t)POWERSTEP01_CONF_PARAM_IGATE_DEVICE_2 |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TCC_DEVICE_2   |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_2|
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_2);
	      CmdSetParam(deviceId, POWERSTEP01_GATECFG2,  (uint16_t)POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_2 |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TDT_DEVICE_2);

	      // Voltage mode
	      if (cmVm == POWERSTEP01_CM_VM_VOLTAGE)
	      {
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_HOLD,  Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_2));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_RUN,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_2));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_ACC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_2));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_DEC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_2));
	        CmdSetParam(deviceId, POWERSTEP01_ST_SLP,     BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_2));
	        CmdSetParam(deviceId, POWERSTEP01_FN_SLP_ACC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_2));
	        CmdSetParam(deviceId, POWERSTEP01_FN_SLP_DEC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_2));
	        CmdSetParam(deviceId, POWERSTEP01_CONFIG,     (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_2 |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_2       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_2       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_2         |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_2       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_2        |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_2       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_2);
	      }
	      else
	      {
	        // Current mode
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_HOLD, Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_2));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_RUN,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_2));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_ACC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_2));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_DEC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_2));
	        CmdSetParam(deviceId, POWERSTEP01_T_FAST,    (uint8_t)POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_2 |
	                                                              (uint8_t)POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_2);
	        CmdSetParam(deviceId, POWERSTEP01_TON_MIN,   Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_2));
	        CmdSetParam(deviceId, POWERSTEP01_TOFF_MIN,  Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_2));

	        CmdSetParam(deviceId, POWERSTEP01_CONFIG,    (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_2 |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_2       |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_2        |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_2         |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_2       |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_2        |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TSW_DEVICE_2           |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_PRED_EN_DEVICE_2);
	      }
	      break;
	   case 3:
	      cmVm = POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_3;
	      CmdSetParam(deviceId, POWERSTEP01_ACC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_ACC_DEVICE_3));
	      CmdSetParam(deviceId, POWERSTEP01_DEC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_DEC_DEVICE_3));
	      CmdSetParam(deviceId, POWERSTEP01_MAX_SPEED, MaxSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_3));
	      CmdSetParam(deviceId, POWERSTEP01_MIN_SPEED, POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_3|
	                                                            MinSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_3));
	      CmdSetParam(deviceId, POWERSTEP01_FS_SPD,    POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_3|
	                                                            FSSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_3));
	      CmdSetParam(deviceId, POWERSTEP01_INT_SPD,   IntSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_3)); //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_K_THERM,   KTherm_to_Par(POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_3));       //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_OCD_TH,    (uint8_t)POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_3);
	      CmdSetParam(deviceId, POWERSTEP01_STALL_TH,  StallTh_to_Par(POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_3));      //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_STEP_MODE, (uint8_t)POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_3 |
	                                                            (uint8_t)POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_3|
	                                                            (uint8_t)POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_3);
	      CmdSetParam(deviceId, POWERSTEP01_ALARM_EN,  POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_3);
	      CmdSetParam(deviceId, POWERSTEP01_GATECFG1,  (uint16_t)POWERSTEP01_CONF_PARAM_IGATE_DEVICE_3 |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TCC_DEVICE_3   |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_3|
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_3);
	      CmdSetParam(deviceId, POWERSTEP01_GATECFG2,  (uint16_t)POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_3 |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TDT_DEVICE_3);

	      // Voltage mode
	      if (cmVm == POWERSTEP01_CM_VM_VOLTAGE)
	      {
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_HOLD,  Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_3));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_RUN,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_3));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_ACC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_3));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_DEC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_3));
	        CmdSetParam(deviceId, POWERSTEP01_ST_SLP,     BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_3));
	        CmdSetParam(deviceId, POWERSTEP01_FN_SLP_ACC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_3));
	        CmdSetParam(deviceId, POWERSTEP01_FN_SLP_DEC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_3));
	        CmdSetParam(deviceId, POWERSTEP01_CONFIG,     (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_3 |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_3       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_3       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_3         |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_3       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_3        |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_3       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_3);
	      }
	      else
	      {
	        // Current mode
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_HOLD, Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_3));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_RUN,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_3));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_ACC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_3));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_DEC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_3));
	        CmdSetParam(deviceId, POWERSTEP01_T_FAST,    (uint8_t)POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_3 |
	                                                              (uint8_t)POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_3);
	        CmdSetParam(deviceId, POWERSTEP01_TON_MIN,   Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_3));
	        CmdSetParam(deviceId, POWERSTEP01_TOFF_MIN,  Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_3));

	        CmdSetParam(deviceId, POWERSTEP01_CONFIG,    (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_3 |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_3       |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_3        |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_3         |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_3       |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_3        |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TSW_DEVICE_3           |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_PRED_EN_DEVICE_3);
	      }
	      break;
	   case 4:
	      cmVm = POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_4;
	      CmdSetParam(deviceId, POWERSTEP01_ACC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_ACC_DEVICE_4));
	      CmdSetParam(deviceId, POWERSTEP01_DEC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_DEC_DEVICE_4));
	      CmdSetParam(deviceId, POWERSTEP01_MAX_SPEED, MaxSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_4));
	      CmdSetParam(deviceId, POWERSTEP01_MIN_SPEED, POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_4|
	                                                            MinSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_4));
	      CmdSetParam(deviceId, POWERSTEP01_FS_SPD,    POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_4|
	                                                            FSSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_4));
	      CmdSetParam(deviceId, POWERSTEP01_INT_SPD,   IntSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_4)); //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_K_THERM,   KTherm_to_Par(POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_4));       //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_OCD_TH,    (uint8_t)POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_4);
	      CmdSetParam(deviceId, POWERSTEP01_STALL_TH,  StallTh_to_Par(POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_4));      //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_STEP_MODE, (uint8_t)POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_4 |
	                                                            (uint8_t)POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_4|
	                                                            (uint8_t)POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_4);
	      CmdSetParam(deviceId, POWERSTEP01_ALARM_EN,  POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_4);
	      CmdSetParam(deviceId, POWERSTEP01_GATECFG1,  (uint16_t)POWERSTEP01_CONF_PARAM_IGATE_DEVICE_4 |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TCC_DEVICE_4   |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_4|
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_4);
	      CmdSetParam(deviceId, POWERSTEP01_GATECFG2,  (uint16_t)POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_4 |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TDT_DEVICE_4);

	      // Voltage mode
	      if (cmVm == POWERSTEP01_CM_VM_VOLTAGE)
	      {
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_HOLD,  Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_4));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_RUN,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_4));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_ACC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_4));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_DEC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_4));
	        CmdSetParam(deviceId, POWERSTEP01_ST_SLP,     BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_4));
	        CmdSetParam(deviceId, POWERSTEP01_FN_SLP_ACC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_4));
	        CmdSetParam(deviceId, POWERSTEP01_FN_SLP_DEC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_4));
	        CmdSetParam(deviceId, POWERSTEP01_CONFIG,     (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_4 |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_4       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_4       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_4         |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_4       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_4        |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_4       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_4);
	      }
	      else
	      {
	        // Current mode
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_HOLD, Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_4));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_RUN,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_4));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_ACC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_4));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_DEC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_4));
	        CmdSetParam(deviceId, POWERSTEP01_T_FAST,    (uint8_t)POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_4 |
	                                                              (uint8_t)POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_4);
	        CmdSetParam(deviceId, POWERSTEP01_TON_MIN,   Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_4));
	        CmdSetParam(deviceId, POWERSTEP01_TOFF_MIN,  Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_4));

	        CmdSetParam(deviceId, POWERSTEP01_CONFIG,    (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_4 |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_4       |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_4        |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_4         |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_4       |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_4        |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TSW_DEVICE_4           |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_PRED_EN_DEVICE_4);
	      }
	      break;
	   case 5:
	      cmVm = POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_5;
	      CmdSetParam(deviceId, POWERSTEP01_ACC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_ACC_DEVICE_5));
	      CmdSetParam(deviceId, POWERSTEP01_DEC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_DEC_DEVICE_5));
	      CmdSetParam(deviceId, POWERSTEP01_MAX_SPEED, MaxSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_5));
	      CmdSetParam(deviceId, POWERSTEP01_MIN_SPEED, POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_5|
	                                                            MinSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_5));
	      CmdSetParam(deviceId, POWERSTEP01_FS_SPD,    POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_5|
	                                                            FSSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_5));
	      CmdSetParam(deviceId, POWERSTEP01_INT_SPD,   IntSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_5)); //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_K_THERM,   KTherm_to_Par(POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_5));       //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_OCD_TH,    (uint8_t)POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_5);
	      CmdSetParam(deviceId, POWERSTEP01_STALL_TH,  StallTh_to_Par(POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_5));      //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_STEP_MODE, (uint8_t)POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_5 |
	                                                            (uint8_t)POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_5|
	                                                            (uint8_t)POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_5);
	      CmdSetParam(deviceId, POWERSTEP01_ALARM_EN,  POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_5);
	      CmdSetParam(deviceId, POWERSTEP01_GATECFG1,  (uint16_t)POWERSTEP01_CONF_PARAM_IGATE_DEVICE_5 |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TCC_DEVICE_5   |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_5|
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_5);
	      CmdSetParam(deviceId, POWERSTEP01_GATECFG2,  (uint16_t)POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_5 |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TDT_DEVICE_5);

	      // Voltage mode
	      if (cmVm == POWERSTEP01_CM_VM_VOLTAGE)
	      {
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_HOLD,  Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_5));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_RUN,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_5));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_ACC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_5));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_DEC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_5));
	        CmdSetParam(deviceId, POWERSTEP01_ST_SLP,     BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_5));
	        CmdSetParam(deviceId, POWERSTEP01_FN_SLP_ACC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_5));
	        CmdSetParam(deviceId, POWERSTEP01_FN_SLP_DEC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_5));
	        CmdSetParam(deviceId, POWERSTEP01_CONFIG,     (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_5 |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_5       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_5       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_5         |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_5       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_5        |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_5       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_5);
	      }
	      else
	      {
	        // Current mode
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_HOLD, Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_5));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_RUN,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_5));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_ACC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_5));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_DEC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_5));
	        CmdSetParam(deviceId, POWERSTEP01_T_FAST,    (uint8_t)POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_5 |
	                                                              (uint8_t)POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_5);
	        CmdSetParam(deviceId, POWERSTEP01_TON_MIN,   Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_5));
	        CmdSetParam(deviceId, POWERSTEP01_TOFF_MIN,  Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_5));

	        CmdSetParam(deviceId, POWERSTEP01_CONFIG,    (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_5 |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_5       |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_5        |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_5         |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_5       |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_5        |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TSW_DEVICE_5           |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_PRED_EN_DEVICE_5);
	      }
	      break;
	   case 6:
	      cmVm = POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_6;
	      CmdSetParam(deviceId, POWERSTEP01_ACC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_ACC_DEVICE_6));
	      CmdSetParam(deviceId, POWERSTEP01_DEC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_DEC_DEVICE_6));
	      CmdSetParam(deviceId, POWERSTEP01_MAX_SPEED, MaxSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_6));
	      CmdSetParam(deviceId, POWERSTEP01_MIN_SPEED, POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_6|
	                                                            MinSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_6));
	      CmdSetParam(deviceId, POWERSTEP01_FS_SPD,    POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_6|
	                                                            FSSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_6));
	      CmdSetParam(deviceId, POWERSTEP01_INT_SPD,   IntSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_6)); //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_K_THERM,   KTherm_to_Par(POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_6));       //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_OCD_TH,    (uint8_t)POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_6);
	      CmdSetParam(deviceId, POWERSTEP01_STALL_TH,  StallTh_to_Par(POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_6));      //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_STEP_MODE, (uint8_t)POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_6 |
	                                                            (uint8_t)POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_6|
	                                                            (uint8_t)POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_6);
	      CmdSetParam(deviceId, POWERSTEP01_ALARM_EN,  POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_6);
	      CmdSetParam(deviceId, POWERSTEP01_GATECFG1,  (uint16_t)POWERSTEP01_CONF_PARAM_IGATE_DEVICE_6 |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TCC_DEVICE_6   |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_6|
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_6);
	      CmdSetParam(deviceId, POWERSTEP01_GATECFG2,  (uint16_t)POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_6 |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TDT_DEVICE_6);

	      // Voltage mode
	      if (cmVm == POWERSTEP01_CM_VM_VOLTAGE)
	      {
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_HOLD,  Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_6));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_RUN,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_6));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_ACC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_6));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_DEC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_6));
	        CmdSetParam(deviceId, POWERSTEP01_ST_SLP,     BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_6));
	        CmdSetParam(deviceId, POWERSTEP01_FN_SLP_ACC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_6));
	        CmdSetParam(deviceId, POWERSTEP01_FN_SLP_DEC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_6));
	        CmdSetParam(deviceId, POWERSTEP01_CONFIG,     (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_6 |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_6       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_6       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_6         |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_6       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_6        |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_6       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_6);
	      }
	      else
	      {
	        // Current mode
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_HOLD, Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_6));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_RUN,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_6));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_ACC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_6));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_DEC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_6));
	        CmdSetParam(deviceId, POWERSTEP01_T_FAST,    (uint8_t)POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_6 |
	                                                              (uint8_t)POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_6);
	        CmdSetParam(deviceId, POWERSTEP01_TON_MIN,   Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_6));
	        CmdSetParam(deviceId, POWERSTEP01_TOFF_MIN,  Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_6));

	        CmdSetParam(deviceId, POWERSTEP01_CONFIG,    (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_6 |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_6       |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_6        |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_6         |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_6       |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_6        |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TSW_DEVICE_6           |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_PRED_EN_DEVICE_6);
	      }
	      break;
	   case 7:
	   default:
	      cmVm = POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_7;
	      CmdSetParam(deviceId, POWERSTEP01_ACC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_ACC_DEVICE_7));
	      CmdSetParam(deviceId, POWERSTEP01_DEC,       AccDec_Steps_to_Par(POWERSTEP01_CONF_PARAM_DEC_DEVICE_7));
	      CmdSetParam(deviceId, POWERSTEP01_MAX_SPEED, MaxSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_7));
	      CmdSetParam(deviceId, POWERSTEP01_MIN_SPEED, POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_7|
	                                                            MinSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_7));
	      CmdSetParam(deviceId, POWERSTEP01_FS_SPD,    POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_7|
	                                                            FSSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_7));
	      CmdSetParam(deviceId, POWERSTEP01_INT_SPD,   IntSpd_Steps_to_Par(POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_7)); //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_K_THERM,   KTherm_to_Par(POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_7));       //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_OCD_TH,    (uint8_t)POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_7);
	      CmdSetParam(deviceId, POWERSTEP01_STALL_TH,  StallTh_to_Par(POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_7));      //voltage mode only but not redefined for current mode
	      CmdSetParam(deviceId, POWERSTEP01_STEP_MODE, (uint8_t)POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_7 |
	                                                            (uint8_t)POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_7|
	                                                            (uint8_t)POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_7);
	      CmdSetParam(deviceId, POWERSTEP01_ALARM_EN,  POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_7);
	      CmdSetParam(deviceId, POWERSTEP01_GATECFG1,  (uint16_t)POWERSTEP01_CONF_PARAM_IGATE_DEVICE_7 |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TCC_DEVICE_7   |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_7|
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_7);
	      CmdSetParam(deviceId, POWERSTEP01_GATECFG2,  (uint16_t)POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_7 |
	                                                            (uint16_t)POWERSTEP01_CONF_PARAM_TDT_DEVICE_7);

	      // Voltage mode
	      if (cmVm == POWERSTEP01_CM_VM_VOLTAGE)
	      {
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_HOLD,  Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_7));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_RUN,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_7));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_ACC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_7));
	        CmdSetParam(deviceId, POWERSTEP01_KVAL_DEC,   Kval_Perc_to_Par(POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_7));
	        CmdSetParam(deviceId, POWERSTEP01_ST_SLP,     BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_7));
	        CmdSetParam(deviceId, POWERSTEP01_FN_SLP_ACC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_7));
	        CmdSetParam(deviceId, POWERSTEP01_FN_SLP_DEC, BEMF_Slope_Perc_to_Par(POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_7));
	        CmdSetParam(deviceId, POWERSTEP01_CONFIG,     (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_7 |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_7       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_7       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_7         |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_7       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_7        |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_7       |
	                                                                  (uint16_t)POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_7);
	      }
	      else
	      {
	        // Current mode
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_HOLD, Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_7));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_RUN,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_7));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_ACC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_7));
	        CmdSetParam(deviceId, POWERSTEP01_TVAL_DEC,  Tval_Current_to_Par(POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_7));
	        CmdSetParam(deviceId, POWERSTEP01_T_FAST,    (uint8_t)POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_7 |
	                                                              (uint8_t)POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_7);
	        CmdSetParam(deviceId, POWERSTEP01_TON_MIN,   Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_7));
	        CmdSetParam(deviceId, POWERSTEP01_TOFF_MIN,  Tmin_Time_to_Par(POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_7));

	        CmdSetParam(deviceId, POWERSTEP01_CONFIG,    (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_7 |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_7       |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_7        |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_7         |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_7       |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_7        |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_TSW_DEVICE_7           |
	                                                              (uint16_t)POWERSTEP01_CONF_PARAM_PRED_EN_DEVICE_7);
	      }
	      break;
	  }
}

/******************************************************//**
 * @brief  Sets the registers of the POWERSTEP01 to their predefined values
 * from POWERSTEP01_target_config.h
 * @param[in] DEVICEId (from 0 to 2)
 * @retval None
 **********************************************************/
void POWERSTEP01::WriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte, uint8_t deviceId)
{
  digitalWrite(_SSPins[deviceId], LOW);
  for (uint32_t i = 0; i < numberOfDevices; i++)
  {
    *pReceivedByte = SPI.transfer(*pByteToTransmit);
    pByteToTransmit++;
    pReceivedByte++;
  }
  digitalWrite(_SSPins[deviceId], HIGH);
  if (isrFlag)
  {
    spiPreemtionByIsr = true;
  }
}

///////////////////////////////// NEW ADDED FUNCTIONS

/******************************************************//**
 * @brief Sets maximum speed of the stepper in the 
 * MAX_SPEED register. if a command tried setting a speed more than this the motor 
 * will simply run at this speed. The value is 0x041 on power up 
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * &param[in] stepsPerSecond steps per second 
 * @retval none
 *********************************************************/
void POWERSTEP01::setMaxSpeed(uint8_t DeviceId, unsigned long stepsPerSecond)
{
	unsigned long integerSpeed = maxSpeedCalc(stepsPerSecond);
	CmdSetParam(DeviceId, POWERSTEP01_MAX_SPEED, integerSpeed);
	return;
}


/******************************************************//**
 * @brief calculates the speed by converting the value 
 * from steps/s to a 10 bit value. the maximum for 10 bits is 0x03ff
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * &param[in] stepsPerSecond steps per second 
 * @retval unsigned long steps per tick 
 *********************************************************/
unsigned long POWERSTEP01::maxSpeedCalc (unsigned long stepsPerSec)
{
	unsigned long temp = ceil(stepsPerSec* .065536);
	if (temp > 0x000003FF) return 0x000003FF;
	else return temp;
}


	/******************************************************//**
 * @brief Sets minimum speed of the stepper in the 
 * MIN_SPEED register.for any move or goto function where no speed is specified
* this value will be used 
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * &param[in] stepsPerSecond steps per second 
 * @retval none
 *********************************************************/
void POWERSTEP01::setMinSpeed(uint8_t DeviceId, int speed){

	CmdSetParam(DeviceId, POWERSTEP01_MIN_SPEED, MinSpeedCalc(speed));
}

/******************************************************//**
 * @brief The value in the MIN_SPEED register is [(steps/s)*(tick)]/(2^-24) where tick is 
 * 250ns (datasheet value)- 0x000 on boot.
 * Multiply desired steps/s by 4.1943 to get an appropriate value for this register
 * This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * &param[in] stepsPerSecond steps per second 
 * @retval unsigned long steps per tick 
 *********************************************************/
unsigned long POWERSTEP01::MinSpeedCalc(float stepsPerSec){

	float temp = stepsPerSec * 4.1943;
	if( (unsigned long) long(temp) > 0x00000FFF) return 0x00000FFF;
	else return (unsigned long) long(temp);
}


/******************************************************//**
 * @brief Configure the acceleration rate, in steps/tick/tick. There is also a DEC register;
 * both of them have a function (AccCalc() and DecCalc() respectively) that convert
 * from steps/s/s into the appropriate value for the register. Writing ACC to 0xfff
 * sets the acceleration and deceleration to 'infinite' (or as near as the driver can
 *  manage). If ACC is set to 0xfff, DEC is ignored. To get infinite deceleration
 * without infinite acceleration, only hard stop will work.
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * &param[in] stepsPerSecondpersecond steps per second per second 
 * @retval none
 *********************************************************/	
void POWERSTEP01::setAcc(uint8_t DeviceId, unsigned long stepsPerSecondPerSecond)
{
	unsigned long integerAcc = accCalc(stepsPerSecondPerSecond);
	CmdSetParam(DeviceId, POWERSTEP01_ACC, integerAcc);
	return;
}

 
/******************************************************//**
 * @brief The value in the ACC register is [(steps/s/s)*(tick^2)]/(2^-40) where tick is
 * 250ns (datasheet value)- 0x08A on boot.
 * Multiply desired steps/s/s by .137438 to get an appropriate value for this register.
 * This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
 * &param[in] stepsPerSecondPerSecond steps per second per second
 * @retval unsigned long 12 bit acc
 *********************************************************/
unsigned long POWERSTEP01::accCalc(unsigned long stepsPerSecPerSec)
{
	unsigned long temp = stepsPerSecPerSec * 0.137438;
	if(temp > 0x00000FFF) return 0x00000FFF;
	else return temp;
}

/******************************************************//**
 * @brief Configure the acceleration rate, in steps/tick/tick. There is also a DEC register;
 * both of them have a function (AccCalc() and DecCalc() respectively) that convert
 * from steps/s/s into the appropriate value for the register. Writing ACC to 0xfff
 * sets the acceleration and deceleration to 'infinite' (or as near as the driver can
 *  manage). If ACC is set to 0xfff, DEC is ignored. To get infinite deceleration
 * without infinite acceleration, only hard stop will work.
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * &param[in] stepsPerSecondpersecond steps per second per second 
 * @retval none
 *********************************************************/	
void POWERSTEP01::setDec(uint8_t DeviceId, unsigned long stepsPerSecondPerSecond)
{
	unsigned long integerDec = decCalc(stepsPerSecondPerSecond);
	CmdSetParam(DeviceId, POWERSTEP01_DEC, integerDec);
	return;
}


/******************************************************//**
 * @brief The calculation for DEC is the same as for ACC. Value is 0x08A on boot.
 * This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
 * Multiply desired steps/s/s by .137438 to get an appropriate value for this register.
 * This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
 * &param[in] stepsPerSecondPerSecond steps per second per second
 * @retval unsigned long 12 bit dec
 *********************************************************/
unsigned long POWERSTEP01::decCalc(unsigned long stepsPerSecPerSec)
{
	unsigned long temp = stepsPerSecPerSec * 0.137438;
	if(temp > 0x00000FFF) return 0x00000FFF;
	else return temp;
}

/******************************************************//**
 * @brief This sets the full speed register to a certain speed
 * the full speed is the speed that if surpassed the motor will cease microstepping 
 * and go to full-step
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * &param[in] stepsPerSecond steps per second 
 * @retval none
 *********************************************************/	
void POWERSTEP01::setFullSpeed(uint8_t DeviceId, float stepsPerSecond)
{
  unsigned long integerSpeed = FSCalc(stepsPerSecond);
  CmdSetParam(DeviceId, POWERSTEP01_FS_SPD, integerSpeed);
}

// Alias for setFullSpeed 
void POWERSTEP01::setThresholdSpeed(uint8_t DeviceId,float stepsPerSecond)
{setFullSpeed(DeviceId,stepsPerSecond);}

/******************************************************//**
 * @brief the value in the FS_SPD register is ([(steps/s)*(tick)]/(2^-18))-0.5 where tick is 
 * 250ns (datasheet value)- 0x027 on boot.
 * Multiply desired steps/s by .065536 and subtract .5 to get an appropriate value for this register
 * This is a 10-bit value, so we need to make sure the value is at or below 0x3FF.
 * &param[in] stepsPerSecond steps per second 
 * @retval unsigned long steps per tick 
 *********************************************************/	
unsigned long POWERSTEP01::FSCalc(float stepsPerSec)
{
  float temp = (stepsPerSec * .065536)-.5;
  if( (unsigned long) long(temp) > 0x000003FF) return 0x000003FF;
  else return (unsigned long) long(temp);
}


/******************************************************//**
 * @brief This function maps the input from 
 * the main function code to the selectstpeMode function to modify the microstepping
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * &param[in] microSteps this parameter can be 1,2,4,8,16,32,64,128 
 * @retval none
 *********************************************************/
void POWERSTEP01::setMicroSteps(uint8_t deviceId, int microSteps){
	
	switch (microSteps)
	{
		case 1:
		SelectStepMode(deviceId, STEP_MODE_FULL);
		break;
		case 2: 
		SelectStepMode(deviceId, STEP_MODE_HALF);
		break;		
		case 4: 
		SelectStepMode(deviceId, STEP_MODE_1_4);
		break;
		case 8: 
		SelectStepMode(deviceId, STEP_MODE_1_8);
		break;
		case 16: 
		SelectStepMode(deviceId, STEP_MODE_1_16);
		break;
		case 32: 
		SelectStepMode(deviceId, STEP_MODE_1_32);
		break;
		case 64: 
		SelectStepMode(deviceId, STEP_MODE_1_64);
		break;
		case 128: 
		SelectStepMode(deviceId, STEP_MODE_1_128);
		break;
		default: 
		SelectStepMode(deviceId, STEP_MODE_FULL);	
	}
	 return;	  
}

/******************************************************//**
 * @brief Issues PowerStep01 Go Until command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @param[in] action ACTION_RESET or ACTION_COPY
 * @param[in] direction movement direction
 * @param[in] speed in 2^-28 step/tick
 * @retval None
 *********************************************************/
void POWERSTEP01::CmdGoUntil(uint8_t deviceId, 
                           motorAction_t action, 
                           motorDir_t direction, 
                           uint32_t speed)
{
  SendCommand(deviceId,
                          (uint8_t)POWERSTEP01_GO_UNTIL | (uint8_t)action | (uint8_t)direction,
                          speed); 
}

/******************************************************//**
 * @brief  Sets the number of devices to be used 
 * @param[in] nbDevices (from 1 to MAX_NUMBER_OF_DEVICES)
 * @retval TRUE if successfull, FALSE if failure, attempt to set a number of 
 * devices greater than MAX_NUMBER_OF_DEVICES
 **********************************************************/
bool POWERSTEP01::SetNbDevices(uint8_t nbDevices)
{
  if (nbDevices <= MAX_NUMBER_OF_DEVICES)
  {
    numberOfDevices = nbDevices;
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

/******************************************************//**
 * @brief Checks if the specified device is busy [Different implementation of IsDeviceBusy 
 * by reading the Busy flag bit ot its status Register
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval true if device is busy, false zero
 *********************************************************/
bool POWERSTEP01::IsBusy(uint8_t deviceId)
{
cli();
	int temp = 0;
	digitalWrite(_SSPins[deviceId],LOW);
	temp=SPI.transfer(POWERSTEP01_GET_STATUS);
	temp= SPI.transfer(0)<<8;
	temp |= SPI.transfer(0);
	SPI.endTransaction();
	digitalWrite(_SSPins[deviceId],HIGH);
	sei();
	return (!bitRead(temp,1));
	

}

/******************************************************//**
 * @brief  Debug   function to get the amount of free ram
 * @param  None
 * @retval number of bytes of free ram
 **********************************************************/
#ifdef _DEBUG_POWERSTEP01
uint16_t GetFreeRam (void)
{
  extern uint16_t __heap_start, *__brkval;
  uint16_t v;
  return (uint16_t) &v - (__brkval == 0 ? (uint16_t) &__heap_start : (uint16_t) __brkval);
}
#endif



                                    

