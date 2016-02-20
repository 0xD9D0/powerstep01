/******************************************************//**
 * @file    POWERSTEP01.h
 * @version V1.0
 * @date    March 3, 2014
 * @brief   Header for POWERSTEP01   library for arduino
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 **********************************************************/ 

#ifndef __POWERSTEP01_H_INCLUDED
#define __POWERSTEP01_H_INCLUDED

#include "Arduino.h"
#include <inttypes.h>

#include "../Powerstep01/Powerstep01_target_config.h"
#include "../Powerstep01/motor.h"

//To use 3 POWERSTEP01 DEVICE boards you have to ebable this flag
//but you will no more able to use Arduino functions which
//are based on timer 0 (delay(), millis()...)
//#define _USE_TIMER_0_FOR_POWERSTEP01

/// Define to print debug logs via the UART 
#ifndef _DEBUG_POWERSTEP01
//#define _DEBUG_POWERSTEP01
#endif

#ifdef _DEBUG_POWERSTEP01
/// Size of the log buffer
#define DEBUG_BUFFER_SIZE    (75)
/// Log buffer
extern char POWERSTEP01StrOut[DEBUG_BUFFER_SIZE];
#endif

/// Clear bit Macro 
#ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

/// Set bit Macro 
#ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

/// Current FW version
#define POWERSTEP01_FW_VERSION (1)

/// POWERSTEP01 max number of bytes of command & arguments to set a parameter
#define POWERSTEP01_CMD_ARG_MAX_NB_BYTES              (4)

/// POWERSTEP01 command + argument bytes number for NOP command
#define POWERSTEP01_CMD_ARG_NB_BYTES_NOP              (1)


/// Powerstep01 command + argument bytes number for RUN command
#define POWERSTEP01_CMD_ARG_NB_BYTES_RUN              (4)
/// Powerstep01 command + argument bytes number for STEP_CLOCK command
#define POWERSTEP01_CMD_ARG_NB_BYTES_STEP_CLOCK       (1)
/// Powerstep01 command + argument bytes number for MOVE command
#define POWERSTEP01_CMD_ARG_NB_BYTES_MOVE             (4)
/// Powerstep01 command + argument bytes number for GO_TO command
#define POWERSTEP01_CMD_ARG_NB_BYTES_GO_TO            (4)
/// Powerstep01 command + argument bytes number for GO_TO_DIR command
#define POWERSTEP01_CMD_ARG_NB_BYTES_GO_TO_DIR        (4)
/// Powerstep01 command + argument bytes number for GO_UNTIL command
#define POWERSTEP01_CMD_ARG_NB_BYTES_GO_UNTIL         (4)
/// Powerstep01 command + argument bytes number for RELEASE_SW command
#define POWERSTEP01_CMD_ARG_NB_BYTES_RELEASE_SW       (1)
/// Powerstep01 command + argument bytes number for GO_HOME command
#define POWERSTEP01_CMD_ARG_NB_BYTES_GO_HOME          (1)
/// Powerstep01 command + argument bytes number for GO_MARK command
#define POWERSTEP01_CMD_ARG_NB_BYTES_GO_MARK          (1)
/// Powerstep01 command + argument bytes number for RESET_POS command
#define POWERSTEP01_CMD_ARG_NB_BYTES_RESET_POS        (1)
/// Powerstep01 command + argument bytes number for RESET_DEVICE command
#define POWERSTEP01_CMD_ARG_NB_BYTES_RESET_DEVICE     (1)
/// Powerstep01 command + argument bytes number for NOP command
#define POWERSTEP01_CMD_ARG_NB_BYTES_SOFT_STOP        (1)
/// Powerstep01 command + argument bytes number for HARD_STOP command
#define POWERSTEP01_CMD_ARG_NB_BYTES_HARD_STOP        (1)
/// Powerstep01 command + argument bytes number for SOFT_HIZ command
#define POWERSTEP01_CMD_ARG_NB_BYTES_SOFT_HIZ         (1)
/// Powerstep01 command + argument bytes number for ARD_HIZ command
#define POWERSTEP01_CMD_ARG_NB_BYTES_HARD_HIZ         (1)
/// Powerstep01 command + argument bytes number for GET_STATUS command
#define POWERSTEP01_CMD_ARG_NB_BYTES_GET_STATUS       (1)

/// Powerstep01 response bytes number 
#define POWERSTEP01_RSP_NB_BYTES_GET_STATUS           (2)  

/// Digital Pins used for the POWERSTEP01 flag pin
#define POWERSTEP01_FLAG_Pin   (2)
/// Digital Pins used for the POWERSTEP01 step clock pin of DEVICE 0
#define POWERSTEP01_PWM_1_Pin  (9)
/// Digital Pins used for the POWERSTEP01 step clock pin of DEVICE 1
#define POWERSTEP01_PWM_2_Pin  (3)
/// Digital Pins used for the POWERSTEP01 step clock pin of DEVICE 2
#define POWERSTEP01_PWM_3_Pin  (6)
/// Digital Pins used for the POWERSTEP01 direction pin of DEVICE 0
#define POWERSTEP01_DIR_1_Pin  (7)
/// Digital Pins used for the POWERSTEP01 direction pin of DEVICE 1
#define POWERSTEP01_DIR_2_Pin  (4)
/// Digital Pins used for the POWERSTEP01 direction pin of DEVICE 2
#define POWERSTEP01_DIR_3_Pin  (5)
/// Digital Pins used for the POWERSTEP01 reset pin
#define POWERSTEP01_Reset_Pin  (8)



/// Maximum number of steps
#define MAX_STEPS         (0x7FFFFFFF)
/// uint8_t max value
#define UINT8_MAX         (uint8_t)(0XFF)
/// uint16_t max value
#define UINT16_MAX        (uint16_t)(0XFFFF)

/// Pwm prescaler array size for timer 0 & 1
#define PRESCALER_ARRAY_TIMER0_1_SIZE   (6)
/// Pwm prescaler array size for timer 2
#define PRESCALER_ARRAY_TIMER2_SIZE     (8)

/// Maximum frequency of the PWMs
#define POWERSTEP01_MAX_PWM_FREQ   (10000)
/// Minimum frequency of the PWMs
#define POWERSTEP01_MIN_PWM_FREQ   (30)
  



/// POWERSTEP01 command + argument bytes number for ENABLE command
#define POWERSTEP01_CMD_ARG_NB_BYTES_ENABLE           (1)
/// POWERSTEP01 command + argument bytes number for DISABLE command
#define POWERSTEP01_CMD_ARG_NB_BYTES_DISABLE          (1)




/// Daisy chain command mask
#define DAISY_CHAIN_COMMAND_MASK (0xFA)

/// powerSTEP01 max absolute position
#define POWERSTEP01_MAX_POSITION (int32_t)(0x001FFFFF)

/// powerSTEP01 min absolute position
#define POWERSTEP01_MIN_POSITION (int32_t)(0xFFE00000)

/// masks for ABS_POS register of PowerStep01
typedef enum {
  POWERSTEP01_ABS_POS_VALUE_MASK        = ((uint32_t) 0x003FFFFF),
  POWERSTEP01_ABS_POS_SIGN_BIT_MASK     = ((uint32_t) 0x00200000)
} powerstep01_AbsPosMasks_t;

/// masks for EL_POS register of PowerStep01
typedef enum {
  POWERSTEP01_ELPOS_STEP_MASK       = ((uint8_t)0xC0),
  POWERSTEP01_ELPOS_MICROSTEP_MASK  = ((uint8_t)0x3F)
} powerstep01_ElPosMasks_t;

/// masks for MIN_SPEED register of PowerStep01
typedef enum {
  POWERSTEP01_LSPD_OPT        = ((uint16_t) ((0x1) << 12)),
  POWERSTEP01_MIN_SPEED_MASK  = ((uint16_t)0x0FFF)
} powerstep01_MinSpeedMasks_t;

/// Low speed optimization (MIN_SPEED register of PowerStep01)
typedef enum {
  POWERSTEP01_LSPD_OPT_OFF    = ((uint16_t)0x0000),
  POWERSTEP01_LSPD_OPT_ON     = ((uint16_t)POWERSTEP01_LSPD_OPT)
} powerstep01_LspdOpt_t;


/// masks for FS_SPD register of PowerStep01
typedef enum {
  POWERSTEP01_BOOST_MODE   = ((uint16_t) ((0x1) << 10)),
  POWERSTEP01_FS_SPD_MASK  = ((uint16_t)0x03FF)
} powerstep01_FsSpdMasks_t;

/// Full step boost (FS_SPD register of PowerStep01)
typedef enum {
  POWERSTEP01_BOOST_MODE_OFF    = ((uint16_t)0x0000),
  POWERSTEP01_BOOST_MODE_ON     = ((uint16_t)POWERSTEP01_BOOST_MODE)
} powerstep01_BoostMode_t;

/// POWERSTEP01 fall step time options (FAST_STEP values for T_FAST register )
typedef enum {
  POWERSTEP01_FAST_STEP_2us = ((uint8_t) 0x00),
  POWERSTEP01_FAST_STEP_4us = ((uint8_t) 0x01),
  POWERSTEP01_FAST_STEP_6us = ((uint8_t) 0x02),
  POWERSTEP01_FAST_STEP_8us = ((uint8_t) 0x03),
  POWERSTEP01_FAST_STEP_10us = ((uint8_t) 0x04),
  POWERSTEP01_FAST_STEP_12us = ((uint8_t) 0x05),
  POWERSTEP01_FAST_STEP_14us = ((uint8_t) 0x06),
  POWERSTEP01_FAST_STEP_16us = ((uint8_t) 0x07),
  POWERSTEP01_FAST_STEP_18us = ((uint8_t) 0x08),
  POWERSTEP01_FAST_STEP_20us = ((uint8_t) 0x09),
  POWERSTEP01_FAST_STEP_22us = ((uint8_t) 0x0A),
  POWERSTEP01_FAST_STEP_24us = ((uint8_t) 0x0B),
  POWERSTEP01_FAST_STEP_26us = ((uint8_t) 0x0C),
  POWERSTEP01_FAST_STEP_28us = ((uint8_t) 0x0D),
  POWERSTEP01_FAST_STEP_30us = ((uint8_t) 0x0E),
  POWERSTEP01_FAST_STEP_32us = ((uint8_t) 0x0F)
} powerstep01_FastStep_t;

/// POWERSTEP01 fast decay time option (TOFF_FAST values for T_FAST register )
typedef enum {
  POWERSTEP01_TOFF_FAST_2us = ((uint8_t) 0x00 << 4),
  POWERSTEP01_TOFF_FAST_4us = ((uint8_t) 0x01 << 4),
  POWERSTEP01_TOFF_FAST_6us = ((uint8_t) 0x02 << 4),
  POWERSTEP01_TOFF_FAST_8us = ((uint8_t) 0x03 << 4),
  POWERSTEP01_TOFF_FAST_10us = ((uint8_t) 0x04 << 4),
  POWERSTEP01_TOFF_FAST_12us = ((uint8_t) 0x05 << 4),
  POWERSTEP01_TOFF_FAST_14us = ((uint8_t) 0x06 << 4),
  POWERSTEP01_TOFF_FAST_16us = ((uint8_t) 0x07 << 4),
  POWERSTEP01_TOFF_FAST_18us = ((uint8_t) 0x08 << 4),
  POWERSTEP01_TOFF_FAST_20us = ((uint8_t) 0x09 << 4),
  POWERSTEP01_TOFF_FAST_22us = ((uint8_t) 0x0A << 4),
  POWERSTEP01_TOFF_FAST_24us = ((uint8_t) 0x0B << 4),
  POWERSTEP01_TOFF_FAST_26us = ((uint8_t) 0x0C << 4),
  POWERSTEP01_TOFF_FAST_28us = ((uint8_t) 0x0D << 4),
  POWERSTEP01_TOFF_FAST_30us = ((uint8_t) 0x0E << 4),
  POWERSTEP01_TOFF_FAST_32us = ((uint8_t) 0x0F << 4)
} POWERSTEP01_TOFF_FAST_t;



/// Overcurrent threshold options (OCD register of PowerStep01)
typedef enum {
	  POWERSTEP01_OCD_TH_31_25mV    = ((uint8_t)0x00),
	  POWERSTEP01_OCD_TH_62_5mV     = ((uint8_t)0x01),
	  POWERSTEP01_OCD_TH_93_75mV    = ((uint8_t)0x02),
	  POWERSTEP01_OCD_TH_125mV      = ((uint8_t)0x03),
	  POWERSTEP01_OCD_TH_156_25mV   = ((uint8_t)0x04),
	  POWERSTEP01_OCD_TH_187_50mV   = ((uint8_t)0x05),
	  POWERSTEP01_OCD_TH_218_75mV   = ((uint8_t)0x06),
	  POWERSTEP01_OCD_TH_250mV      = ((uint8_t)0x07),
	  POWERSTEP01_OCD_TH_281_25mV   = ((uint8_t)0x08),
	  POWERSTEP01_OCD_TH_312_5mV    = ((uint8_t)0x09),
	  POWERSTEP01_OCD_TH_343_75mV   = ((uint8_t)0x0A),
	  POWERSTEP01_OCD_TH_375mV      = ((uint8_t)0x0B),
	  POWERSTEP01_OCD_TH_406_25mV   = ((uint8_t)0x0C),
	  POWERSTEP01_OCD_TH_437_5mV    = ((uint8_t)0x0D),
	  POWERSTEP01_OCD_TH_468_75mV   = ((uint8_t)0x0E),
	  POWERSTEP01_OCD_TH_500mV      = ((uint8_t)0x0F),
	  POWERSTEP01_OCD_TH_531_25mV   = ((uint8_t)0x10),
	  POWERSTEP01_OCD_TH_562_5mV    = ((uint8_t)0x11),
	  POWERSTEP01_OCD_TH_593_75mV   = ((uint8_t)0x12),
	  POWERSTEP01_OCD_TH_625mV      = ((uint8_t)0x13),
	  POWERSTEP01_OCD_TH_656_25mV   = ((uint8_t)0x14),
	  POWERSTEP01_OCD_TH_687_5mV    = ((uint8_t)0x15),
	  POWERSTEP01_OCD_TH_718_75mV   = ((uint8_t)0x16),
	  POWERSTEP01_OCD_TH_750mV      = ((uint8_t)0x17),
	  POWERSTEP01_OCD_TH_781_25mV   = ((uint8_t)0x18),
	  POWERSTEP01_OCD_TH_812_5mV    = ((uint8_t)0x19),
	  POWERSTEP01_OCD_TH_843_75mV   = ((uint8_t)0x1A),
	  POWERSTEP01_OCD_TH_875mV      = ((uint8_t)0x1B),
	  POWERSTEP01_OCD_TH_906_25mV   = ((uint8_t)0x1C),
	  POWERSTEP01_OCD_TH_937_75mV   = ((uint8_t)0x1D),
	  POWERSTEP01_OCD_TH_968_75mV   = ((uint8_t)0x1E),
	  POWERSTEP01_OCD_TH_1V         = ((uint8_t)0x1F)
} powerstep01_OcdTh_t;

/// masks for STEP_MODE register of PowerStep01
typedef enum {
  POWERSTEP01_STEP_MODE_STEP_SEL    = ((uint8_t)0x07),
  POWERSTEP01_STEP_MODE_CM_VM       = ((uint8_t)0x08),
  POWERSTEP01_STEP_MODE_SYNC_SEL    = ((uint8_t)0x70),
  POWERSTEP01_STEP_MODE_SYNC_EN     = ((uint8_t)0x80)
} powerstep01_StepModeMasks_t;

/// Voltage or Current mode selection (CM_VM field of STEP_MODE register of PowerStep01)
typedef enum {
  POWERSTEP01_CM_VM_VOLTAGE    = ((uint8_t)0x00),
  POWERSTEP01_CM_VM_CURRENT    = ((uint8_t)0x08)
} powerstep01_CmVm_t;

/// Stepping options (field STEP_SEL of STEP_MODE register of PowerStep01)
typedef enum {
 POWERSTEP01_STEP_SEL_1      = ((uint8_t)0x00),
 POWERSTEP01_STEP_SEL_1_2    = ((uint8_t)0x01),
 POWERSTEP01_STEP_SEL_1_4    = ((uint8_t)0x02),
 POWERSTEP01_STEP_SEL_1_8    = ((uint8_t)0x03),
 POWERSTEP01_STEP_SEL_1_16   = ((uint8_t)0x04),
 POWERSTEP01_STEP_SEL_1_32   = ((uint8_t)0x05),
 POWERSTEP01_STEP_SEL_1_64   = ((uint8_t)0x06),
 POWERSTEP01_STEP_SEL_1_128  = ((uint8_t)0x07)
} powerstep01_StepSel_t;

/// Powerstep01 Sync Output frequency enabling bitw
#define POWERSTEP01_SYNC_EN   ((0x1) << 7)

/// SYNC_SEL options (STEP_MODE register of PowerStep01)
typedef enum {
  POWERSTEP01_SYNC_SEL_DISABLED   = ((uint8_t)0x00),
  POWERSTEP01_SYNC_SEL_1_2        = ((uint8_t)(POWERSTEP01_SYNC_EN|0x00)),
  POWERSTEP01_SYNC_SEL_1          = ((uint8_t)(POWERSTEP01_SYNC_EN|0x10)),
  POWERSTEP01_SYNC_SEL_2          = ((uint8_t)(POWERSTEP01_SYNC_EN|0x20)),
  POWERSTEP01_SYNC_SEL_4          = ((uint8_t)(POWERSTEP01_SYNC_EN|0x30)),
  POWERSTEP01_SYNC_SEL_8          = ((uint8_t)(POWERSTEP01_SYNC_EN|0x40)),
  POWERSTEP01_SYNC_SEL_16         = ((uint8_t)(POWERSTEP01_SYNC_EN|0x50)),
  POWERSTEP01_SYNC_SEL_32         = ((uint8_t)(POWERSTEP01_SYNC_EN|0x60)),
  POWERSTEP01_SYNC_SEL_64         = ((uint8_t)(POWERSTEP01_SYNC_EN|0x70))
} powerstep01_SyncSel_t;

/// Alarms conditions (ALARM_EN register of PowerStep01)
typedef enum {
  POWERSTEP01_ALARM_EN_OVERCURRENT        = ((uint8_t)0x01),
  POWERSTEP01_ALARM_EN_THERMAL_SHUTDOWN   = ((uint8_t)0x02),
  POWERSTEP01_ALARM_EN_THERMAL_WARNING    = ((uint8_t)0x04),
  POWERSTEP01_ALARM_EN_UVLO               = ((uint8_t)0x08),
  POWERSTEP01_ALARM_EN_ADC_UVLO           = ((uint8_t)0x10),
  POWERSTEP01_ALARM_EN_STALL_DETECTION    = ((uint8_t)0x20),
  POWERSTEP01_ALARM_EN_SW_TURN_ON         = ((uint8_t)0x40),
  POWERSTEP01_ALARM_EN_WRONG_NPERF_CMD    = ((uint8_t)0x80)
} powerstep01_AlarmEn_t;

/// masks for GATECFG1 register of PowerStep01
typedef enum {
  POWERSTEP01_GATECFG1_TCC         = ((uint16_t)0x001F),
  POWERSTEP01_GATECFG1_IGATE       = ((uint16_t)0x00E0),
  POWERSTEP01_GATECFG1_TBOOST      = ((uint16_t)0x0700),
  POWERSTEP01_GATECFG1_WD_EN       = ((uint16_t)0x0800)
} powerstep01_GateCfg1Masks_t;

/// Control current Time (field TCC of GATECFG1 register of PowerStep01)
typedef enum {
  POWERSTEP01_TCC_125ns       = ((uint8_t)0x00),
  POWERSTEP01_TCC_250ns       = ((uint8_t)0x01),
  POWERSTEP01_TCC_375ns       = ((uint8_t)0x02),
  POWERSTEP01_TCC_500ns       = ((uint8_t)0x03),
  POWERSTEP01_TCC_625ns       = ((uint8_t)0x04),
  POWERSTEP01_TCC_750ns       = ((uint8_t)0x05),
  POWERSTEP01_TCC_875ns       = ((uint8_t)0x06),
  POWERSTEP01_TCC_1000ns      = ((uint8_t)0x07),
  POWERSTEP01_TCC_1125ns      = ((uint8_t)0x08),
  POWERSTEP01_TCC_1250ns      = ((uint8_t)0x09),
  POWERSTEP01_TCC_1375ns      = ((uint8_t)0x0A),
  POWERSTEP01_TCC_1500ns      = ((uint8_t)0x0B),
  POWERSTEP01_TCC_1625ns      = ((uint8_t)0x0C),
  POWERSTEP01_TCC_1750ns      = ((uint8_t)0x0D),
  POWERSTEP01_TCC_1875ns      = ((uint8_t)0x0E),
  POWERSTEP01_TCC_2000ns      = ((uint8_t)0x0F),
  POWERSTEP01_TCC_2125ns      = ((uint8_t)0x10),
  POWERSTEP01_TCC_2250ns      = ((uint8_t)0x11),
  POWERSTEP01_TCC_2375ns      = ((uint8_t)0x12),
  POWERSTEP01_TCC_2500ns      = ((uint8_t)0x13),
  POWERSTEP01_TCC_2625ns      = ((uint8_t)0x14),
  POWERSTEP01_TCC_2750ns      = ((uint8_t)0x15),
  POWERSTEP01_TCC_2875ns      = ((uint8_t)0x16),
  POWERSTEP01_TCC_3000ns      = ((uint8_t)0x17),
  POWERSTEP01_TCC_3125ns      = ((uint8_t)0x18),
  POWERSTEP01_TCC_3250ns      = ((uint8_t)0x19),
  POWERSTEP01_TCC_3375ns      = ((uint8_t)0x1A),
  POWERSTEP01_TCC_3500ns      = ((uint8_t)0x1B),
  POWERSTEP01_TCC_3625ns      = ((uint8_t)0x1C),
  POWERSTEP01_TCC_3750ns      = ((uint8_t)0x1D),
  POWERSTEP01_TCC_3750ns_bis  = ((uint8_t)0x1E),
  POWERSTEP01_TCC_3750ns_ter  = ((uint8_t)0x1F)
} powerstep01_Tcc_t;

/// Igate options (GATECFG1 register of PowerStep01)
typedef enum {
  POWERSTEP01_IGATE_4mA     = (((uint8_t)0x00)<<5),
  POWERSTEP01_IGATE_4mA_Bis = (((uint8_t)0x01)<<5),
  POWERSTEP01_IGATE_8mA     = (((uint8_t)0x02)<<5),
  POWERSTEP01_IGATE_16mA    = (((uint8_t)0x03)<<5),
  POWERSTEP01_IGATE_24mA    = (((uint8_t)0x04)<<5),
  POWERSTEP01_IGATE_32mA    = (((uint8_t)0x05)<<5),
  POWERSTEP01_IGATE_64mA    = (((uint8_t)0x06)<<5),
  POWERSTEP01_IGATE_96mA    = (((uint8_t)0x07)<<5),
} powerstep01_Igate_t;

/// Turn off boost time (TBOOST field of GATECFG1 register of PowerStep01)
typedef enum {
  POWERSTEP01_TBOOST_0ns                = (((uint8_t)0x00)<<8),
  POWERSTEP01_TBOOST_62_5__83_3__125ns  = (((uint8_t)0x01)<<8),
  POWERSTEP01_TBOOST_125ns              = (((uint8_t)0x02)<<8),
  POWERSTEP01_TBOOST_250ns              = (((uint8_t)0x03)<<8),
  POWERSTEP01_TBOOST_375ns              = (((uint8_t)0x04)<<8),
  POWERSTEP01_TBOOST_500ns              = (((uint8_t)0x05)<<8),
  POWERSTEP01_TBOOST_750ns              = (((uint8_t)0x06)<<8),
  POWERSTEP01_TBOOST_1000ns             = (((uint8_t)0x07)<<8),
} powerstep01_Tboost_t;

/// External clock watchdog (WD_EN field of GATECFG1 register of PowerStep01)
typedef enum {
  POWERSTEP01_WD_EN_DISABLE   = ((uint16_t)0x0000),
  POWERSTEP01_WD_EN_ENABLE    = ((uint16_t) ((0x1) << 11))
} powerstep01_WdEn_t;

/// Blanking time (TBLANK field of GATECFG2 register of PowerStep01)
typedef enum {
  POWERSTEP01_TBLANK_125ns    = (((uint8_t)0x00)<<5),
  POWERSTEP01_TBLANK_250ns    = (((uint8_t)0x01)<<5),
  POWERSTEP01_TBLANK_375ns    = (((uint8_t)0x02)<<5),
  POWERSTEP01_TBLANK_500ns    = (((uint8_t)0x03)<<5),
  POWERSTEP01_TBLANK_625ns    = (((uint8_t)0x04)<<5),
  POWERSTEP01_TBLANK_750ns    = (((uint8_t)0x05)<<5),
  POWERSTEP01_TBLANK_875ns    = (((uint8_t)0x06)<<5),
  POWERSTEP01_TBLANK_1000ns   = (((uint8_t)0x07)<<5),
} powerstep01_TBlank_t;


/// Dead time (TDT field of GATECFG2 register of PowerStep01)
typedef enum {
  POWERSTEP01_TDT_125ns   = ((uint8_t)0x00),
  POWERSTEP01_TDT_250ns   = ((uint8_t)0x01),
  POWERSTEP01_TDT_375ns   = ((uint8_t)0x02),
  POWERSTEP01_TDT_500ns   = ((uint8_t)0x03),
  POWERSTEP01_TDT_625ns   = ((uint8_t)0x04),
  POWERSTEP01_TDT_750ns   = ((uint8_t)0x05),
  POWERSTEP01_TDT_875ns   = ((uint8_t)0x06),
  POWERSTEP01_TDT_1000ns  = ((uint8_t)0x07),
  POWERSTEP01_TDT_1125ns  = ((uint8_t)0x08),
  POWERSTEP01_TDT_1250ns  = ((uint8_t)0x09),
  POWERSTEP01_TDT_1375ns  = ((uint8_t)0x0A),
  POWERSTEP01_TDT_1500ns  = ((uint8_t)0x0B),
  POWERSTEP01_TDT_1625ns  = ((uint8_t)0x0C),
  POWERSTEP01_TDT_1750ns  = ((uint8_t)0x0D),
  POWERSTEP01_TDT_1875ns  = ((uint8_t)0x0E),
  POWERSTEP01_TDT_2000ns  = ((uint8_t)0x0F),
  POWERSTEP01_TDT_2125ns  = ((uint8_t)0x10),
  POWERSTEP01_TDT_2250ns  = ((uint8_t)0x11),
  POWERSTEP01_TDT_2375ns  = ((uint8_t)0x12),
  POWERSTEP01_TDT_2500ns  = ((uint8_t)0x13),
  POWERSTEP01_TDT_2625ns  = ((uint8_t)0x14),
  POWERSTEP01_TDT_2750ns  = ((uint8_t)0x15),
  POWERSTEP01_TDT_2875ns  = ((uint8_t)0x16),
  POWERSTEP01_TDT_3000ns  = ((uint8_t)0x17),
  POWERSTEP01_TDT_3125ns  = ((uint8_t)0x18),
  POWERSTEP01_TDT_3250ns  = ((uint8_t)0x19),
  POWERSTEP01_TDT_3375ns  = ((uint8_t)0x1A),
  POWERSTEP01_TDT_3500ns  = ((uint8_t)0x1B),
  POWERSTEP01_TDT_3625ns  = ((uint8_t)0x1C),
  POWERSTEP01_TDT_3750ns  = ((uint8_t)0x1D),
  POWERSTEP01_TDT_3875ns  = ((uint8_t)0x1E),
  POWERSTEP01_TDT_4000ns  = ((uint8_t)0x1F)
} powerstep01_Tdt_t;

/// masks for GATECFG2 register of PowerStep01
typedef enum {
  POWERSTEP01_GATECFG2_TDT      = ((uint8_t)0x1F),
  POWERSTEP01_GATECFG2_TBLANK   = ((uint8_t)0xE0)
} powerstep01_GateCfg2Masks_t;

/// Masks for CONFIG register of Powerstep01
typedef enum {
  POWERSTEP01_CONFIG_OSC_SEL      = ((uint16_t)0x0007),
  POWERSTEP01_CONFIG_EXT_CLK      = ((uint16_t)0x0008),
  POWERSTEP01_CONFIG_SW_MODE      = ((uint16_t)0x0010),
  POWERSTEP01_CONFIG_OC_SD        = ((uint16_t)0x0080),
  POWERSTEP01_CONFIG_UVLOVAL      = ((uint16_t)0x0100),
  POWERSTEP01_CONFIG_VCCVAL       = ((uint16_t)0x0200),
  // Masks specific for voltage mode
  POWERSTEP01_CONFIG_EN_VSCOMP    = ((uint16_t)0x0020),
  POWERSTEP01_CONFIG_F_PWM_DEC    = ((uint16_t)0x1C00),
  POWERSTEP01_CONFIG_F_PWM_INT    = ((uint16_t)0xE000),
  // Masks specific for current mode
  POWERSTEP01_CONFIG_TSW          = ((uint16_t)0x7C00),
  POWERSTEP01_CONFIG_PRED_EN      = ((uint16_t)0x8000)
} powerstep01_ConfigMasks_t;



/// Masks for CONFIG register of Powerstep01  (specific for current mode)
#define POWERSTEP01_CONFIG_EN_TQREG (POWERSTEP01_CONFIG_EN_VSCOMP)

/// Oscillator management (EXT_CLK and OSC_SEL fields of CONFIG register of Powerstep01)
typedef enum {
  POWERSTEP01_CONFIG_INT_16MHZ               = ((uint16_t)0x0000),
  POWERSTEP01_CONFIG_INT_16MHZ_OSCOUT_2MHZ   = ((uint16_t)0x0008),
  POWERSTEP01_CONFIG_INT_16MHZ_OSCOUT_4MHZ   = ((uint16_t)0x0009),
  POWERSTEP01_CONFIG_INT_16MHZ_OSCOUT_8MHZ   = ((uint16_t)0x000A),
  POWERSTEP01_CONFIG_INT_16MHZ_OSCOUT_16MHZ  = ((uint16_t)0x000B),
  POWERSTEP01_CONFIG_EXT_8MHZ_XTAL_DRIVE     = ((uint16_t)0x0004),
  POWERSTEP01_CONFIG_EXT_16MHZ_XTAL_DRIVE    = ((uint16_t)0x0005),
  POWERSTEP01_CONFIG_EXT_24MHZ_XTAL_DRIVE    = ((uint16_t)0x0006),
  POWERSTEP01_CONFIG_EXT_32MHZ_XTAL_DRIVE    = ((uint16_t)0x0007),
  POWERSTEP01_CONFIG_EXT_8MHZ_OSCOUT_INVERT  = ((uint16_t)0x000C),
  POWERSTEP01_CONFIG_EXT_16MHZ_OSCOUT_INVERT = ((uint16_t)0x000D),
  POWERSTEP01_CONFIG_EXT_24MHZ_OSCOUT_INVERT = ((uint16_t)0x000E),
  POWERSTEP01_CONFIG_EXT_32MHZ_OSCOUT_INVERT = ((uint16_t)0x000F)
} powerstep01_ConfigOscMgmt_t;

/// Oscillator management (EXT_CLK and OSC_SEL fields of CONFIG register of Powerstep01)
typedef enum {
  POWERSTEP01_CONFIG_SW_HARD_STOP = ((uint16_t)0x0000),
  POWERSTEP01_CONFIG_SW_USER      = ((uint16_t)0x0010)
} powerstep01_ConfigSwMode_t;

/// Voltage supply compensation enabling for voltage mode (EN_VSCOMP field of CONFIG register of Powerstep01)
typedef enum {
  POWERSTEP01_CONFIG_VS_COMP_DISABLE  = ((uint16_t)0x0000),
  POWERSTEP01_CONFIG_VS_COMP_ENABLE   = ((uint16_t)0x0020)
} powerstep01_ConfigEnVscomp_t;

/// External torque regulation enabling (EN_TQREG field of CONFIG register of Powerstep01)
typedef enum {
  POWERSTEP01_CONFIG_TQ_REG_TVAL_USED = ((uint16_t)0x0000),
  POWERSTEP01_CONFIG_TQ_REG_ADC_OUT   = ((uint16_t)0x0020)
} powerstep01_ConfigEnTqReg_t;

/// Overcurrent shutdown (OC_SD field of CONFIG register of Powerstep01)
typedef enum {
  POWERSTEP01_CONFIG_OC_SD_DISABLE  = ((uint16_t)0x0000),
  POWERSTEP01_CONFIG_OC_SD_ENABLE   = ((uint16_t)0x0080)
} powerstep01_ConfigOcSd_t;

/// UVLO thresholds (UVLOVAL field of CONFIG register of Powerstep01)
typedef enum {
  POWERSTEP01_CONFIG_UVLOVAL_LOW      = ((uint16_t)0x0000),
  POWERSTEP01_CONFIG_UVLOVAL_HIGH     = ((uint16_t)0x0100),
} powerstep01_ConfigUvLoVal_t;

/// Vcc voltage  (VCCVAL field of CONFIG register of Powerstep01)
typedef enum {
  POWERSTEP01_CONFIG_VCCVAL_7_5V    = ((uint16_t)0x0000),
  POWERSTEP01_CONFIG_VCCVAL_15V     = ((uint16_t)0x0200)
} powerstep01_ConfigVccVal_t;

/// PWM frequency division factor (F_PWM_INT field of CONFIG register of Powerstep01)
typedef enum {
  POWERSTEP01_CONFIG_PWM_DIV_1    = (((uint16_t)0x00)<<13),
  POWERSTEP01_CONFIG_PWM_DIV_2    = (((uint16_t)0x01)<<13),
  POWERSTEP01_CONFIG_PWM_DIV_3    = (((uint16_t)0x02)<<13),
  POWERSTEP01_CONFIG_PWM_DIV_4    = (((uint16_t)0x03)<<13),
  POWERSTEP01_CONFIG_PWM_DIV_5    = (((uint16_t)0x04)<<13),
  POWERSTEP01_CONFIG_PWM_DIV_6    = (((uint16_t)0x05)<<13),
  POWERSTEP01_CONFIG_PWM_DIV_7    = (((uint16_t)0x06)<<13)
} powerstep01_ConfigFPwmInt_t;

/// PWM frequency multiplication factor (F_PWM_DEC field of CONFIG register of Powerstep01)
typedef enum {
  POWERSTEP01_CONFIG_PWM_MUL_0_625  = (((uint16_t)0x00)<<10),
  POWERSTEP01_CONFIG_PWM_MUL_0_75   = (((uint16_t)0x01)<<10),
  POWERSTEP01_CONFIG_PWM_MUL_0_875  = (((uint16_t)0x02)<<10),
  POWERSTEP01_CONFIG_PWM_MUL_1      = (((uint16_t)0x03)<<10),
  POWERSTEP01_CONFIG_PWM_MUL_1_25   = (((uint16_t)0x04)<<10),
  POWERSTEP01_CONFIG_PWM_MUL_1_5    = (((uint16_t)0x05)<<10),
  POWERSTEP01_CONFIG_PWM_MUL_1_75   = (((uint16_t)0x06)<<10),
  POWERSTEP01_CONFIG_PWM_MUL_2      = (((uint16_t)0x07)<<10)
} powerstep01_ConfigFPwmDec_t;

/// Switching period  (TSW field of CONFIG register of Powerstep01)
typedef enum {
  POWERSTEP01_CONFIG_TSW_004us    =(((uint16_t)0x01)<<10),
  POWERSTEP01_CONFIG_TSW_008us    =(((uint16_t)0x02)<<10),
  POWERSTEP01_CONFIG_TSW_012us    =(((uint16_t)0x03)<<10),
  POWERSTEP01_CONFIG_TSW_016us    =(((uint16_t)0x04)<<10),
  POWERSTEP01_CONFIG_TSW_020us    =(((uint16_t)0x05)<<10),
  POWERSTEP01_CONFIG_TSW_024us    =(((uint16_t)0x06)<<10),
  POWERSTEP01_CONFIG_TSW_028us    =(((uint16_t)0x07)<<10),
  POWERSTEP01_CONFIG_TSW_032us    =(((uint16_t)0x08)<<10),
  POWERSTEP01_CONFIG_TSW_036us    =(((uint16_t)0x09)<<10),
  POWERSTEP01_CONFIG_TSW_040us    =(((uint16_t)0x0A)<<10),
  POWERSTEP01_CONFIG_TSW_044us    =(((uint16_t)0x0B)<<10),
  POWERSTEP01_CONFIG_TSW_048us    =(((uint16_t)0x0C)<<10),
  POWERSTEP01_CONFIG_TSW_052us    =(((uint16_t)0x0D)<<10),
  POWERSTEP01_CONFIG_TSW_056us    =(((uint16_t)0x0E)<<10),
  POWERSTEP01_CONFIG_TSW_060us    =(((uint16_t)0x0F)<<10),
  POWERSTEP01_CONFIG_TSW_064us    =(((uint16_t)0x10)<<10),
  POWERSTEP01_CONFIG_TSW_068us    =(((uint16_t)0x11)<<10),
  POWERSTEP01_CONFIG_TSW_072us    =(((uint16_t)0x12)<<10),
  POWERSTEP01_CONFIG_TSW_076us    =(((uint16_t)0x13)<<10),
  POWERSTEP01_CONFIG_TSW_080us    =(((uint16_t)0x14)<<10),
  POWERSTEP01_CONFIG_TSW_084us    =(((uint16_t)0x15)<<10),
  POWERSTEP01_CONFIG_TSW_088us    =(((uint16_t)0x16)<<10),
  POWERSTEP01_CONFIG_TSW_092us    =(((uint16_t)0x17)<<10),
  POWERSTEP01_CONFIG_TSW_096us    =(((uint16_t)0x18)<<10),
  POWERSTEP01_CONFIG_TSW_100us    =(((uint16_t)0x19)<<10),
  POWERSTEP01_CONFIG_TSW_104us    =(((uint16_t)0x1A)<<10),
  POWERSTEP01_CONFIG_TSW_108us    =(((uint16_t)0x1B)<<10),
  POWERSTEP01_CONFIG_TSW_112us    =(((uint16_t)0x1C)<<10),
  POWERSTEP01_CONFIG_TSW_116us    =(((uint16_t)0x1D)<<10),
  POWERSTEP01_CONFIG_TSW_120us    =(((uint16_t)0x1E)<<10),
  POWERSTEP01_CONFIG_TSW_124us    =(((uint16_t)0x1F)<<10)
} powerstep01_ConfigTsw_t;

/// Voltage supply compensation enabling for current mode(EN_PRED field of CONFIG register of Powerstep01)
typedef enum {
  POWERSTEP01_CONFIG_PRED_DISABLE =((uint16_t)0x0000),
  POWERSTEP01_CONFIG_PRED_ENABLE  =((uint16_t)0x8000)
} powerstep01_ConfigPredEn_t;

/// POWERSTEP01 power bridge output slew_rates options (POW_SR values for CONFIG register)
typedef enum {
  POWERSTEP01_CONFIG_SR_320V_us    =((uint16_t)0x0000),
  POWERSTEP01_CONFIG_SR_075V_us    =((uint16_t)0x0100),
  POWERSTEP01_CONFIG_SR_110V_us    =((uint16_t)0x0200),
  POWERSTEP01_CONFIG_SR_260V_us    =((uint16_t)0x0300)
} POWERSTEP01_CONFIG_POW_SR_t;

/// POWERSTEP01 Off time options (TOFF values for CONFIG register)
typedef enum {
  POWERSTEP01_CONFIG_TOFF_004us   = (((uint16_t) 0x01) << 10),
  POWERSTEP01_CONFIG_TOFF_008us   = (((uint16_t) 0x02) << 10),
  POWERSTEP01_CONFIG_TOFF_012us  = (((uint16_t) 0x03) << 10),
  POWERSTEP01_CONFIG_TOFF_016us  = (((uint16_t) 0x04) << 10),
  POWERSTEP01_CONFIG_TOFF_020us  = (((uint16_t) 0x05) << 10),
  POWERSTEP01_CONFIG_TOFF_024us  = (((uint16_t) 0x06) << 10),
  POWERSTEP01_CONFIG_TOFF_028us  = (((uint16_t) 0x07) << 10),
  POWERSTEP01_CONFIG_TOFF_032us  = (((uint16_t) 0x08) << 10),
  POWERSTEP01_CONFIG_TOFF_036us  = (((uint16_t) 0x09) << 10),
  POWERSTEP01_CONFIG_TOFF_040us  = (((uint16_t) 0x0A) << 10),
  POWERSTEP01_CONFIG_TOFF_044us  = (((uint16_t) 0x0B) << 10),
  POWERSTEP01_CONFIG_TOFF_048us  = (((uint16_t) 0x0C) << 10),
  POWERSTEP01_CONFIG_TOFF_052us  = (((uint16_t) 0x0D) << 10),
  POWERSTEP01_CONFIG_TOFF_056us  = (((uint16_t) 0x0E) << 10),
  POWERSTEP01_CONFIG_TOFF_060us  = (((uint16_t) 0x0F) << 10),
  POWERSTEP01_CONFIG_TOFF_064us  = (((uint16_t) 0x10) << 10),
  POWERSTEP01_CONFIG_TOFF_068us  = (((uint16_t) 0x11) << 10),
  POWERSTEP01_CONFIG_TOFF_072us  = (((uint16_t) 0x12) << 10),
  POWERSTEP01_CONFIG_TOFF_076us  = (((uint16_t) 0x13) << 10),
  POWERSTEP01_CONFIG_TOFF_080us  = (((uint16_t) 0x14) << 10),
  POWERSTEP01_CONFIG_TOFF_084us  = (((uint16_t) 0x15) << 10),
  POWERSTEP01_CONFIG_TOFF_088us  = (((uint16_t) 0x16) << 10),
  POWERSTEP01_CONFIG_TOFF_092us  = (((uint16_t) 0x17) << 10),
  POWERSTEP01_CONFIG_TOFF_096us  = (((uint16_t) 0x18) << 10),
  POWERSTEP01_CONFIG_TOFF_100us = (((uint16_t) 0x19) << 10),
  POWERSTEP01_CONFIG_TOFF_104us = (((uint16_t) 0x1A) << 10),
  POWERSTEP01_CONFIG_TOFF_108us = (((uint16_t) 0x1B) << 10),
  POWERSTEP01_CONFIG_TOFF_112us = (((uint16_t) 0x1C) << 10),
  POWERSTEP01_CONFIG_TOFF_116us = (((uint16_t) 0x1D) << 10),
  POWERSTEP01_CONFIG_TOFF_120us = (((uint16_t) 0x1E) << 10),
  POWERSTEP01_CONFIG_TOFF_124us = (((uint16_t) 0x1F) << 10)
} POWERSTEP01_CONFIG_TOFF_t;

/// Bit mask for STATUS Register of PowerStep01²
typedef enum {
  POWERSTEP01_STATUS_HIZ          = (((uint16_t)0x0001)),
  POWERSTEP01_STATUS_BUSY         = (((uint16_t)0x0002)),
  POWERSTEP01_STATUS_SW_F         = (((uint16_t)0x0004)),
  POWERSTEP01_STATUS_SW_EVN       = (((uint16_t)0x0008)),
  POWERSTEP01_STATUS_DIR          = (((uint16_t)0x0010)),
  POWERSTEP01_STATUS_MOT_STATUS   = (((uint16_t)0x0060)),
  POWERSTEP01_STATUS_CMD_ERROR    = (((uint16_t)0x0080)),
  POWERSTEP01_STATUS_STCK_MOD     = (((uint16_t)0x0100)),
  POWERSTEP01_STATUS_UVLO         = (((uint16_t)0x0200)),
  POWERSTEP01_STATUS_UVLO_ADC     = (((uint16_t)0x0400)),
  POWERSTEP01_STATUS_TH_STATUS    = (((uint16_t)0x1800)),
  POWERSTEP01_STATUS_OCD          = (((uint16_t)0x2000)),
  POWERSTEP01_STATUS_STALL_A      = (((uint16_t)0x4000)),
  POWERSTEP01_STATUS_STALL_B      = (((uint16_t)0x8000))
} powerstep01_StatusMasks_t;

/// Motor state (MOT_STATUS filed of STATUS register of PowerStep01)
typedef enum {
  POWERSTEP01_STATUS_MOT_STATUS_STOPPED       = (((uint16_t)0x0000)<<5),
  POWERSTEP01_STATUS_MOT_STATUS_ACCELERATION  = (((uint16_t)0x0001)<<5),
  POWERSTEP01_STATUS_MOT_STATUS_DECELERATION  = (((uint16_t)0x0002)<<5),
  POWERSTEP01_STATUS_MOT_STATUS_CONST_SPD     = (((uint16_t)0x0003)<<5)
} powerstep01_Status_t;

/// POWERSTEP01 STATUS register options
typedef enum {
  POWERSTEP01_STATUS_DIR_FORWARD = (((uint16_t) 0x0001) << 4),
  POWERSTEP01_STATUS_DIR_REVERSE = (((uint16_t) 0x0000) << 4)
} POWERSTEP01_STATUS_DIR_t;

/// Powerstep01 internal register addresses
typedef enum {
  POWERSTEP01_ABS_POS     = ((uint8_t)0x01),
  POWERSTEP01_EL_POS      = ((uint8_t)0x02),
  POWERSTEP01_MARK        = ((uint8_t)0x03),
  POWERSTEP01_SPEED       = ((uint8_t)0x04),
  POWERSTEP01_ACC         = ((uint8_t)0x05),
  POWERSTEP01_DEC         = ((uint8_t)0x06),
  POWERSTEP01_MAX_SPEED   = ((uint8_t)0x07),
  POWERSTEP01_MIN_SPEED   = ((uint8_t)0x08),
  POWERSTEP01_FS_SPD      = ((uint8_t)0x15),
  POWERSTEP01_KVAL_HOLD   = ((uint8_t)0x09),
  POWERSTEP01_KVAL_RUN    = ((uint8_t)0x0A),
  POWERSTEP01_KVAL_ACC    = ((uint8_t)0x0B),
  POWERSTEP01_KVAL_DEC    = ((uint8_t)0x0C),
  POWERSTEP01_INT_SPD     = ((uint8_t)0x0D),
  POWERSTEP01_ST_SLP      = ((uint8_t)0x0E),
  POWERSTEP01_FN_SLP_ACC  = ((uint8_t)0x0F),
  POWERSTEP01_FN_SLP_DEC  = ((uint8_t)0x10),
  POWERSTEP01_K_THERM     = ((uint8_t)0x11),
  POWERSTEP01_ADC_OUT     = ((uint8_t)0x12),
  POWERSTEP01_OCD_TH      = ((uint8_t)0x13),
  POWERSTEP01_STALL_TH    = ((uint8_t)0x14),
  POWERSTEP01_STEP_MODE   = ((uint8_t)0x16),
  POWERSTEP01_ALARM_EN    = ((uint8_t)0x17),
  POWERSTEP01_GATECFG1    = ((uint8_t)0x18),
  POWERSTEP01_GATECFG2    = ((uint8_t)0x19),
  POWERSTEP01_CONFIG      = ((uint8_t)0x1A),
  POWERSTEP01_STATUS      = ((uint8_t)0x1B)
} powerstep01_Registers_t;

/// Powerstep01 address of register TVAL_HOLD (Current mode only)
#define   POWERSTEP01_TVAL_HOLD  (POWERSTEP01_KVAL_HOLD )
/// Powerstep01 address of register TVAL_RUN (Current mode only)
#define   POWERSTEP01_TVAL_RUN   (POWERSTEP01_KVAL_RUN)
/// Powerstep01 address of register TVAL_HOLD (Current mode only)
#define   POWERSTEP01_TVAL_ACC   (POWERSTEP01_KVAL_ACC)
/// Powerstep01 address of register TVAL_DEC (Current mode only)
#define   POWERSTEP01_TVAL_DEC   (POWERSTEP01_KVAL_DEC)
/// Powerstep01 address of register T_FAST  (Current mode only)
#define   POWERSTEP01_T_FAST     (POWERSTEP01_ST_SLP)
/// Powerstep01 address of register TON_MIN  (Current mode only)
#define   POWERSTEP01_TON_MIN    (POWERSTEP01_FN_SLP_ACC)
/// Powerstep01 address of register TOFF_MIN (Current mode only)
#define   POWERSTEP01_TOFF_MIN   (POWERSTEP01_FN_SLP_DEC)

/// Powerstep01 application commands
typedef enum {
  POWERSTEP01_NOP               = ((uint8_t)0x00),
  POWERSTEP01_SET_PARAM         = ((uint8_t)0x00),
  POWERSTEP01_GET_PARAM         = ((uint8_t)0x20),
  POWERSTEP01_RUN               = ((uint8_t)0x50),
  POWERSTEP01_STEP_CLOCK        = ((uint8_t)0x58),
  POWERSTEP01_MOVE              = ((uint8_t)0x40),
  POWERSTEP01_GO_TO             = ((uint8_t)0x60),
  POWERSTEP01_GO_TO_DIR         = ((uint8_t)0x68),
  POWERSTEP01_GO_UNTIL          = ((uint8_t)0x82),
  POWERSTEP01_GO_UNTIL_ACT_CPY  = ((uint8_t)0x8A),
  POWERSTEP01_RELEASE_SW        = ((uint8_t)0x92),
  POWERSTEP01_GO_HOME           = ((uint8_t)0x70),
  POWERSTEP01_GO_MARK           = ((uint8_t)0x78),
  POWERSTEP01_RESET_POS         = ((uint8_t)0xD8),
  POWERSTEP01_RESET_DEVICE      = ((uint8_t)0xC0),
  POWERSTEP01_SOFT_STOP         = ((uint8_t)0xB0),
  POWERSTEP01_HARD_STOP         = ((uint8_t)0xB8),
  POWERSTEP01_SOFT_HIZ          = ((uint8_t)0xA0),
  POWERSTEP01_HARD_HIZ          = ((uint8_t)0xA8),
  POWERSTEP01_GET_STATUS        = ((uint8_t)0xD0),
  POWERSTEP01_RESERVED_CMD1     = ((uint8_t)0xEB),
  POWERSTEP01_RESERVED_CMD2     = ((uint8_t)0xF8)
} powerstep01_Commands_t;

/* Exported macro ------------------------------------------------------------*/

/** @defgroup Powerstep01_Exported_Macros
  * @{
  */

/// Macro for Speed conversion (step/s to parameter) range 0 to 15625 steps/s
#define Speed_Steps_to_Par(steps) ((uint32_t)(((steps)*67.108864)+0.5))
/// Macro for Speed conversion (parameter to step/s), range 0 to 15625 steps/s
#define Speed_Par_to_Steps(Par) ((uint32_t)((Par)*0.01490116119))

/// Macro for acc/Dec rates conversion (step/s^2 to parameter), range 14.55 to 59590 steps/s2
#define AccDec_Steps_to_Par(steps) ((uint16_t)(((steps)*0.068719476736)+0.5))
/// Macro for acc/Dec rates conversion (parameter to step/s^2)
#define AccDec_Par_to_Steps(Par) ((uint16_t)((Par)*14.5519152283))


/// Macro for max Speed conversion (step/s to parameter), range 15.25 to 15610 steps/s
#define MaxSpd_Steps_to_Par(steps) ((uint16_t)(((steps)*0.065536)+0.5))
/// Macro for max Speed conversion (parameter to step/s)
#define MaxSpd_Par_to_Steps(Par) ((uint16_t)((Par)*15,258789))

/// Macro for min Speed conversion (step/s to parameter), range 0 to 976.3 steps/s */
#define MinSpd_Steps_to_Par(steps) ((uint16_t)(((steps)*4.194304)+0.5))
/// Macro for min Speed conversion (parameter to step/s)
#define MinSpd_Par_to_Steps(Par) ((uint16_t)((Par)*0.238418579))

/// Macro for full Step Speed conversion (step/s to parameter), range 7.63 to 15625 steps/s
#define FSSpd_Steps_to_Par(steps) ((uint16_t)(((steps)*0.065536)+0.5))
/// Macro for full Step Speed conversion (parameter to step/s)
#define FSSpd_Par_to_Steps(Par) ((uint16_t)((Par)*15,258789))

// Macros for voltage mode
/// Macro for intersect Speed conversion, range 0 to 3906 steps/s
#define IntSpd_Steps_to_Par(steps) ((uint16_t)(((steps)*16.777216)+0.5))
/// Macro for KVAL conversions, range 0.4% to 99.6%
#define Kval_Perc_to_Par(perc) ((uint8_t)(((perc)/0.390625)+0.5))
/// Macro for BEMF compensation slopes, range 0 to 0.4% s/step
#define BEMF_Slope_Perc_to_Par(perc) ((uint8_t)(((perc)/0.00156862745098)+0.5))
/// Macro for K_THERM compensation conversion, range 1 to 1.46875
#define KTherm_to_Par(KTherm) ((uint8_t)(((KTherm - 1)/0.03125)+0.5))
/// Macro for stall Threshold conversion, range 31.25mV to 1000mV
#define StallTh_to_Par(StallTh) ((uint8_t)(((StallTh - 31.25)/31.25)+0.5))

// Macros for current mode
/// Macro for torque regulation DAC current conversion, range 7.8mV to 1000mV
#define Tval_Current_to_Par(Tval) ((uint8_t)(((Tval - 7.8)/7.8)+0.5))
/// Macro for minimum time conversion, range 0.5us to 64us
#define Tmin_Time_to_Par(Tmin) ((uint8_t)(((Tmin - 0.5)*2)+0.5))


/// POWERSTEP01 library class
class POWERSTEP01 {
  public:
    // constructor:
    POWERSTEP01();
    
    /// @defgroup group1 DEVICE control functions
    ///@{
    void AttachFlagInterrupt(void (*callback)(void));     //Attach a user callback to the flag Interrupt
    void Begin(uint8_t nbDEVICEs);                        //Start the POWERSTEP01 library
    uint16_t GetAcceleration(uint8_t DEVICEId);           //Return the acceleration in pps^2
    uint16_t GetCurrentSpeed(uint8_t DEVICEId);           //Return the current speed in pps
    uint16_t GetDeceleration(uint8_t DEVICEId);           //Return the deceleration in pps^2
    uint8_t GetFwVersion(void);                           //Return the FW version
    int32_t GetMark(uint8_t DEVICEId);                    //Return the mark position
    uint16_t GetMaxSpeed(uint8_t DEVICEId);               //Return the max speed in pps
    uint16_t GetMinSpeed(uint8_t DEVICEId);               //Return the min speed in pps
    int32_t GetPosition(uint8_t DEVICEId);                //Return the ABS_POSITION (32b signed)
    void CmdGoHome(uint8_t DEVICEId);                        //Move to the home position
    void CmdGoMark(uint8_t DEVICEId);                        //Move to the Mark position
    void CmdGoTo(uint8_t DEVICEId, int32_t abs_pos);  //Go to the specified position
    void CmdHardStop(uint8_t DEVICEId);                      //Stop the motor and disable the power bridge
    void CmdMove(uint8_t deviceId, motorDir_t direction, uint32_t n_step);
    void CmdSetHome(uint8_t DEVICEId);                          //Set current position to be the home position
    void CmdSetMark(uint8_t DEVICEId);                          //Set current position to be the Markposition
    void CmdSoftStop(uint8_t DEVICEId);                         //Progressively stops the motor
    void WaitWhileActive(uint8_t DEVICEId);                  //Wait for the DEVICE state becomes Inactive
    void CmdRun(uint8_t deviceId, motorDir_t direction, uint32_t speed);
    ///@}
    
    /// @defgroup group2 POWERSTEP01 control functions
    ///@{
    uint32_t CmdGetParam(uint8_t DEVICEId,          //Send the POWERSTEP01_GET_PARAM command
                                 powerstep01_Registers_t param);
    uint16_t CmdGetStatus(uint8_t DEVICEId);        // Send the POWERSTEP01_GET_STATUS command
    void CmdNop(uint8_t DEVICEId);                  //Send the POWERSTEP01_NOP command
    void CmdSetParam(uint8_t deviceId,              //Send the POWERSTEP01_SET_PARAM command
                             uint32_t param,
                             uint32_t value);
    uint16_t ReadStatusRegister(uint8_t DEVICEId);  // Read the POWERSTEP01_STATUS register without
                                                    // clearing the flags
    void Reset(void);                               //Set the POWERSTEP01 reset pin
    void ReleaseReset(void);                        //Release the POWERSTEP01 reset pin
    void SelectStepMode(uint8_t deviceId, motorStepMode_t stepMode);
    ///@}
    
    /// @defgroup group3 Delay functions
    ///@{
    /// @brief Required when 3 POWERSTEP01 DEVICEs are used
    /// to avoid conflicting depencieswith wiring.c 
    /// (redefinition of ISR(TIMER0_OVF_vect).  
    /// When only 2 POWERSTEP01 DEVICEs are used, prefer the use
    /// of standard Arduino functions (delay, delayMicroseconds).
    static void WaitMs(uint16_t msDelay); // Wait for a delay in ms
    static void WaitUs(uint16_t usDelay); // Wait for a delay in us
    ///@}
        
    /// @defgroup group4 Functions for timer ISRs only
    /// @brief To be used inside the library by the timer ISRs only 
    /// Must not be used elsewhere.
    ///@{
    static class POWERSTEP01 *GetInstancePtr(void);
    ///@}
    


  private:
    static void FlagInterruptHandler(void);
    static void BusyInterruptHandler(void);

    void SetRegisterToPredefinedValues(uint8_t deviceId);
    void WriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte);    
    
    int32_t ConvertPosition(uint32_t abs_position_reg);
    void ErrorHandler(uint16_t error);
    void SendCommand(uint8_t DEVICEId, uint8_t param);
    void SendCommand(uint8_t deviceId, uint8_t param, uint32_t value);
    void CmdHardHiZ(uint8_t deviceId);
    void CmdResetPos(uint8_t deviceId);
    bool IsDeviceBusy(uint8_t deviceId);
    // variable members        
    static volatile class POWERSTEP01 *instancePtr;
    static volatile void(*flagInterruptCallback)(void);
    static volatile void(*busyInterruptCallback)(void);
    static volatile bool isrFlag;
    static volatile bool spiPreemtionByIsr;
    static volatile uint8_t numberOfDevices;
    static uint8_t spiTxBursts[POWERSTEP01_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
    static uint8_t spiRxBursts[POWERSTEP01_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];

    motorDrv_t* Powerstep01_GetMotorHandle(void);                  //Return handle of the motor driver handle

};

#ifdef _DEBUG_POWERSTEP01
uint16_t GetFreeRam (void);
#endif

#endif /* #ifndef __POWERSTEP01_H_INCLUDED */
