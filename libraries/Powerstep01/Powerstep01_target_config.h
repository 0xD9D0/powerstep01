/******************************************************//**
 * @file    POWERSTEP01_target_config.h
 * @version V1.0
 * @date    March 3, 2014
 * @brief   Predefines values for the POWERSTEP01 registers
 * and for the shields parameters
  *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 **********************************************************/

#ifndef __POWERSTEP01_TARGET_CONFIG_H
#define __POWERSTEP01_TARGET_CONFIG_H

/// The maximum number of shields in the daisy chain
#define MAX_NUMBER_OF_SHIELDS                 (3)

/************************ Speed Profile  *******************************/

/// Acceleration rate in step/s2 for shield 0 (must be greater than 0)
#define POWERSTEP01_CONF_PARAM_ACC_SHIELD_0        (160)
/// Acceleration rate in step/s2 for shield 1 (must be greater than 0)
#define POWERSTEP01_CONF_PARAM_ACC_SHIELD_1        (160)
/// Acceleration rate in step/s2 for shield 2 (must be greater than 0)
#define POWERSTEP01_CONF_PARAM_ACC_SHIELD_2        (160)

/// Deceleration rate in step/s2 for shield 0 (must be greater than 0)
#define POWERSTEP01_CONF_PARAM_DEC_SHIELD_0        (160)
/// Deceleration rate in step/s2 for shield 1 (must be greater than 0)
#define POWERSTEP01_CONF_PARAM_DEC_SHIELD_1        (160)
/// Deceleration rate in step/s2 for shield 2 (must be greater than 0)
#define POWERSTEP01_CONF_PARAM_DEC_SHIELD_2        (160)

/// Maximum speed in step/s for shield 0 (30 step/s < Maximum speed <= 10 000 step/s )
#define POWERSTEP01_CONF_PARAM_MAX_SPEED_SHIELD_0  (1600)
/// Maximum speed in step/s for shield 1 (30 step/s < Maximum speed <= 10 000 step/s )
#define POWERSTEP01_CONF_PARAM_MAX_SPEED_SHIELD_1  (1600)
/// Maximum speed in step/s for shield 2 (30 step/s < Maximum speed <= 10 000 step/s )
#define POWERSTEP01_CONF_PARAM_MAX_SPEED_SHIELD_2  (1600)
/// Minimum speed in step/s for shield 0 (30 step/s <= Minimum speed < 10 000 step/s)
#define POWERSTEP01_CONF_PARAM_MIN_SPEED_SHIELD_0  (800)
/// Minimum speed in step/s for shield 1 (30 step/s <= Minimum speed < 10 000 step/s)
#define POWERSTEP01_CONF_PARAM_MIN_SPEED_SHIELD_1  (800)
/// Minimum speed in step/s for shield 2 (30 step/s <= Minimum speed < 10 000 step/s)
#define POWERSTEP01_CONF_PARAM_MIN_SPEED_SHIELD_2  (800)


/************************ Phase Current Control *******************************/

// Current value that is assigned to the torque regulation DAC
/// TVAL register value for shield 0 (range 31.25mA to 4000mA)
#define POWERSTEP01_CONF_PARAM_TVAL_SHIELD_0  (250)
/// TVAL register value for shield 1 (range 31.25mA to 4000mA)
#define POWERSTEP01_CONF_PARAM_TVAL_SHIELD_1  (250)
/// TVAL register value for shield 2 (range 31.25mA to 4000mA)
#define POWERSTEP01_CONF_PARAM_TVAL_SHIELD_2  (250)

/// Fall time value (T_FAST field of T_FAST register) for shield 0 (range 2us to 32us)
#define POWERSTEP01_CONF_PARAM_FAST_STEP_SHIELD_0  (POWERSTEP01_FAST_STEP_12us)
/// Fall time value (T_FAST field of T_FAST register) for shield 1 (range 2us to 32us)
#define POWERSTEP01_CONF_PARAM_FAST_STEP_SHIELD_1  (POWERSTEP01_FAST_STEP_12us)
/// Fall time value (T_FAST field of T_FAST register) for shield 2 (range 2us to 32us)
#define POWERSTEP01_CONF_PARAM_FAST_STEP_SHIELD_2  (POWERSTEP01_FAST_STEP_12us)

/// Maximum fast decay time (T_OFF field of T_FAST register) for shield 0 (range 2us to 32us)
#define POWERSTEP01_CONF_PARAM_TOFF_FAST_SHIELD_0  (POWERSTEP01_TOFF_FAST_8us)
/// Maximum fast decay time (T_OFF field of T_FAST register) for shield 1 (range 2us to 32us)
#define POWERSTEP01_CONF_PARAM_TOFF_FAST_SHIELD_1  (POWERSTEP01_TOFF_FAST_8us)
/// Maximum fast decay time (T_OFF field of T_FAST register) for shield 2 (range 2us to 32us)
#define POWERSTEP01_CONF_PARAM_TOFF_FAST_SHIELD_2  (POWERSTEP01_TOFF_FAST_8us)

/// Minimum ON time (TON_MIN register) for shield 0 (range 0.5us to 64us)
#define POWERSTEP01_CONF_PARAM_TON_MIN_SHIELD_0 (3)
/// Minimum ON time (TON_MIN register) for shield 1 (range 0.5us to 64us)
#define POWERSTEP01_CONF_PARAM_TON_MIN_SHIELD_1 (3)
/// Minimum ON time (TON_MIN register) for shield 2 (range 0.5us to 64us)
#define POWERSTEP01_CONF_PARAM_TON_MIN_SHIELD_2 (3)

/// Minimum OFF time (TOFF_MIN register) for shield 0 (range 0.5us to 64us)
#define POWERSTEP01_CONF_PARAM_TOFF_MIN_SHIELD_0 (21)
/// Minimum OFF time (TOFF_MIN register) for shield 1 (range 0.5us to 64us)
#define POWERSTEP01_CONF_PARAM_TOFF_MIN_SHIELD_1 (21)
/// Minimum OFF time (TOFF_MIN register) for shield 2 (range 0.5us to 64us)
#define POWERSTEP01_CONF_PARAM_TOFF_MIN_SHIELD_2 (21)

/******************************* Others ***************************************/

/// Overcurrent threshold settings for shield 0 (OCD_TH register)
#define POWERSTEP01_CONF_PARAM_OCD_TH_SHIELD_0  (POWERSTEP01_OCD_TH_750mA)
/// Overcurrent threshold settings for shield 1 (OCD_TH register)
#define POWERSTEP01_CONF_PARAM_OCD_TH_SHIELD_1  (POWERSTEP01_OCD_TH_750mA)
/// Overcurrent threshold settings for shield 2 (OCD_TH register)
#define POWERSTEP01_CONF_PARAM_OCD_TH_SHIELD_2  (POWERSTEP01_OCD_TH_750mA)

/// Alarm settings for shield 0 (ALARM_EN register)
#define POWERSTEP01_CONF_PARAM_ALARM_EN_SHIELD_0  (POWERSTEP01_ALARM_EN_OVERCURRENT |\
                                                POWERSTEP01_ALARM_EN_THERMAL_SHUTDOWN |\
                                                POWERSTEP01_ALARM_EN_THERMAL_WARNING |\
                                                POWERSTEP01_ALARM_EN_UNDERVOLTAGE |\
                                                POWERSTEP01_ALARM_EN_SW_TURN_ON |\
                                                POWERSTEP01_ALARM_EN_WRONG_NPERF_CMD)

///Alarm settings for shield 1 (ALARM_EN register)
#define POWERSTEP01_CONF_PARAM_ALARM_EN_SHIELD_1  (POWERSTEP01_ALARM_EN_OVERCURRENT |\
                                                POWERSTEP01_ALARM_EN_THERMAL_SHUTDOWN |\
                                                POWERSTEP01_ALARM_EN_THERMAL_WARNING |\
                                                POWERSTEP01_ALARM_EN_UNDERVOLTAGE |\
                                                POWERSTEP01_ALARM_EN_SW_TURN_ON |\
                                                POWERSTEP01_ALARM_EN_WRONG_NPERF_CMD)

/// Alarm settings for shield 2 (ALARM_EN register)
#define POWERSTEP01_CONF_PARAM_ALARM_EN_SHIELD_2  (POWERSTEP01_ALARM_EN_OVERCURRENT |\
                                                POWERSTEP01_ALARM_EN_THERMAL_SHUTDOWN |\
                                                POWERSTEP01_ALARM_EN_THERMAL_WARNING |\
                                                POWERSTEP01_ALARM_EN_UNDERVOLTAGE |\
                                                POWERSTEP01_ALARM_EN_SW_TURN_ON |\
                                                POWERSTEP01_ALARM_EN_WRONG_NPERF_CMD)

/// Step selection settings for shield 0 (STEP_SEL field of STEP_MODE register)
#define POWERSTEP01_CONF_PARAM_STEP_SEL_SHIELD_0  (POWERSTEP01_STEP_SEL_1_16)
/// Step selection settings for shield 1 (STEP_SEL field of STEP_MODE register)
#define POWERSTEP01_CONF_PARAM_STEP_SEL_SHIELD_1  (POWERSTEP01_STEP_SEL_1_16)
/// Step selection settings for shield 2 (STEP_SEL field of STEP_MODE register)
#define POWERSTEP01_CONF_PARAM_STEP_SEL_SHIELD_2  (POWERSTEP01_STEP_SEL_1_16)

/// Synch. selection settings for shield 0 (SYNC_SEL field of STEP_MODE register)
#define POWERSTEP01_CONF_PARAM_SYNC_SEL_SHIELD_0  (POWERSTEP01_SYNC_SEL_1_2)
/// Synch. selection settings for shield 1 (SYNC_SEL field of STEP_MODE register)
#define POWERSTEP01_CONF_PARAM_SYNC_SEL_SHIELD_1  (POWERSTEP01_SYNC_SEL_1_2)
/// Synch. selection settings for shield 2 (SYNC_SEL field of STEP_MODE register)
#define POWERSTEP01_CONF_PARAM_SYNC_SEL_SHIELD_2  (POWERSTEP01_SYNC_SEL_1_2)

/// Target Swicthing Period for shield 0 (field TOFF of CONFIG register)
#define POWERSTEP01_CONF_PARAM_TOFF_SHIELD_0  (POWERSTEP01_CONFIG_TOFF_044us)
/// Target Swicthing Period for shield 1 (field TOFF of CONFIG register)
#define POWERSTEP01_CONF_PARAM_TOFF_SHIELD_1  (POWERSTEP01_CONFIG_TOFF_044us)
/// Target Swicthing Period for shield 2 (field TOFF of CONFIG register)
#define POWERSTEP01_CONF_PARAM_TOFF_SHIELD_2  (POWERSTEP01_CONFIG_TOFF_044us)

/// Slew rate for shield 0 (POW_SR field of CONFIG register)
#define POWERSTEP01_CONF_PARAM_SR_SHIELD_0  (POWERSTEP01_CONFIG_SR_320V_us)
/// Slew rate for shield 1 (POW_SR field of CONFIG register)
#define POWERSTEP01_CONF_PARAM_SR_SHIELD_1  (POWERSTEP01_CONFIG_SR_320V_us)
/// Slew rate for shield 2 (POW_SR field of CONFIG register)
#define POWERSTEP01_CONF_PARAM_SR_SHIELD_2  (POWERSTEP01_CONFIG_SR_320V_us)

/// Over current shutwdown enabling for shield 0 (OC_SD field of CONFIG register)
#define POWERSTEP01_CONF_PARAM_OC_SD_SHIELD_0  (POWERSTEP01_CONFIG_OC_SD_ENABLE)
/// Over current shutwdown enabling for shield 1 (OC_SD field of CONFIG register)
#define POWERSTEP01_CONF_PARAM_OC_SD_SHIELD_1  (POWERSTEP01_CONFIG_OC_SD_ENABLE)
/// Over current shutwdown enabling for shield 2 (OC_SD field of CONFIG register)
#define POWERSTEP01_CONF_PARAM_OC_SD_SHIELD_2  (POWERSTEP01_CONFIG_OC_SD_ENABLE)

/// Torque regulation method for shield 0 (EN_TQREG field of CONFIG register)
#define POWERSTEP01_CONF_PARAM_TQ_REG_SHIELD_0  (POWERSTEP01_CONFIG_EN_TQREG_TVAL_USED)
///Torque regulation method for shield 1 (EN_TQREG field of CONFIG register)
#define POWERSTEP01_CONF_PARAM_TQ_REG_SHIELD_1  (POWERSTEP01_CONFIG_EN_TQREG_TVAL_USED)
/// Torque regulation method for shield 2 (EN_TQREG field of CONFIG register)
#define POWERSTEP01_CONF_PARAM_TQ_REG_SHIELD_2  (POWERSTEP01_CONFIG_EN_TQREG_TVAL_USED)

/// Clock setting for shield 0 (OSC_CLK_SEL field of CONFIG register)
#define POWERSTEP01_CONF_PARAM_CLOCK_SETTING_SHIELD_0  (POWERSTEP01_CONFIG_INT_16MHZ)
/// Clock setting for shield 1 (OSC_CLK_SEL field of CONFIG register)
#define POWERSTEP01_CONF_PARAM_CLOCK_SETTING_SHIELD_1  (POWERSTEP01_CONFIG_INT_16MHZ)
/// Clock setting for shield 2 (OSC_CLK_SEL field of CONFIG register)
#define POWERSTEP01_CONF_PARAM_CLOCK_SETTING_SHIELD_2  (POWERSTEP01_CONFIG_INT_16MHZ)

#endif /* __Powerstep01_TARGET_CONFIG_H */
