/**
 ******************************************************************************
 * @file           : DrivePOC_Controller_NXP.h
 * @brief          : Main header file for implementing the Drive POC Controller
 * 					 using NXP microcontroller on the FRDM-KV31F eval board
 * @author         : Sreedhar, Sangeerth
 * @company        : Agnikul Cosmos Private Limited
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) Agnikul Cosmos Private Limited
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
*/


/*******************************************************************************
 * Header inclusions
 ******************************************************************************/
#include "DrivePOC_MemHandler.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PIT_IRQ_HANDLER PIT0_IRQHandler /**< Defining the peripheral ID of PIT */
#define PIT_IRQ_ID      PIT0_IRQn       /**< Defining the IRQ ID of PIT */
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk) /**< Source clock for PIT driver */
#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk) /**< Source clock for FTM */
extern float SWITCHING_FREQUENCY;
extern float SWITCHING_TIME;
extern float M;
extern float LUT_FREQUENCY;
extern float FREQ_NEEDED;
extern float RAMP_TIME;
extern float SWITCHINGTIME;
extern float DECELERATION_TIME;
extern float FREQ_MAP;
extern float MAX_NUM_OF_LUT_SIZE;
extern float START_UP_OK;
extern float STOP_OK;
extern float accel_del_freq;
extern float decel_del_freq;
extern float f_freq_map;
extern float DC_cap_link_volts;
extern int8_t mod_scheme_select;
//!@name DEFINES FOR DIFFERENT TIMES OF EXPECTED INTERRUPTS
//!@{
#define LTC_FREQ        20000  /**< Every 1 sec the LTC Interrupt will be working irrespective of the Switching Frequency */
#define PRINTING_FREQ   100000U /**< Factor for 1000 ms */
#define SLOW_LOOP_TIME 400U
//!@}

/*******************************************************************************
 * Function prototypes
 ******************************************************************************/
/**
 * @brief Function definition to configure the Periodic Interrupt Timer (PIT)
 * @detail The function calls the PIT Handler which runs every 100us or 50us or 33us or 25us or 20 us based on the Switching Frequency
 * For 10kHz - 100 us
 * For 20kHz -  50 us
 * For 30kHz -  33 us
 * For 40kHz -  25 us
 * For 50kHz -  20 us
 * @param  None
 * @return None
 *
 * */
void PIT_Configuration(void);
