/**
 ******************************************************************************
 * @file           : DrivePOC_FaultHandler.h
 * @brief          : Functionalities pertaining to Handling faults in the complete drive system
 *
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


/**
 * @defgroup group4 Fault Handler
 * @brief Fault Handler group
 * @details Group pertaining to all Memory and storage operations
 * of the Drive POC Controller implementation
 * @{
 */
/*******************************************************************************
 * Header inclusions
 ******************************************************************************/
#include "fsl_debug_console.h"
#include "mcdrv_frdmkv31f.h"
/*******************************************************************************
 * Function definitions
 ******************************************************************************/

/**
 * @brief Function definition that will make the Duty Cycle of all the switches to zero
 * @param None
 * @return None
 * */
void Get_Fault_Duty_Cycle(void);
//uint8_t offset_cal(float_t svdpwm);
float_t offset_cal(float_t msa, float_t msb, float_t msc, float_t KO0, float_t KO1);
//float_t romesq_cal (float i_ph_a, float i_ph_b, float i_ph_c);
/**@}*/
//extern float i_ph_rms;
