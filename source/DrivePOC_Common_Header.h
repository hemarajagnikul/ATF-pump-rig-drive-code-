/**
 ******************************************************************************
 * @file           : DrivePOC_Common_Header.h
 * @brief          : Common header containing structures and enumerations
 *                   used across the entire project
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
#include "DrivePOC_Controller_NXP.h"

/** Motor control libraries headers */
#include "mlib_FP.h"
#include "gflib_FP.h"
#include "gdflib_FP.h"
#include "gmclib_FP.h"
#include "amclib_FP.h"


/*******************************************************************************
 * Structures and Enumerations
 ******************************************************************************/
/**
 * Main state machine - States
 * */
typedef enum Main_SM_States
{
	/*! Initialization state of Main State machine */
    MAIN_SM_INIT = 0,
	/*! Fault state of Main State machine */
	MAIN_SM_FAULT  = 1,
	/*! Stop state of Main State machine */
	MAIN_SM_STOP  = 2,
	/*! Run state of Main State machine */
	MAIN_SM_RUN   = 3
} Main_SM_States;

/**
 * Motor state machine - Run sub-states
 * */
typedef enum Motor_SM_States
{
	/*! Calibration state of Motor State machine */
    MOTOR_SM_CALIB     = 0,
	/*! Ready state of Motor State machine */
    MOTOR_SM_READY     = 1,
	/*! Start up state of Motor State machine */
	MOTOR_SM_START	   = 2,
	/*! Spin state of Motor State machine */
    MOTOR_SM_SPIN      = 3,
	/*! Decelerate state of Motor State machine */
	MOTOR_SM_DECELERATE =4,
	//motor_run =0,

} Motor_SM_States;

/**
 * Main State machine's control signals for state transition
 * */
typedef struct Main_SM_ControlSig
{
	bool main_sm_ctrl_init_done; /**< Control signal indicating Initialization of Main State machine */
	bool main_sm_ctrl_fault;     /**< Control signal indicating Fault state of Main State machine */
	bool main_sm_ctrl_stop;      /**< Control signal indicating Stop state of Main State machine */
	bool main_sm_ctrl_run;       /**< Control signal indicating Run state of Main State machine */
} Main_SM_ControlSig;

/**
 * Motor State machine's control signals for state transition
 * */
typedef struct Motor_SM_ControlSig
{
	bool calib;   /**< Control signal indicating ADC calibration of Motor State machine */
	bool ready;   /**< Control signal indicating Speed command of Motor State machine */
	bool start_ok; /**< Control signal indicating Start up is done*/
	bool spin;  /**< Control signal indicating Startup OK status of Motor State machine */
} Motor_SM_ControlSig;

/**
 * Structure for V/F Control Open Loop
 */
typedef struct mcs_acim_open_loop_str{
	GMCLIB_3COOR_T_F16 s_dutyabc_f16;
	GMCLIB_3COOR_T_FLT s_dutyabc_flt;
	GMCLIB_3COOR_T_FLT s_iabc;
	GMCLIB_3COOR_T_FLT s_vabc;
	float_t fltudcbus;
	float_t fltidcbus;
}mcs_acim_open_loop_str;

typedef struct __attribute__((__packed__)){
	uint8_t SOF;
	uint8_t pkt_id;
	uint16_t data;
	uint8_t checksum;
}command_packet_t;

typedef enum
{
	C_Motor_command =0x11,
	Switching_freq=0x12,
	Ramp_up_time=0x13,
	Ramp_down_time=0x14,
	Set_speed=0x15,
	Modulation_scheme=0x17,
    Drive_Input_Voltage = 0x31,
} ;
