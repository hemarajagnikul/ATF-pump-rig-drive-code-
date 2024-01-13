/**
 ******************************************************************************
 * @file           : DrivePOC_Control_Loop.h
 * @brief          : Functionalities pertaining to the V/F controls-Algorithm's
 *                   state machines (Main & Motor)
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
 * @defgroup group3 V/F Control Algorithm
 * @brief V/F Control Algorithm group
 * @details Group pertaining to all functionalities of the V/F Control Algorithm
 * in the Drive POC Controller implementation
 * @{
 */

/*******************************************************************************
 * Header inclusions
 ******************************************************************************/



/*******************************************************************************
 * Function prototypes
 ******************************************************************************/


/**
 * @brief Function definition corresponding to fault indicated by PTC 150 Thermocouple
 * @param  None
 * @return None
 *
 * */
extern int motor_run;
extern int get_out;
void PTC_FaultDiag(void);


/**
 * @brief Function definition corresponding to the Fault diagnosis
 * @param  None
 * @return None
 *
 * */
void System_FaultDiag(void);


/**
 * @brief Function definition corresponding to the main state machine of V/F control algorithm
 * @param  None
 * @return GMCLIB_3COOR_T_FLT dutyCycles
 *
 * • Fault—the system detected a fault condition and waits until it is cleared.
   • Init—initialization of variables.
   • Stop—the system is initialized and waiting for the Run command.
   • Run—the system is running; it can be stopped by the Stop command.
 *
 * */
GMCLIB_3COOR_T_FLT Open_Loop_MainStateMC(void);

/**
 * @brief Function definition corresponding to the Motor state machine of V/F Control algorithm
 * @param  None
 * @return None
 *
 * @details The motor state machines are based on the main state machine structure.
 * The Run state sub-states are added on top of the main structure to control
 * the motors properly
 *
 * */
void Open_Loop_MotorStateMC(void);


/**
 * @brief Function definition corresponding to the Fast loop of the V/F control algorithm
 * @param  None
 * @return GMCLIB_3COOR_T_FLT dutyCycles
 *
 * */
GMCLIB_3COOR_T_FLT Open_Loop_Control(void);


/**
 *@brief Function definition that gets the value of duty cycle from the duty cycle array.
 *@param None
 *@return None
 * */
void Get_Duty_Cycle(void);


/**
 * @brief ADC Calibration phase
 * @param none
 * @return None
 * */
void Motor_SM_Calibration(void);


/**
 * @brief Motor Ready State - Checks the Ready state of Gate Driver(both Ready-1 and Ready-2)
 * @param none
 * @return none
 * */
void Motor_SM_Ready(void);



//Closes the @defgroup block. Always kept last.

/**@}*/

