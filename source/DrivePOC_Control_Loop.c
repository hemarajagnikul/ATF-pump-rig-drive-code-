/**
 ******************************************************************************
 * @file           : DrivePOC_Control_Loop.c
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

/*******************************************************************************
 * Header inclusions
 ******************************************************************************/


#include "Drive_Parameters.h"
#include "stdbool.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_cmp.h"
#include "fsl_gpio.h"
#include "DrivePOC_Controller_NXP.h"
#include "DrivePOC_CommHandler.h"
#include "DrivePOC_MemHandler.h"
#include "DrivePOC_Control_Loop.h"
#include "DrivePOC_FaultHandler.h"
/*******************************************************************************
 * Variables
 ******************************************************************************/

Main_SM_States main_sm_state;       /**< Main state machine - States*/
Motor_SM_States motor_sm_state;     /**< Motor state machine - Run sub-states */
Main_SM_ControlSig main_ctrl_sig;   /**< Main state machine - control signals for state transition */
Motor_SM_ControlSig motor_ctrl_sig; /**< Motor state machine - control signals for state transition */
mcs_acim_open_loop_str s_open_loop;       /**< V/F Control Algorithm parameters */
int motor_run=0;
int get_out=0;

#ifdef TESTBENCH_LOCAL
testbench_i tb1;
testbench_sf tbsf1;
bool start_testbench = false;
#endif /* TESTBENCH_LOCAL */

/*******************************************************************************
 * Function definitions
 ******************************************************************************/
/*
 * @brief Function definition corresponding to the Open loop Control of ACIM using V/F algorithm
 * @param  None
 * @return GMCLIB_3COOR_T_FLT dutyCycles
 *
 * */
GMCLIB_3COOR_T_FLT Open_Loop_Control(void)
{
	GMCLIB_3COOR_T_FLT tempduty;
    /** Function call for the Main state machine of Open Loop algorithm */
    tempduty=Open_Loop_MainStateMC();
    return tempduty;
}


/*******************************************************************************
 * Function definitions
 ******************************************************************************/
/*
 * @brief Open loop Control Initializations
 * @param parameters None
 * @return None
 * **/
void Open_Loop_Control_Init(void){
	s_open_loop.s_dutyabc_f16.f16A			 = FRAC16(0.5F);
	s_open_loop.s_dutyabc_f16.f16B			 = FRAC16(0.5F);
	s_open_loop.s_dutyabc_f16.f16C			 = FRAC16(0.5F);
	s_open_loop.s_dutyabc_flt.fltA			 = 0.5F;
	s_open_loop.s_dutyabc_flt.fltB			 = 0.5F;
	s_open_loop.s_dutyabc_flt.fltC			 = 0.5F;
	main_ctrl_sig.main_sm_ctrl_fault		 = false;
	main_ctrl_sig.main_sm_ctrl_init_done	 = true;
	main_ctrl_sig.main_sm_ctrl_run           = true;
	motor_ctrl_sig.spin                		 = true;

}


/*
 * @brief Function definition corresponding to the main state machine of V/F algorithm
 * @param  None
 * @return GMCLIB_3COOR_T_FLT dutyCycles
 *
 * • Fault—the system detected a fault condition and waits until it is cleared.
   • Init—initialization of variables.
   • Stop—the system is initialized and waiting for the Run command.
   • Run—the system is running; it can be stopped by the Stop command.
 *
 * */
GMCLIB_3COOR_T_FLT Open_Loop_MainStateMC(void)
{
	/** \image state_machines.png */
	GMCLIB_3COOR_T_FLT dutycycles;
    /**
       The following are transition functions between these state functions:

       • Init → Stop  —the initialization is done, the system is entering the Stop state.
       • Stop → Run   —the Run command is applied, the system is entering the Run state
         (if the Run command is acknowledged).
       • Run → Stop   —the Stop command is applied, the system is entering the Stop state
         (if the Stop command is acknowledged).
       • Fault → Stop —the fault flag is cleared, the system is entering the Stop state.
       • [Init, Stop, Run] → Fault—a fault condition occurred, the system is entering the Fault state.
     */
    //PRINTF("%d,%d",main_sm_state,motor_sm_state);

//    switch(main_sm_state)
//    {	/**The following image shows the software flow \image latex software_flow.jpg */
//        case MAIN_SM_INIT:
        	Open_Loop_Control_Init();
            /* Checking for faults / initialization done command */
//        	System_FaultDiag();
//
//            if (true == main_ctrl_sig.main_sm_ctrl_fault)
//            {
//                main_sm_state = MAIN_SM_FAULT;
//            }
//            else if (true == main_ctrl_sig.main_sm_ctrl_init_done)
//            {
//                main_sm_state = MAIN_SM_STOP;
//            }
//            break;
//        case MAIN_SM_FAULT:
//
//        	Get_Fault_Duty_Cycle();
//            /* Checking for faults / initialization done command */
//        	System_FaultDiag();
//            if (false == main_ctrl_sig.main_sm_ctrl_fault)
//            {
//                main_sm_state = MAIN_SM_STOP;
//            }
//            break;
//        case MAIN_SM_STOP:
//            /** Checking for faults / run command */
//        	System_FaultDiag();
//            if (true == main_ctrl_sig.main_sm_ctrl_fault)
//            {
//                main_sm_state = MAIN_SM_FAULT;
//            }
//            else if (true == main_ctrl_sig.main_sm_ctrl_run)
//            {
//                main_sm_state  = MAIN_SM_RUN;
//                motor_sm_state = MOTOR_SM_CALIB;
//            }
//            break;
//        case MAIN_SM_RUN:
        	Open_Loop_MotorStateMC();
//            System_FaultDiag();
//            if (true == main_ctrl_sig.main_sm_ctrl_fault)
//            {
//                main_sm_state = MAIN_SM_FAULT;
//            }
//            else if (true == main_ctrl_sig.main_sm_ctrl_stop)
//            {
//                main_sm_state = MAIN_SM_STOP;
//            }
//            break;
//        default:
//            main_sm_state = MAIN_SM_STOP;
//            break;
//    }
    dutycycles.fltA=s_open_loop.s_dutyabc_flt.fltA;
    dutycycles.fltB=s_open_loop.s_dutyabc_flt.fltB;
    dutycycles.fltC=s_open_loop.s_dutyabc_flt.fltC;
    //PRINTF("%f,%f,%f\n",s_foc_acimvar.s_svpwmabcduty.fltA,s_foc_acimvar.s_svpwmabcduty.fltB,s_foc_acimvar.s_svpwmabcduty.fltC);

    return dutycycles;
}


/*
 * @brief Function definition corresponding to the Fault diagnosis
 * @param  None
 * @return None
 *
 * */
void System_FaultDiag(void)
{
    /** \todo: Add other functional checks to detect faults.*/
	//PTC_FaultDiag();

}

/*
 * @brief Function definition corresponding to fault indicated by PTC 150 Thermocouple on the Motor
 * @param  None
 * @return None
 * */
void PTC_FaultDiag(){
		if(1==Measure_from_PTC_150())
		{
		    main_ctrl_sig.main_sm_ctrl_fault = true;
		    LED_RED_ON();
		    //PRINTF("\n The temperature detected is higher than 150 deg C");
		}
		else{
		    main_ctrl_sig.main_sm_ctrl_fault = false;
		    LED_RED_OFF();
		    //PRINTF("\n The temperature is normal");
		}
}



/*
 * @brief Function definition corresponding to the Motor state machine of V/F control algorithm
 * @param  None
 * @return None
 * @details The motor state machines are based on the main state machine structure.
 * The Run state sub-states are added on top of the main structure to control
 * the motors properly
 *
 * */
void Open_Loop_MotorStateMC(void)
{
    /* The following are transition functions between these state functions:
        • Calib → Ready
            — Calibration of LTC2986 ADC is done; entering the Ready state.
        • Ready → Startup
            — Gate Driver is checked for Ready 1 and Ready 2 signals if it is V-0.7 Inverter.
            \todo:-We are not going to use Inverter V-0.7 any more. So decision on how this state can be made useful needs to be made.
            — This state can be removed based on the advice from the team.
            — In the startup phase V/F Control Algorithm is executed.
        • Startup → Spin
            — The motor then enters the steady state to produce the AC sine frequency needed.
        • Spin → Deceleration
        	— For the motor to decelerate the pin PTC8 has to be made HIGH externally as an indication to stop the motor to the MCU.
      */
     // int motor_run=0;

    switch(motor_sm_state)
    {
       // case MOTOR_SM_CALIB:
            /*
             * The LTC 2986 Analog 2 Digital Converter is calibrated according to the sensors connected
             * */
//        	Motor_SM_Calibration();
//            if (motor_ctrl_sig.calib == true)
//            {
//                motor_sm_state = MOTOR_SM_READY;
//            }
//            else{
//                 main_sm_state=MAIN_SM_FAULT;
//            }
//            break;
      //  case MOTOR_SM_READY:
        	/**\todo: To check if this Ready state in Motor State Machine is required in future for any of the testings */

           // if (motor_ctrl_sig.ready == true) // write the condition here
//        	Get_Fault_Duty_Cycle();
//        	if (motor_run == 1)
//            {
//        		get_out=0;
//                 motor_sm_state = MOTOR_SM_START;
//            }
//            else{
//            	main_sm_state=MAIN_SM_FAULT;
//            }
       //     break;
        case MOTOR_SM_START:
            /* Executing Open Loop V/F Control Algorithm */
        	Get_V_F_Duty_Cycle();
        	if (true == Get_Start_Up_status()){
        		motor_sm_state = MOTOR_SM_SPIN;
        	}
        	if(true == Get_Stop_status()){
        		motor_sm_state = MOTOR_SM_DECELERATE;
        	}
        	break;
         case MOTOR_SM_SPIN:
            /* Executing Open Loop V/F Control Algorithm */
        	Get_Duty_Cycle();
        	if(true == Get_Stop_status()){
        	      motor_sm_state = MOTOR_SM_DECELERATE;
        	}
             break;
         case MOTOR_SM_DECELERATE:
             /* Executing Open Loop V/F Control Algorithm */
        	 Get_Deceleration_Duty_Cycle();
        	 if (get_out == 1)
			 {
				 /**< Change the state of state machine to Motor ready state*/
				  motor_sm_state = MOTOR_SM_READY;
				  get_out = 0;
			 }
        	 break;
//        	 motor_sm_state = MOTOR_SM_READY; // added this to avoid reset function
        default:Get_Fault_Duty_Cycle();
    	if (motor_run == 1)
        {
    		get_out=0;
             motor_sm_state = MOTOR_SM_START;
        }
            break;
    }
}



/*
 * @brief ADC Calibration phase
 * @param parameters none
 * @return None
 * */
void Motor_SM_Calibration(void) {
	uint8_t global_config;
    ltc_configure_channels(3,0xE80F3000);/*Rsense Resistor of 972 ohm*/
    ltc_configure_channels(5, SENSOR_TYPE__RTD_PT_1000|RTD_RSENSE_CHANNEL__3|RTD_NUM_WIRES__2_WIRE|RTD_EXCITATION_MODE__NO_ROTATION_SHARING|RTD_EXCITATION_CURRENT__100UA|RTD_STANDARD__EUROPEAN);/*RTD - PT 1000*/
    ltc_configure_channels(6, SENSOR_TYPE__TYPE_K_THERMOCOUPLE|TC_COLD_JUNCTION_CH__5|TC_SINGLE_ENDED|TC_OPEN_CKT_DETECT__YES|TC_OPEN_CKT_DETECT_CURRENT__500UA);/*Thermocouple K Type-Single Ended*/
	/**Configure Channels at which the sensors are connected in LTC Board*/
    global_config=ltc_transfer_byte(WRITE_TO_RAM,0X00F0,REJECTION__50_60_HZ|TEMP_UNIT__C);/**Set the Global Configuration Register*/
    motor_ctrl_sig.calib = true;
    motor_ctrl_sig.ready = true;
}




