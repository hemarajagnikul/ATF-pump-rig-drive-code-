/**
 ******************************************************************************
 * @file           : DrivePOC_Controller_NXP.c
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



#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKV31F51212.h"
#include "fsl_debug_console.h"
#include "fsl_ftm.h"
#include "fsl_pit.h"
#include "fsl_common.h"
#include "gmclib_FP.h" //motor control library
#include "time.h"
#include "Drive_Parameters.h"
#include "DrivePOC_Controller_NXP.h"
#include "DrivePOC_CommHandler.h"
#include "DrivePOC_Control_Loop.h"
#include "DrivePOC_MemHandler.h"
#include "DrivePOC_FaultHandler.h"
#include "mcdrv_frdmkv31f.h"
//#include "fsl_uart.h"
//!@name Global Variables
volatile uint32_t g_loop_counter          = 0U;
int encoder_test_variable=0,dac_test_variable=0;
float encoder_test_counter=0, DC_cap_link_volts=0;
float	  g_rpm			= 0;
//float_t g_drive_phase_currents[25];
int8_t mod_scheme_select=0;
//volatile uint32_t motor_run =0U;
#define DEMO_UART            UART0
#define DEMO_UART_CLKSRC     kCLOCK_CoreSysClk
#define DEMO_UART_CLK_FREQ   CLOCK_GetFreq(kCLOCK_CoreSysClk)

bool_t g_temperature_check;
GMCLIB_3COOR_T_FLT s_pwm_duty_cycle;
bool decrement = false;
#ifdef TESTBENCH_LOCAL
bool reset_loop_counter                 = false;
#endif /* TESTBENCH_LOCAL */
command_packet_t cmd_pckt_t;
static uint8_t rx_size = 0;
uint8_t checksum = 0;
static uint8_t command_array[sizeof(cmd_pckt_t)];
bool validate_command(command_packet_t *cmd_pkt);

/*******************************************************************************
 * Function definitions
 ******************************************************************************/
float SWITCHING_FREQUENCY=40000;
float SWITCHING_TIME;
float M;
float LUT_FREQUENCY;
float FREQ_NEEDED= 67;
float RAMP_TIME=5;
float SWITCHINGTIME;
float DECELERATION_TIME=5;
float FREQ_MAP;
float MAX_NUM_OF_LUT_SIZE;
float START_UP_OK;
float STOP_OK;
float accel_del_freq;
float decel_del_freq;
float f_freq_map;
int transmitgui=0, checksum_gui=0, freq_uc=0, freq_gui=0, speed_uc=0,speed_gui=0,i_rms_uc, i_rms_gui;
int rtd_val_ch0_uc,rtd_val_ch0_gui,rtd_val_ch1_uc,rtd_val_ch1_gui,rtd_val_ch2_uc,rtd_val_ch2_gui,rtd_val_ch3_uc,rtd_val_ch3_gui,rtd_val_ch00_uc,rtd_val_ch00_gui,rtd_val_ch01_uc,rtd_val_ch01_gui;
float i_ph_alpha, i_ph_beta, i_ph_rms;
int encoder_test_variable_1=1;
/*
 * To change the PWM deadtime go to CM4F_RTCESL_4.5.1_MCUX ---> MCDRV ---> mcdrv_frdmkv31f.h
 *
 *
 * @brief Function definition to configure the Periodic Interrupt Timer (PIT)
 * @details The function calls the PIT Handler which runs every 100us or 50us or 33us or 25us or 20 us based on the Switching Frequency
 * For 10kHz - 100 us
 * For 20kHz -  50 us
 * For 30kHz -  33 us
 * For 40kHz -  25 us
 * For 50kHz -  20 us
 * @param  None
 * @return None
 *
 * */
void PIT_Configuration(void)
{
    pit_config_t pitConfig;
    PIT_GetDefaultConfig(&pitConfig);
    PIT_Init(PIT, &pitConfig);/* Init pit module */
    /* Set timer period for channel 0 */
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(SWITCHING_TIME, PIT_SOURCE_CLOCK)); /* Sample rate 100 khz ~ 10 us */
    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);/** Enable timer interrupts for channel 0 */
    EnableIRQ(PIT_IRQ_ID);/* Enable at the NVIC */
}

/*
 * @brief   Application entry point.
 */
int main(void) {
    FREQ_MAP =(BIT_MAX/FREQ_NEEDED);
    SWITCHING_TIME =(1000000/SWITCHING_FREQUENCY);
    M = (FREQ_NEEDED)/MAX_FREQ;
    LUT_FREQUENCY = ((float)(SWITCHING_FREQUENCY/(3*68)));
    SWITCHINGTIME = 1/SWITCHING_FREQUENCY;
     MAX_NUM_OF_LUT_SIZE = SWITCHING_FREQUENCY/LUT_FREQUENCY;
     START_UP_OK = (RAMP_TIME*SWITCHING_FREQUENCY);
     STOP_OK	= (DECELERATION_TIME*SWITCHING_FREQUENCY);
     accel_del_freq=(FREQ_NEEDED-MIN_FREQ)*SWITCHINGTIME/(RAMP_TIME);
     decel_del_freq=(FREQ_NEEDED-MIN_FREQ)*SWITCHINGTIME/(DECELERATION_TIME);
     f_freq_map=((float_t)BIT_MAX/(float_t)FREQ_NEEDED);

    BOARD_InitBootPins();
    BOARD_InitBootClocks();/** Init board hardware. */
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    BOARD_InitDebugConsole(); /** Init FSL debug console. */
#endif
    BOARD_InitBootPeripherals(); /** Initialize peripherals used */
    DrivePOC_Comm_Handler_Init();/** Additional initializations (ADC in the future) */
    PIT_Configuration();/** PIT configuration */
    uart_config_t config;
    PIT_StartTimer(PIT, kPIT_Chnl_0);
//    ltc_configure_channels(3,0xE80F3000);/*Rsense Resistor of 972 ohm*/
    ltc_configure_channels(5, SENSOR_TYPE__TYPE_K_THERMOCOUPLE|TC_COLD_JUNCTION_CH__NONE|TC_SINGLE_ENDED|TC_OPEN_CKT_DETECT__YES|TC_OPEN_CKT_DETECT_CURRENT__500UA);/*Thermocouple K Type-Single Ended*/
	//PRINTF("%lx\n",SENSOR_TYPE__TYPE_K_THERMOCOUPLE|TC_COLD_JUNCTION_CH__NONE|TC_SINGLE_ENDED|TC_OPEN_CKT_DETECT__YES|TC_OPEN_CKT_DETECT_CURRENT__500UA);
    UART_GetDefaultConfig(&config);
      config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
      config.enableTx     = true;
      config.enableRx     = true;
      UART_Init(DEMO_UART, &config, DEMO_UART_CLK_FREQ);
    GPIO_PinWrite(GPIOB, 11, 1);
    UART_WriteBlocking(DEMO_UART, "enter 1 to motor start:\r\n enter 0 to motor stop:\r\n",51);

    while (1)
    {
    	// GPIO_PinWrite(GPIOC, 8, 1);

    	//asm __volatile("NOP");
    }
//    GPIO_PinWrite(GPIOB, 11, 0);
   // GPIO_PinWrite(GPIOC, 9, 0);
}



/* Interrupt handlers */
void PIT_IRQ_HANDLER(void)
{
	 GPIO_PinWrite(GPIOC, 8, 0);
	float_t temperature;
	/* Clear pit interrupt flag */
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);

	// E-stop command
		if(GPIO_PinRead(GPIOC, 9)==0){
			motor_run=0;
		}


	s_pwm_duty_cycle = Open_Loop_Control();
	DrivePOC_UpdateDutyCyc(s_pwm_duty_cycle);
//			GPIO_PinWrite(GPIOC, 6, 0);//convsta pulled LOW
//		/**\todo:- delay should be introduced using a timer.*/
//			GPIO_PinWrite(GPIOC, 6, 1);//convsta pulled HIGH

//			if (encoder_test_variable_1==1){
//				GPIO_PinWrite(GPIOC, 10, 0);
//				GPIO_PinWrite(GPIOC, 11, 1);
//			}
//			else if (encoder_test_variable_1==2){
//				GPIO_PinWrite(GPIOC, 10, 1);
		//		GPIO_PinWrite(GPIOC, 11, 1);
//				encoder_test_variable_1=0;
//						}
//			encoder_test_variable_1++;
//			else if (encoder_test_variable==3){
//				GPIO_PinWrite(GPIOC, 10, 0);
//				GPIO_PinWrite(GPIOC, 11, 1);
//						}
//			else if (encoder_test_variable==4){
//			if (encoder_test_variable==3){
	if (encoder_test_variable == (SWITCHING_FREQUENCY / 1000)) {
//				GPIO_PinWrite(GPIOC, 10, 0);
//				GPIO_PinWrite(GPIOC, 11, 0);
//		g_rpm = (float) ((float) ReadFromEncoder() * (float) 14.65);//quad encoder mode
		g_rpm = (float) ((float) ReadFromEncoder() * (float) 58.59);//direction and one pulse mode
//				encoder_test_counter=(float)((float) ReadFromEncoder()* (float) 0.01465); // 0.01465 = 60/(4*1024)
//				g_rpm         = (encoder_test_counter * SWITCHING_FREQUENCY)/4;
		encoder_test_variable = 0;
	}
//			encoder_test_counter=(ReadFromEncoder()*0.01465);
//			g_rpm         = (encoder_test_counter * SWITCHING_FREQUENCY);
			if (dac_test_variable==0){
//			DAC_Update((uint16_t)(s_open_loop.s_dutyabc_flt.fltA*0xFFF),(uint16_t)(s_open_loop.s_dutyabc_flt.fltC*0xFFF));
			DAC_Update((uint16_t)(g_rpm/2),(uint16_t)(i_ph_rms/10));
//			DAC_Update((uint16_t)((g_Ia+8192)/4),(uint16_t)((g_Ib+8192)/4));
//            DAC_Update((g_drive_phase_currents[0]/16),(g_drive_phase_currents[1]/16));
//			DAC_Update((uint16_t)(g_v_across_pt_1000_1*1400),(uint16_t)(g_v_across_pt_1000_2*1400));
			}
//			if (dac_test_variable==1){
//			DAC_Update((uint16_t)((i_ph_alpha+12288)/6),(uint16_t)((i_ph_beta+12288)/6));
//			}
//
			if (transmitgui == 0) {
			UART_WriteByte(DEMO_UART, 254); //0xFE
			checksum_gui = 254;
		}

		if (transmitgui == 1) {
			UART_WriteByte(DEMO_UART, motor_run); //0x18
			checksum_gui ^= motor_run;
		}
		if (transmitgui == 2) {
			freq_gui = ((freq*FREQ_NEEDED) / 4096);
			freq_uc = (round(freq_gui / 255));
			freq_gui =  (freq_gui % 255);
			checksum_gui ^= (freq_gui);
			UART_WriteByte(DEMO_UART, (freq_gui));
//		rtd_val_ch00_uc = (round((int) (rtd_val_ch4 / 255)));
//		rtd_val_ch00_gui = ((int) rtd_val_ch4 % 255);
//		checksum_gui ^= (rtd_val_ch00_gui);
//		UART_WriteByte(DEMO_UART, rtd_val_ch00_gui);
		}
		if (transmitgui == 3) {
//			checksum_gui ^= rtd_val_ch00_uc;
//			UART_WriteByte(DEMO_UART, rtd_val_ch00_uc);
			checksum_gui ^= freq_uc;
			UART_WriteByte(DEMO_UART, freq_uc);
		}
		if (transmitgui == 4) {
			speed_uc = (round((int)(g_rpm / 255)));
			speed_gui = ((int) g_rpm % 255);
			checksum_gui ^= (speed_gui);
			UART_WriteByte(DEMO_UART, speed_gui);
//		rtd_val_ch01_uc = (round((int) (rtd_val_ch5 / 255)));
//		rtd_val_ch01_gui = ((int) rtd_val_ch5 % 255);
//		checksum_gui ^= (rtd_val_ch01_gui);
//		UART_WriteByte(DEMO_UART, rtd_val_ch01_gui);
		}
		if (transmitgui == 5) {
			checksum_gui ^= speed_uc;
			UART_WriteByte(DEMO_UART, speed_uc);
//			checksum_gui ^= rtd_val_ch01_uc;
//			UART_WriteByte(DEMO_UART, rtd_val_ch01_uc);
		}
		if (transmitgui == 6) {
//			i_rms_uc =(round(  (int)(i_ph_rms/ 255)));
//			i_rms_gui = ((int) i_ph_rms % 255);
//			checksum_gui ^= (i_rms_gui);
			checksum_gui ^= (rpm_dir);
			UART_WriteByte(DEMO_UART, rpm_dir);
		}
		if (transmitgui == 7) {
			rtd_val_ch0_uc= (round((int)( rtd_val_ch0/ 255)));
			rtd_val_ch0_gui = ((int) rtd_val_ch0 % 255);
			checksum_gui ^= (rtd_val_ch0_gui);
			UART_WriteByte(DEMO_UART, rtd_val_ch0_gui);
		}
		if (transmitgui == 8) {
			checksum_gui ^= rtd_val_ch0_uc;
			UART_WriteByte(DEMO_UART, rtd_val_ch0_uc);
		}
		if (transmitgui == 9) {
			rtd_val_ch1_uc= (round((int)( rtd_val_ch1/ 255)));
			rtd_val_ch1_gui = ((int) rtd_val_ch1 % 255);
			checksum_gui ^= (rtd_val_ch1_gui);
			UART_WriteByte(DEMO_UART, rtd_val_ch1_gui);
		}
		if (transmitgui == 10) {
			checksum_gui ^= rtd_val_ch1_uc;
			UART_WriteByte(DEMO_UART, rtd_val_ch1_uc);
		}
		if (transmitgui == 11) {
			rtd_val_ch2_uc= (round((int)( rtd_val_ch2/ 255)));
			rtd_val_ch2_gui = ((int) rtd_val_ch2 % 255);
			checksum_gui ^= (rtd_val_ch2_gui);
			UART_WriteByte(DEMO_UART, rtd_val_ch2_gui);
		}
		if (transmitgui == 12) {
			checksum_gui ^= rtd_val_ch2_uc;
			UART_WriteByte(DEMO_UART, rtd_val_ch2_uc);
		}
		if (transmitgui == 13) {
			rtd_val_ch3_uc= (round((int)( rtd_val_ch3/ 255)));
			rtd_val_ch3_gui = ((int) rtd_val_ch3 % 255);
			checksum_gui ^= (rtd_val_ch3_gui);
			UART_WriteByte(DEMO_UART, rtd_val_ch3_gui);
		}
		if (transmitgui == 14) {
			checksum_gui ^= rtd_val_ch3_uc;
			UART_WriteByte(DEMO_UART, rtd_val_ch3_uc);
		}
		if (transmitgui == 15) {
			UART_WriteByte(DEMO_UART, checksum_gui);
			checksum_gui = 0;
		}
		transmitgui++;
		if (transmitgui >= 16) {
			transmitgui = 0;
		}
			encoder_test_variable++;
		//	UART_WriteByte(DEMO_UART, rx_size*40);
		//	UART_WriteByte(DEMO_UART, '0xFE');
		//	UART_WriteByte(DEMO_UART, '0');
					//UART_WriteBlocking(DEMO_UART, "motor stopped:\r\n",17);
			if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(DEMO_UART))
			{
						command_array[rx_size] = UART_ReadByte(DEMO_UART);
					//	UART_WriteBlocking(DEMO_UART, "UART_ReadByte(DEMO_UART):\r\n",10);
						rx_size += 1;
//
						if (rx_size == sizeof(cmd_pckt_t))
						{
							rx_size = 0;
							checksum=0;
							if (validate_command(command_array) == true)
							{
								handle_command(command_array);
							}
							else
							{
								/**< Invalid command Received */
							}
						}
					}

//			DrivePOC_MH_GetVIvalues(); // -> function to enable the SPI based data acquistion
//
//		DrivePOC_MH_UpdateVIvalues();
//			Collect_Data_from_PT_1000();

	SDK_ISR_EXIT_BARRIER;
	 GPIO_PinWrite(GPIOC, 8, 1);
}

bool validate_command(command_packet_t *cmd_pkt) {
//	if (cmd_pkt->SOF != 0x41) {
//		return false;
//	} else {
//		return true;
//	}
	//motor_run=1;
	uint8_t *ptr = (uint8_t*) cmd_pkt;
	for (int i = 0; i < sizeof(command_packet_t); i++) {
		checksum ^= ptr[i];
	}
	if (checksum == 0) {
	return true;
	} else {
		return false;
	}
}
//what if data is sending in wrong format? - how to identify this?
// or always fixed bit data has to be sent out by sender?
void handle_command(command_packet_t *cmd_pkt) {
	switch (cmd_pkt->pkt_id) {
	case C_Motor_command: {
		motor_run = ((cmd_pkt->data));
		if (motor_run>1){
			motor_run=0;
		}
		if (motor_run<0){
			motor_run=0;
		}
		break;
	}
	case Switching_freq: {
		// fsw =  ((cmd_pkt->data));
		if (motor_run==0){
		SWITCHING_FREQUENCY=((cmd_pkt->data));
		if (SWITCHING_FREQUENCY>20000){
			SWITCHING_FREQUENCY = 12000;
		}
		if (SWITCHING_FREQUENCY<8000){
					SWITCHING_FREQUENCY = 12000;
				}
		FREQ_MAP =(BIT_MAX/FREQ_NEEDED);
		SWITCHING_TIME =(1000000/SWITCHING_FREQUENCY);
		PIT_SetTimerPeriod(PIT, kPIT_Chnl_0,
				USEC_TO_COUNT(SWITCHING_TIME, PIT_SOURCE_CLOCK)); /* Sample rate 100 khz ~ 10 us */
		g_sClockSetup.ui16M1PwmFreq = SWITCHING_FREQUENCY;
		g_sClockSetup.ui16M1PwmModulo = g_sClockSetup.ui32BusClock
				/ g_sClockSetup.ui16M1PwmFreq;
		/* initial value of the FTM counter */
		FTM0->CNTIN = (uint32_t) (-g_sClockSetup.ui16M1PwmModulo / 2);

		/* modulo value */
		FTM0->MOD = (uint32_t) ((g_sClockSetup.ui16M1PwmModulo / 2) - 1);
//		FTM0->CONTROLS[0].CnV = (uint32_t) (-g_sClockSetup.ui16M1PwmModulo / 4);
//		FTM0->CONTROLS[1].CnV = (uint32_t) (g_sClockSetup.ui16M1PwmModulo / 4);
//		FTM0->CONTROLS[2].CnV = (uint32_t) (-g_sClockSetup.ui16M1PwmModulo / 4);
//		FTM0->CONTROLS[3].CnV = (uint32_t) (g_sClockSetup.ui16M1PwmModulo / 4);
//		FTM0->CONTROLS[4].CnV = (uint32_t) (-g_sClockSetup.ui16M1PwmModulo / 4);
//		FTM0->CONTROLS[5].CnV = (uint32_t) (g_sClockSetup.ui16M1PwmModulo / 4);
	    M = (FREQ_NEEDED)/MAX_FREQ;
		LUT_FREQUENCY = ((float)(SWITCHING_FREQUENCY/(3*68)));
		SWITCHINGTIME = 1/SWITCHING_FREQUENCY;
		MAX_NUM_OF_LUT_SIZE = SWITCHING_FREQUENCY/LUT_FREQUENCY;
		START_UP_OK = (RAMP_TIME*SWITCHING_FREQUENCY);
		 STOP_OK	= (DECELERATION_TIME*SWITCHING_FREQUENCY);
		 accel_del_freq=(FREQ_NEEDED-MIN_FREQ)*SWITCHINGTIME/(RAMP_TIME);
		 decel_del_freq=(FREQ_NEEDED-MIN_FREQ)*SWITCHINGTIME/(DECELERATION_TIME);
		 f_freq_map=((float_t)BIT_MAX/(float_t)FREQ_NEEDED);
		break;
		}
	}
	case Ramp_up_time: {
		if (motor_run==0){
			RAMP_TIME =((cmd_pkt->data));
			if (RAMP_TIME>60){
				RAMP_TIME = 60;
			}
			if (RAMP_TIME<5){
				RAMP_TIME = 5;
					}
	    FREQ_MAP =(BIT_MAX/FREQ_NEEDED);
	    SWITCHING_TIME =(1000000/SWITCHING_FREQUENCY);
	    M = (FREQ_NEEDED)/MAX_FREQ;
	    LUT_FREQUENCY = ((float)(SWITCHING_FREQUENCY/(3*68)));
	    SWITCHINGTIME = 1/SWITCHING_FREQUENCY;
	     MAX_NUM_OF_LUT_SIZE = SWITCHING_FREQUENCY/LUT_FREQUENCY;
	     START_UP_OK = (RAMP_TIME*SWITCHING_FREQUENCY);
	     STOP_OK	= (DECELERATION_TIME*SWITCHING_FREQUENCY);
	     accel_del_freq=(FREQ_NEEDED-MIN_FREQ)*SWITCHINGTIME/(RAMP_TIME);
	     decel_del_freq=(FREQ_NEEDED-MIN_FREQ)*SWITCHINGTIME/(DECELERATION_TIME);
	     f_freq_map=((float_t)BIT_MAX/(float_t)FREQ_NEEDED);
		}
		break;
	}
	case Ramp_down_time: {
		if (motor_run==0){
			DECELERATION_TIME =((cmd_pkt->data));
			if (DECELERATION_TIME>60){
				DECELERATION_TIME = 60;
			}
			if (DECELERATION_TIME<5){
				DECELERATION_TIME = 5;
					}
	    FREQ_MAP =(BIT_MAX/FREQ_NEEDED);
	    SWITCHING_TIME =(1000000/SWITCHING_FREQUENCY);
	    M = (FREQ_NEEDED)/MAX_FREQ;
	    LUT_FREQUENCY = ((float)(SWITCHING_FREQUENCY/(3*68)));
	    SWITCHINGTIME = 1/SWITCHING_FREQUENCY;
	     MAX_NUM_OF_LUT_SIZE = SWITCHING_FREQUENCY/LUT_FREQUENCY;
	     START_UP_OK = (RAMP_TIME*SWITCHING_FREQUENCY);
	     STOP_OK	= (DECELERATION_TIME*SWITCHING_FREQUENCY);
	     accel_del_freq=(FREQ_NEEDED-MIN_FREQ)*SWITCHINGTIME/(RAMP_TIME);
	     decel_del_freq=(FREQ_NEEDED-MIN_FREQ)*SWITCHINGTIME/(DECELERATION_TIME);
	     f_freq_map=((float_t)BIT_MAX/(float_t)FREQ_NEEDED);
		}
		break;
	}
	case Set_speed: {
		if (motor_run==0){
			FREQ_NEEDED =((cmd_pkt->data)/60);
			if (FREQ_NEEDED>MAX_FREQ){
				FREQ_NEEDED = MAX_FREQ-2;
			}
			if (FREQ_NEEDED<50){
				FREQ_NEEDED = 50;
					}
	    FREQ_MAP =(BIT_MAX/FREQ_NEEDED);
	    SWITCHING_TIME =(1000000/SWITCHING_FREQUENCY);
	    M = (FREQ_NEEDED)/MAX_FREQ;
	    LUT_FREQUENCY = ((float)(SWITCHING_FREQUENCY/(3*68)));
	    SWITCHINGTIME = 1/SWITCHING_FREQUENCY;
	     MAX_NUM_OF_LUT_SIZE = SWITCHING_FREQUENCY/LUT_FREQUENCY;
	     START_UP_OK = (RAMP_TIME*SWITCHING_FREQUENCY);
	     STOP_OK	= (DECELERATION_TIME*SWITCHING_FREQUENCY);
	     accel_del_freq=(FREQ_NEEDED-MIN_FREQ)*SWITCHINGTIME/(RAMP_TIME);
	     decel_del_freq=(FREQ_NEEDED-MIN_FREQ)*SWITCHINGTIME/(DECELERATION_TIME);
	     f_freq_map=((float_t)BIT_MAX/(float_t)FREQ_NEEDED);
		}
		break;
	}
//	case Modulation_scheme: {
//		if (motor_run==0){
//		mod_scheme_select = ((cmd_pkt->data));
//		}
//		break;
//	}
//	case Drive_Input_Voltage: {
//		if (motor_run==0){
//			DC_cap_link_volts = ((cmd_pkt->data));
//			}
//			break;
//	}
	}
}
