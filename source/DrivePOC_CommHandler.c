/**
 ******************************************************************************
 * @file           : DrivePOC_CommHandler.c
 * @brief          : Communication Handler functions providing for all
 *                   interfacing between NXP to other sensors/systems as well PWM Generation
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
#include "peripherals.h"
#include "fsl_debug_console.h"
#include "fsl_ftm.h"
#include "math.h"
#include "gmclib_FP.h"
#include "mcdrv_frdmkv31f.h"
#include "DrivePOC_CommHandler.h"
#define buffer_size 512//buffer size for number of values to be stored and considered for rms & mean calc
//the buffer size doesn't affect the calculation time of RMS_Mean_Calc
int num_samples = buffer_size; //stores the value of number of samples considered for rms & mean calc.
float buff0[buffer_size],buff1[buffer_size],buff2[buffer_size],buff3[buffer_size],buff4[buffer_size],buff5[buffer_size]; //stores the values of the buffer of 6 channel which are used for rms & mean calc
float mean_val_ch[6]; //stores value of mean for channel 1 to 6
float sob_ch[6] = { 0 };       //stores sum of values for mean calc
float Ch_val[8] = { 0.0 }; //stores the converted value from data read to true value
int call = 0;
int index_avg = -1;
int flag = 0;
int rma=0;//running mean/average variable
float sob_ch0,sob_ch1,sob_ch2,sob_ch3,sob_ch4,sob_ch5;
int rtd_val_ch0,rtd_val_ch1,rtd_val_ch2,rtd_val_ch3,rtd_val_ch4,rtd_val_ch5;
void RMS_Mean_Calc(int num_samples, float mean_val[], float sob[], int channel);
//!@name Encoder Variables
//!@{
volatile float    g_angular_vel           = 0.0f;
volatile float    g_delta_time_ms         = 0.0f;
//volatile float	  g_rpm			= 0.0f;
volatile uint32_t g_prev_encoder_count    = 0U;
volatile uint32_t g_cur_encoder_count     = 0U;
volatile uint32_t g_delta_encoder_count   = 0U;
volatile uint8_t g_dir_when_overflow      = 0U;
volatile bool b_encoder_direction         = false;
//!@}
int convdata=0,convdata1=0,i=0;

//!@name PT-1000 Variables
//!@{
adc16_config_t adc16ConfigStruct;
adc16_channel_config_t adc16ChannelConfigStruct;
float g_v_across_pt_1000_0,g_v_across_pt_1000_1,g_v_across_pt_1000_2,g_v_across_pt_1000_3,g_v_across_pt_1000_4,g_v_across_pt_1000_5,g_resistance_pt_1000,g_drive_phase_currents[25];
//!@}
uint8_t *p_ads_rx;
int16_t g_Ia=0;
int16_t g_Ib=0;
int16_t g_Ic=0;
int16_t g_Idc=0;
int16_t g_Va=0;
int16_t g_Vb=0;
int16_t g_Vc=0;
int16_t g_Vdc=0;
int16_t rpm_dir=0;
//!@name ADS SPI Variables
//!@{
dspi_master_config_t ads_masterConfig;
//dspi_master_edma_handle_t ads_g_dspi_edma_m_handle;
//edma_handle_t ads_dspiEdmaMasterRxRegToRxDataHandle;
//edma_handle_t ads_dspiEdmaMasterTxDataToIntermediaryHandle;
//edma_handle_t ads_dspiEdmaMasterIntermediaryToTxRegHandle;
dspi_transfer_t ads_masterXfer;
uint8_t ads_masterRxData[ADS_TRANSFER_SIZE] = {0};
uint8_t ads_masterTxData[ADS_TRANSFER_SIZE] = {0};
//edma_config_t ads_userConfig;
uint32_t ads_masterRxChannel = 3U;
uint32_t ads_masterIntermediaryChannel = 4U;
uint32_t ads_masterTxChannel = 5U;
//!@}
dac_config_t dacConfigStruct;

/******************************************************************************
 * define motor 1 3-ph PWM control functions                                  *
 ******************************************************************************/
#define M1_MCDRV_PWM_CLK_INIT() (InitClock())



/*
 * @brief Function definition to read out of the FTM1 registers for Quadrature decoding
 * @param None
 * @return None
 *
 * */
volatile uint32_t ReadFromEncoder(void)
{
    /* Read direction */
//    if (FTM_GetQuadDecoderFlags(FTM1_PERIPHERAL) & kFTM_QuadDecoderCountingIncreaseFlag)
//    {
//        b_encoder_direction = true;
//    }
//    else
//    {
//        b_encoder_direction = false;
//    }

    /* Read Encoder counter value */
    g_cur_encoder_count = FTM_GetQuadDecoderCounterValue(FTM1_PERIPHERAL);
	/* \todo Calculating delta-encoder count
	 * Considering the direction also while calculating delta count
	 * so that we don't have a jump in the speed measurement when
	 * the timer overflows. To be tested with the HIL  */
   // **** Working code ****
	//if ((g_cur_encoder_count >= g_prev_encoder_count) && (b_encoder_direction))
    if ((g_cur_encoder_count >= g_prev_encoder_count))
	{
		g_delta_encoder_count = g_cur_encoder_count - g_prev_encoder_count;
		rpm_dir=0;
		//			if (g_delta_encoder_count <= 0) {
//						g_delta_encoder_count = -g_cur_encoder_count + g_prev_encoder_count;
//							}
//		g_prev_encoder_count 	= g_cur_encoder_count;
	}
//	else if ((g_cur_encoder_count < g_prev_encoder_count) && (!b_encoder_direction))
//    else if ((g_cur_encoder_count < g_prev_encoder_count))
//	{
//		rpm_dir=1;
//		//g_delta_encoder_count = (30000-(g_cur_encoder_count)) - g_prev_encoder_count;
//		//g_prev_encoder_count 	= (30000 - g_cur_encoder_count);
//		g_delta_encoder_count = g_prev_encoder_count - g_cur_encoder_count;
////				g_delta_encoder_count = (30000-(g_cur_encoder_count)) - g_prev_encoder_count;
////				g_prev_encoder_count 	= (30000 - g_cur_encoder_count);
//	}
//
	g_prev_encoder_count 	= g_cur_encoder_count;
    //   **** Working code ****
//	if ( (b_encoder_direction))
//	{
//		g_delta_encoder_count = g_cur_encoder_count - g_prev_encoder_count;
//
//		g_prev_encoder_count 	= g_cur_encoder_count;
//	}
//	else if ((!b_encoder_direction))
//	{
//		g_delta_encoder_count = (30000-(g_cur_encoder_count)) - g_prev_encoder_count;
//		g_prev_encoder_count 	= (30000 - g_cur_encoder_count);
////		if (g_delta_encoder_count <= 0) {
////					g_delta_encoder_count = g_cur_encoder_count - g_prev_encoder_count;
////						}
////		//g_delta_encoder_count = g_prev_encoder_count - g_cur_encoder_count;
//	}

//
//    /* Assigning delta-time between measurements - SLOW_LOOP_FREQ */
//    g_delta_time_ms          = SLOW_LOOP_TIME;

	/* Calculate Speed - Pulses/sec
	 *
	 *     Encoder pulse count = g_delta_encoder_count (pulses per 0.4ms)
	 *     Decoding frequency  = SLOW_LOOP_FREQ (every 0.4 ms)
	 *
	 *     Encoder pulse count
	 *    -------------------------- * 1000  = x Pulses/sec
	 *     Decoding-Frequency (ms)
	 *
	 *     */
//	g_angular_vel			= (((float) g_delta_encoder_count) * 1000) / g_delta_time_ms;

	/* Calculate Speed - Revolutions per minute
	 *
	 * 	   Encoder speed in Pulses / sec
	 * 	   Encoder resolution (Scanning drum) - 512 pulses per revolution
	 * 	   Since Quadrature decoding mode is considered,
	 * 	   Encoder resolution = (512 * 4) pulses per revolution
	 *
	 * 	   Encoder speed (Pulses / sec)
	 * 	  ---------------------------------- * 60 (second -> minute) = Rev / Min
	 *     Encoder resolution (Pulses / rev)
	 *
	 * */
//	g_rpm         = ((float) g_angular_vel * 60.0f) / (512 * 4);
//	g_rpm         = (g_delta_encoder_count * 36.621);
    /* Store the value in the required format in Memory handler */
//	if (g_delta_encoder_count > 30000) {
//		g_delta_encoder_count = (g_delta_encoder_count - 30000);
//	}
    return g_delta_encoder_count;
//    DrivePOC_MH_UpdateEncoderSpeed(g_rpm);

}



/*
 * @brief Function definition to print debug data on the Serial terminal
 * over UART @ 115200 bps baudrate.
 * @param None
 * @return None
 *
 * */
//void PrintDebugInfo(void)
//{
//	/* Placeholder for other debug data */
//
//
//
//	/* Print data processed from Quad encoder */
//    if (b_encoder_direction)
//    {
//        PRINTF("Encoder direction: +, g_cur_encoder_count: %ld\r\n", g_cur_encoder_count);
//    }
//    else
//    {
//        PRINTF("Encoder direction: -, g_cur_encoder_count: %ld\r\n", g_cur_encoder_count);
//    }
//
//    PRINTF("Pulses: %ld, Time: %0.2f ms, Speed: %0.3f RPM\r\n----------------\r\n",
//				g_delta_encoder_count, g_delta_time_ms, g_rpm);
//}

/*
 * @brief Function definition to initialize the Communication Handler
 * of the Drive POC controller for ADC and PWM
 * @param None
 * @return None
 *
 * */
void DrivePOC_Comm_Handler_Init(void)
{
	PWM_Init();
	ltc_spi_edma_init();
	Comparator_Init();
	InbuiltADC_Init();
	ads_spi_edma_init();
	DAC_init();
}

/*
 * @brief Function definition to initialize the Timer PWMs
 * @param None
 * @return None
 */
void PWM_Init(void){
		GMCLIB_3COOR_T_FLT initialduty;
		initialduty.fltA=0.2f;
		initialduty.fltB=0.2f;
		initialduty.fltC=0.2f;
		/* Clock setup for PWM peripheral */
		M1_MCDRV_PWM_CLK_INIT();

		/* 6-channel PWM peripheral init */
		M1_MCDRV_PWM_PERIPH_INIT();

		/* Assigning address of the duty cycle variable */
		g_sM1Pwm3ph.psUABC = &(s_open_loop.s_dutyabc_f16);

		/* Initializing 0.5 as duty cycle for PWM */
		DrivePOC_UpdateDutyCyc(initialduty);
}

/*
 * @brief Initializes the Comparator Peripherals for PTC-150
 * @param None
 * @return None
 * */
void Comparator_Init(void){
    cmp_config_t mCmpConfigStruct;
    CMP_GetDefaultConfig(&mCmpConfigStruct);
    CMP_Init(CMP_BASE, &mCmpConfigStruct);
    cmp_dac_config_t mCmpDacConfigStruct;
    mCmpDacConfigStruct.referenceVoltageSource = kCMP_VrefSourceVin2; /* VCC. */
    mCmpDacConfigStruct.DACValue               = CMP_THRESHOLD;
    CMP_SetDACConfig(CMP_BASE, &mCmpDacConfigStruct);
    CMP_SetInputChannels(CMP_BASE, CMP_USER_CHANNEL, CMP_DAC_CHANNEL);
}

/*
 * @brief Function definition to disable PWM
 *
 * @param None
 * @return None
 *
 * */
void DrivePOC_Comm_Handler_PWMDis(void)
{
	/* Disable PWM output */
	M1_MCDRV_PWM3PH_DIS(&g_sM1Pwm3ph);
}

/* @brief Update duty cycle for PWM and Enable PWM
 *
 * @param GMCLIB_3COOR_T_FLT dutycycle
 * @return None
 *
 * */
void DrivePOC_UpdateDutyCyc(GMCLIB_3COOR_T_FLT dutyCycle)
{

    /* Setting up PWM's duty cycle to passed value */
	s_open_loop.s_dutyabc_f16.f16A                 = FRAC16(dutyCycle.fltA);
	s_open_loop.s_dutyabc_f16.f16B                 = FRAC16(dutyCycle.fltB);
	s_open_loop.s_dutyabc_f16.f16C                 = FRAC16(dutyCycle.fltC);
    //PRINTF("%d,%d,%d\n",s_foc_acimvar.s_dutyabc.f16A,s_foc_acimvar.s_dutyabc.f16B,s_foc_acimvar.s_dutyabc.f16C);
    M1_MCDRV_PWM3PH_SET(&g_sM1Pwm3ph); /** PWM duty cycles calculation and update */
    M1_MCDRV_PWM3PH_EN(&g_sM1Pwm3ph);/** Enable PWM output */

}



//!@name LTC SPI Variables
//!@{
uint8_t ltc_masterRxData[LTC_TRANSFER_SIZE] = {0};
uint8_t ltc_masterTxData[LTC_TRANSFER_SIZE] = {0};
dspi_master_config_t ltc_masterConfig;
dspi_master_edma_handle_t ltc_g_dspi_edma_m_handle;
edma_handle_t ltc_dspiEdmaMasterRxRegToRxDataHandle;
edma_handle_t ltc_dspiEdmaMasterTxDataToIntermediaryHandle;
edma_handle_t ltc_dspiEdmaMasterIntermediaryToTxRegHandle;
volatile bool ltc_isTransferCompleted = false;
dspi_transfer_t ltc_masterXfer;
edma_config_t ltc_userConfig;
uint32_t ltc_masterRxChannel = 0U;
uint32_t ltc_masterIntermediaryChannel = 1U;
uint32_t ltc_masterTxChannel = 2U;
//!@}


/*
 * @brief Function definition to get the start address corresponding to the channel number across which sensor is connected
 *
 * @param base_address base address of the memory location
 * @param channel_number represents the nth channel where the sensor gets connected to that particular channel and (n-1)th channel where n>1
 * @return start address corresponding to that channel
 * */
uint16_t ltc_get_start_address(uint16_t base_address, uint8_t channel_number)
{
  return base_address + 4 * (channel_number-1);
}



/*
 * @brief Function definiton to transfer 4 bytes
 * @details This function is used only while the Channel Assignment Data(Memory address Data specific to the channel) is being sent
 * from the MCU to the LTC Board and also while receiving the measured temperature/voltage data from the LTC Board to the MCU
 * @param ram_read_or_write Read or Write signal is sent first from the MCU to the LTC Board
 * @param start_address Channel specific address
 * @see ltc_get_start_address()
 * @param input_data Represents the data which is sent as an input to the LTC Board from the MCU through MOSI Line
 * @see ltc_spi_transfer_block()
 * */
uint32_t ltc_transfer_four_bytes(uint8_t ram_read_or_write, uint16_t start_address, uint32_t input_data)
{
  uint8_t tx[7];
  uint8_t *rx;
  uint32_t output_data;
  tx[0] = ram_read_or_write;
  tx[1] = (uint8_t)0xFF&(start_address>>8);
  tx[2] = (uint8_t)0x00FF&(start_address);
  tx[3] = (uint8_t)(input_data >> 24);
  tx[4] = (uint8_t)(input_data >> 16);
  tx[5] = (uint8_t)(input_data >> 8);
  tx[6] = (uint8_t) input_data;

  rx=ltc_spi_transfer_block(7,tx,rx);

  output_data = (((uint32_t) (rx[3] << 24)) |
                ((uint32_t) rx[4] << 16) |
                ((uint32_t) rx[5] << 8)  |
                ((uint32_t) rx[6]));
  return output_data;
}



/*
 * @brief Function definition for SPI Transfer
 * @detail The communication is Half Duplex Mode
 * @param TRANSFER_SIZEE number of bytes getting transferred
 * @param ttxx array pointer to send TRANSFER_SIZEE of bytes from MCU to LTC Board
 * @param rrxx array point to receive TRANSFER_SIZEE of bytes from LTC Board to MCU
 * @note Had given the receive byte address also as an input because, without this the received value will
 * disappear once the code exits this function block and this will lead to dangling pointer issue
 *
 */
uint8_t *ltc_spi_transfer_block(uint8_t TRANSFER_SIZEE, uint8_t *ttxx, uint8_t *rrxx)
{
	for(int ii=0;ii<TRANSFER_SIZEE;ii++)
	{
		ltc_masterTxData[ii]=ttxx[ii];
		ltc_masterRxData[ii]=0U;
	}

    /* Start master transfer */
    ltc_masterXfer.txData = ltc_masterTxData;
    ltc_masterXfer.rxData = ltc_masterRxData;
    ltc_masterXfer.dataSize = TRANSFER_SIZEE;
    ltc_masterXfer.configFlags = kDSPI_MasterCtar0 | LTC_DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;
    ltc_g_dspi_edma_m_handle.state=0;
    if (kStatus_Success != DSPI_MasterTransferEDMA(LTC_DSPI_MASTER_BASEADDR, &ltc_g_dspi_edma_m_handle, &ltc_masterXfer))
    {
        PRINTF("There is error when start LTC DSPI_MasterTransferEDMA \r\n ");
    }
    delay(25);

    rrxx=ltc_masterRxData;
    return rrxx;
}



/*
 * @brief configures channel
 * @detail It is used in the Calibration phase of Motor state machine
 * @param channel_number Channel number at which the sensor is connected
 * @param channel_assignment_data Channel specific data
 * @return None
 */
void ltc_configure_channels(uint8_t channel_number,uint32_t channel_assignment_data)
{
	uint32_t ack;
	uint16_t start_address = ltc_get_start_address(CH_ADDRESS_BASE, channel_number);
	ack=ltc_transfer_four_bytes(WRITE_TO_RAM, start_address, channel_assignment_data);
}



/*
 * @brief Function definition used a callback to check if the EDMA Transfer is successful or not
 * @param base DSPI peripheral base address
 * @param handle DSPI handle pointer to dspi_master_edma_handle_t
 * @param status using the status flag verifies if the EDMA Transfer is successful or not
 * @param userData A callback function parameter
 * @return None
 * */
void LTC_DSPI_MasterUserCallback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData)
{
    if (status == kStatus_Success)
    {
        //PRINTF("This is LTC DSPI master edma transfer completed callback. \r\n\r\n");
    }
}



/*
 * @brief Function which creates a delay in milli second
 * @detail uses NOP - No Operation inside a for loop to create delays.
 * @note Can standardize this function using Counter to provide delays
 * @param delay_in_ms Delay value in milliseconds
 */
void delay(int delay_in_ms)
{
	int a=delay_in_ms*10000/144;
	for(int kk=0;kk<a;kk++)
	{
		__asm volatile("nop");
	}
}



/*
 * @brief Function definition for transfer of a single byte data
 * @param ram_read_or_write Read or Write Signal
 * @param start_address Address Specific to Channel
 * @param input_data The data that has to be sent via the MOSI line of the MCU to the LTC Board (i.e.) input to LTC Board
 * @note This function is used while transferring Global Configuration Parameters
 * */
uint8_t ltc_transfer_byte(uint8_t ram_read_or_write, uint16_t start_address, uint8_t input_data)
{
  uint8_t tx[4];
  uint8_t *rx;
  uint8_t output_data;
  tx[0] = ram_read_or_write;
  tx[1] = (uint8_t)(start_address >> 8);
  tx[2] = (uint8_t)start_address;
  tx[3] = input_data;
  rx=ltc_spi_transfer_block(4, tx, rx);
  output_data=rx[3];
  return output_data;
}


/*
 * @brief Function definition to measure the sensor output
 * @details We can measure the output in terms of Voltage, Resistance or Temperature and this function will have
 * print options to display the result in the console window and hence its output is of void kind
 * @param channel_number Channel number at which the sensor is connected
 * @attention If the sensor is connected between n and n-1 th channel the channel number to be chosen is n and not n-1
 * @param channel_output Specify the kind of output, if the output we need is Voltage or Temperature or Resistance
 * @return None
 * @attention We have to update this part of code specific to the state machine because the value need not be printed
 * and it has to be given as an input to the state machine to check if the Temperature values are well within the limits
 * */
float_t ltc_measure_channel(uint8_t channel_number, uint8_t channel_output)
{ /**\todo Update this function according to state machine*/
	uint32_t temperature;
	uint8_t fault_data;
  ltc_convert_channel( channel_number);
  temperature=ltc_get_result( channel_number, channel_output);
  fault_data=temperature>>24;
  ltc_print_fault_data(fault_data);
  return (float_t)(temperature & 0x00FFFFFF)/1024.0F;
}



/*
 * @brief Function definition which performs the Initiate conversion action
 * @detail A conversion is initiated by writing a measurement command into RAM memory location 0x000 and
 * this function block takes care of this
 * @param channel_number Channel number at which the sensor is connected
 * @attention If the sensor is connected between n and n-1 th channel the channel number to be chosen is n and not n-1
 * @return None
 */
void ltc_convert_channel(uint8_t channel_number)
{
  // Start conversion
  uint8_t x;
  x=ltc_transfer_byte(WRITE_TO_RAM, COMMAND_STATUS_REGISTER, CONVERSION_CONTROL_BYTE | channel_number);
  ltc_wait_for_process_to_finish();
}



/*
 * @brief Function definition to check if the value 0x40 is being returned by the LTC Board in MOSI line of the LTC Board(MISO Line of the MCU)
 * which in turn indicates that the transfer of Initiate Conversion Command is a success
 * @return None
 * @see ltc_convert_channel()
 */
void ltc_wait_for_process_to_finish()
{
  uint8_t process_finished = 0;
  uint8_t data;
  while (process_finished == 0)
  {
    data = ltc_transfer_byte( READ_FROM_RAM, COMMAND_STATUS_REGISTER, 0x00);
    process_finished  = data & 0x40;
  }
}




/*
 * @brief Function which receives the Temperature or the Voltage value measured in the MISO line of the MCU
 * @param channel_number Channel number at which the sensor is connected
 * @attention If the sensor is connected between n and n-1 th channel the channel number to be chosen is n and not n-1
 * @param channel_output Specify the kind of output, if the output we need is Voltage or Temperature or Resistance
 * */
uint32_t ltc_get_result( uint8_t channel_number, uint8_t channel_output)
{
  uint32_t raw_data;
  uint8_t fault_data;
  uint16_t start_address = ltc_get_start_address(CONVERSION_RESULT_MEMORY_BASE, channel_number);
  uint32_t raw_conversion_result;
  raw_data = ltc_transfer_four_bytes( READ_FROM_RAM, start_address, 0X00);
  return raw_data;
}


/*
 * @brief Function definition to print the output in the console window
 * @param raw_conversion_result 32 bit output obtained in the MISO line of the MCU
 * @note Of this 32 bit first 8 bit represent the kind of Fault - Refer to Page-num-36 in the LTC Board. This shows
 * the fault bits for the Temperature sensor if it is of RTD kind
 * @param channel_output Specify the kind of output, if the output we need is Voltage or Temperature or Resistance
 * */
void ltc_print_conversion_result(uint32_t raw_conversion_result, uint8_t channel_output)
{
  int32_t signed_data = raw_conversion_result;
  float scaled_result;
  if (signed_data & 0x800000)
    signed_data = signed_data | 0xFF000000;
  /* Translate and print result*/
  if (channel_output == TEMPERATURE)
  {
    scaled_result = (float)(signed_data) *1.0/ 1024.0;
    PRINTF(" %f",scaled_result);
  }
  else if (channel_output == VOLTAGE)
  {
    scaled_result =  (float)(signed_data) / 2097152.0;
    //PRINTF("  Direct ADC reading in V =  %f\n",scaled_result);
  }
  else if (channel_output == CODE)
  {
    //PRINTF("  Direct ADC code = %ld\n",signed_data);
  }
}



/*
 * @brief Function definition to calculate if the value needed is in terms of Voltage or Resistance instead of Temperature
 * @param channel_number Channel number at which the sensor is connected
 * @attention If the sensor is connected between n and n-1 th channel the channel number to be chosen is n and not n-1
 * @return None
 * */
void ltc_read_voltage_or_resistance_results(uint8_t channel_number)
{
  int32_t raw_data;
  float voltage_or_resistance_result;
  uint16_t start_address = ltc_get_start_address(VOUT_CH_BASE, channel_number);
  raw_data = ltc_transfer_four_bytes(READ_FROM_RAM, start_address, 0);
  voltage_or_resistance_result = (float)raw_data/1024000000.0;
  //PRINTF("  Voltage or resistance = %f",voltage_or_resistance_result);
}



/*
 * @brief Function to print the Fault byte in the console window indicating the kind of fault as per the Datasheet
 * @param fault_byte The first 8-bits obtained in the MISO Line of the MCU after the Initiate Conversion command indicates the Fault byte
 * @return None
 * */
void ltc_print_fault_data(uint8_t fault_byte)
{
  if (fault_byte & SENSOR_HARD_FAILURE)
    //PRINTF("  - SENSOR HARD FALURE");
  if (fault_byte & ADC_HARD_FAILURE)
//	  PRINTF("  - ADC_HARD_FAILURE");
  if (fault_byte & CJ_HARD_FAILURE)
//	  PRINTF("  - CJ_HARD_FAILURE");
  if (fault_byte & CJ_SOFT_FAILURE)
//	  PRINTF("  - CJ_SOFT_FAILURE");
  if (fault_byte & SENSOR_ABOVE)
//	  PRINTF("  - SENSOR_ABOVE");
  if (fault_byte & SENSOR_BELOW)
//	  PRINTF("  - SENSOR_BELOW");
  if (fault_byte & ADC_RANGE_ERROR)
//	  PRINTF("  - ADC_RANGE_ERROR");
  if (!(fault_byte & VALID))
//	  PRINTF("INVALID READING !!!!!!");
  if (fault_byte == 0b11111111)
	  PRINTF("CONFIGURATION ERROR !!!!!!");
	  PRINTF("\nFAULT DATA = %d\n",fault_byte);

}



/*
 * @brief Initializes the DSPI & EDMA Peripheral
 * @return None
 * */
void ltc_spi_edma_init(void)
{
    /** DMA Mux setting and EDMA init */

    /** DMA MUX init */
    DMAMUX_Init(LTC_DSPI_MASTER_DMA_MUX_BASEADDR);
    DMAMUX_SetSource(LTC_DSPI_MASTER_DMA_MUX_BASEADDR, ltc_masterRxChannel, LTC_DSPI_MASTER_DMA_RX_REQUEST_SOURCE);
    DMAMUX_EnableChannel(LTC_DSPI_MASTER_DMA_MUX_BASEADDR, ltc_masterRxChannel);

#if (defined LTC_DSPI_MASTER_DMA_TX_REQUEST_SOURCE)
    DMAMUX_SetSource(LTC_DSPI_MASTER_DMA_MUX_BASEADDR, ltc_masterTxChannel, LTC_DSPI_MASTER_DMA_TX_REQUEST_SOURCE);
    DMAMUX_EnableChannel(LTC_DSPI_MASTER_DMA_MUX_BASEADDR, ltc_masterTxChannel);
#endif

    /* EDMA init*/
    /*
     * ltc_userConfig.enableRoundRobinArbitration = false;
     * ltc_userConfig.enableHaltOnError = true;
     * ltc_userConfig.enableContinuousLinkMode = false;
     * ltc_userConfig.enableDebugMode = false;
     */
    EDMA_GetDefaultConfig(&ltc_userConfig);
    EDMA_Init(LTC_DSPI_MASTER_DMA_BASEADDR, &ltc_userConfig);
    uint32_t srcClock_Hz;

    /*Master config*/
    ltc_masterConfig.whichCtar = kDSPI_Ctar0;
    ltc_masterConfig.ctarConfig.baudRate = LTC_TRANSFER_BAUDRATE;
    ltc_masterConfig.ctarConfig.bitsPerFrame = 8U;
    ltc_masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
    ltc_masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
    ltc_masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
    ltc_masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 1000000000U / LTC_TRANSFER_BAUDRATE;
    ltc_masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 1000000000U / LTC_TRANSFER_BAUDRATE;
    ltc_masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000000000U / LTC_TRANSFER_BAUDRATE;
    ltc_masterConfig.whichPcs = LTC_DSPI_MASTER_PCS_FOR_INIT;
    ltc_masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;
    ltc_masterConfig.enableContinuousSCK = false;
    ltc_masterConfig.enableRxFifoOverWrite = false;
    ltc_masterConfig.enableModifiedTimingFormat = false;
    ltc_masterConfig.samplePoint = kDSPI_SckToSin0Clock;
    srcClock_Hz = LTC_DSPI_MASTER_CLK_FREQ;
    DSPI_MasterInit(LTC_DSPI_MASTER_BASEADDR, &ltc_masterConfig, srcClock_Hz);

    /** Set up dspi master */
    memset(&(ltc_dspiEdmaMasterRxRegToRxDataHandle), 0, sizeof(ltc_dspiEdmaMasterRxRegToRxDataHandle));
    memset(&(ltc_dspiEdmaMasterTxDataToIntermediaryHandle), 0, sizeof(ltc_dspiEdmaMasterTxDataToIntermediaryHandle));
    memset(&(ltc_dspiEdmaMasterIntermediaryToTxRegHandle), 0, sizeof(ltc_dspiEdmaMasterIntermediaryToTxRegHandle));

    EDMA_CreateHandle(&(ltc_dspiEdmaMasterRxRegToRxDataHandle), LTC_DSPI_MASTER_DMA_BASEADDR, ltc_masterRxChannel);
    EDMA_CreateHandle(&(ltc_dspiEdmaMasterTxDataToIntermediaryHandle), LTC_DSPI_MASTER_DMA_BASEADDR,
                      ltc_masterIntermediaryChannel);
    EDMA_CreateHandle(&(ltc_dspiEdmaMasterIntermediaryToTxRegHandle), LTC_DSPI_MASTER_DMA_BASEADDR, ltc_masterTxChannel);

    DSPI_MasterTransferCreateHandleEDMA(LTC_DSPI_MASTER_BASEADDR, &ltc_g_dspi_edma_m_handle, LTC_DSPI_MasterUserCallback,
                                        NULL, &ltc_dspiEdmaMasterRxRegToRxDataHandle,
                                        &ltc_dspiEdmaMasterTxDataToIntermediaryHandle,
                                        &ltc_dspiEdmaMasterIntermediaryToTxRegHandle);


}




/*
 * @brief Function definition to collect the Temperature value
 * @param None
 * @return None
 * */
float_t Collect_Data_from_PT_1000(void){
	float_t temperature;
//	ADC16_SetChannelConfig(PT_1000_ADC16_BASE, PT_1000_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
//	while (0U == (kADC16_ChannelConversionDoneFlag &
//	              ADC16_GetChannelStatusFlags(PT_1000_ADC16_BASE, PT_1000_ADC16_CHANNEL_GROUP)))
//	{
//	}
//	g_v_across_pt_1000=(float_t)ADC16_GetChannelConversionValue(PT_1000_ADC16_BASE, PT_1000_ADC16_CHANNEL_GROUP)*3.3F/65535.0F;
	//g_resistance_pt_1000=POTENTIAL_DIVIDER_R1*g_v_across_pt_1000/(POTENTIAL_DIVIDER_INPUT-g_v_across_pt_1000);
//	temperature=(-PT_1000_A*RESISTANCE_0_DEG+sqrt(PT_1000_A*PT_1000_A*RESISTANCE_0_DEG*RESISTANCE_0_DEG-4.0f*PT_1000_B*RESISTANCE_0_DEG*(RESISTANCE_0_DEG-g_resistance_pt_1000)))/(2*PT_1000_B*RESISTANCE_0_DEG);
	//return temperature;
//	adc16ChannelConfigStruct.channelNumber = 8;
//	ADC16_SetChannelConfig(PT_1000_ADC16_BASE,
//	PT_1000_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
//	while (ADC16_GetChannelStatusFlags(PT_1000_ADC16_BASE,
//	PT_1000_ADC16_CHANNEL_GROUP) != kADC16_ChannelConversionDoneFlag) {
//	}
//	g_v_across_pt_1000 = (ADC16_GetChannelConversionValue(
//	PT_1000_ADC16_BASE, PT_1000_ADC16_CHANNEL_GROUP) * 3.3) / 65536;
	adc16ChannelConfigStruct.channelNumber = 12;
		ADC16_SetChannelConfig(PT_1000_ADC16_BASE,
		PT_1000_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
		while (ADC16_GetChannelStatusFlags(PT_1000_ADC16_BASE,
		PT_1000_ADC16_CHANNEL_GROUP) != kADC16_ChannelConversionDoneFlag) {
		}
		g_v_across_pt_1000_0 = (ADC16_GetChannelConversionValue(
		PT_1000_ADC16_BASE, PT_1000_ADC16_CHANNEL_GROUP) * 3.3) / 65536;
		Ch_val[0]=g_v_across_pt_1000_0;

	adc16ChannelConfigStruct.channelNumber = 13;
		ADC16_SetChannelConfig(PT_1000_ADC16_BASE,
		PT_1000_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
		while (ADC16_GetChannelStatusFlags(PT_1000_ADC16_BASE,
		PT_1000_ADC16_CHANNEL_GROUP) != kADC16_ChannelConversionDoneFlag) {
		}
		g_v_across_pt_1000_1 = (ADC16_GetChannelConversionValue(
		PT_1000_ADC16_BASE, PT_1000_ADC16_CHANNEL_GROUP) * 3.3) / 65536;
		Ch_val[1]=g_v_across_pt_1000_1;

		adc16ChannelConfigStruct.channelNumber = 17;
			ADC16_SetChannelConfig(PT_1000_ADC16_BASE,
			PT_1000_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
			while (ADC16_GetChannelStatusFlags(PT_1000_ADC16_BASE,
			PT_1000_ADC16_CHANNEL_GROUP) != kADC16_ChannelConversionDoneFlag) {
			}
			g_v_across_pt_1000_2 = (ADC16_GetChannelConversionValue(
			PT_1000_ADC16_BASE, PT_1000_ADC16_CHANNEL_GROUP) * 3.3) / 65536;
			Ch_val[2]=g_v_across_pt_1000_2;

	adc16ChannelConfigStruct.channelNumber = 18;
	ADC16_SetChannelConfig(PT_1000_ADC16_BASE,
	PT_1000_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
	while (ADC16_GetChannelStatusFlags(PT_1000_ADC16_BASE,
	PT_1000_ADC16_CHANNEL_GROUP) != kADC16_ChannelConversionDoneFlag) {
	}
	g_v_across_pt_1000_3 = (ADC16_GetChannelConversionValue(
	PT_1000_ADC16_BASE, PT_1000_ADC16_CHANNEL_GROUP) * 3.3) / 65536;
	Ch_val[3]=g_v_across_pt_1000_3;
//	RMS_Mean_Calc(num_samples, mean_val_ch, sob_ch,
//				call);
//		call == 1 ? call = 0 : call++;
	adc16ChannelConfigStruct.channelNumber = 8;
			ADC16_SetChannelConfig(PT_1000_ADC16_BASE,
			PT_1000_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
			while (ADC16_GetChannelStatusFlags(PT_1000_ADC16_BASE,
			PT_1000_ADC16_CHANNEL_GROUP) != kADC16_ChannelConversionDoneFlag) {
			}
			g_v_across_pt_1000_4 = (ADC16_GetChannelConversionValue(
			PT_1000_ADC16_BASE, PT_1000_ADC16_CHANNEL_GROUP) * 3.3) / 65536;
			Ch_val[4]=g_v_across_pt_1000_4;

		adc16ChannelConfigStruct.channelNumber = 9;
			ADC16_SetChannelConfig(PT_1000_ADC16_BASE,
			PT_1000_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
			while (ADC16_GetChannelStatusFlags(PT_1000_ADC16_BASE,
			PT_1000_ADC16_CHANNEL_GROUP) != kADC16_ChannelConversionDoneFlag) {
			}
			g_v_across_pt_1000_5 = (ADC16_GetChannelConversionValue(
			PT_1000_ADC16_BASE, PT_1000_ADC16_CHANNEL_GROUP) * 3.3) / 65536;
			Ch_val[5]=g_v_across_pt_1000_5;



	index_avg == buffer_size - 1 ? (index_avg = 0, flag = 1) : index_avg++;
	num_samples = (flag == 1) ? buffer_size : index_avg;
	buff0[index_avg] = Ch_val[0];
	buff1[index_avg] = Ch_val[1];
	buff2[index_avg] = Ch_val[2];
	buff3[index_avg] = Ch_val[3];
	buff4[index_avg] = Ch_val[4];
	buff5[index_avg] = Ch_val[5];
	index_avg == buffer_size - 1 ?
			((sob_ch0 = sob_ch0
					+ buff0[index_avg] - buff0[0]),(sob_ch1 = sob_ch1
							+ buff1[index_avg] - buff1[0]),(sob_ch2 = sob_ch2
									+ buff2[index_avg] - buff2[0]),(sob_ch3 = sob_ch3
											+ buff3[index_avg] - buff3[0]),(sob_ch4 = sob_ch4
													+ buff4[index_avg] - buff4[0]),(sob_ch5 = sob_ch5
															+ buff5[index_avg] - buff5[0])) :
			((sob_ch0 =
					sob_ch0 + buff0[index_avg] - buff0[index_avg + 1]),(sob_ch1 =
							sob_ch1+ buff1[index_avg] - buff1[index_avg + 1]),(sob_ch2 =
									sob_ch2 + buff2[index_avg] - buff2[index_avg + 1]),(sob_ch3 =
											sob_ch3 + buff3[index_avg] - buff3[index_avg + 1]),(sob_ch4 =
													sob_ch4 + buff4[index_avg] - buff4[index_avg + 1]),(sob_ch5 =
															sob_ch5 + buff5[index_avg] - buff5[index_avg + 1]));
	 rtd_val_ch0 = (int)((sob_ch0*19859) / (num_samples));//channel-3
	 rtd_val_ch1 = (int)((sob_ch1*19859) / (num_samples));//channel-4
	 rtd_val_ch2 = (int)((sob_ch2*19859) / (num_samples));//channel-5
	 rtd_val_ch3 = (int)((sob_ch3*19859) / (num_samples));//channel-6
	 rtd_val_ch4 = (int)((sob_ch4*19859) / (num_samples));//channel-1
	 rtd_val_ch5 = (int)((sob_ch5*19859) / (num_samples));//channel-2
//	rma == 1 ? rma = 0 : rma++;
//	g_v_across_pt_1000_2=mean_val_ch[0];
//	g_v_across_pt_1000_1=mean_val_ch[1];
//	adc16ChannelConfigStruct.channelNumber = 9;
//	ADC16_SetChannelConfig(PT_1000_ADC16_BASE,
//	PT_1000_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
//	while (ADC16_GetChannelStatusFlags(PT_1000_ADC16_BASE,
//	PT_1000_ADC16_CHANNEL_GROUP) != kADC16_ChannelConversionDoneFlag) {
//	}
//	g_drive_phase_currents[0] = (ADC16_GetChannelConversionValue(
//	PT_1000_ADC16_BASE, PT_1000_ADC16_CHANNEL_GROUP));
//
//	adc16ChannelConfigStruct.channelNumber = 12;
//	ADC16_SetChannelConfig(PT_1000_ADC16_BASE,
//	PT_1000_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
//	while (ADC16_GetChannelStatusFlags(PT_1000_ADC16_BASE,
//	PT_1000_ADC16_CHANNEL_GROUP) != kADC16_ChannelConversionDoneFlag) {
//	}
//	g_drive_phase_currents[1] = (ADC16_GetChannelConversionValue(
//	PT_1000_ADC16_BASE, PT_1000_ADC16_CHANNEL_GROUP));


//					for (int incre = 12; incre <= 15; incre++) {
//								adc16ChannelConfigStruct.channelNumber = incre;
//								ADC16_SetChannelConfig(PT_1000_ADC16_BASE,
//										PT_1000_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
//								while (ADC16_GetChannelStatusFlags(PT_1000_ADC16_BASE,
//										PT_1000_ADC16_CHANNEL_GROUP)
//										!= kADC16_ChannelConversionDoneFlag) {
//								}
//								g_drive_phase_currents[incre] = ADC16_GetChannelConversionValue(
//										PT_1000_ADC16_BASE, PT_1000_ADC16_CHANNEL_GROUP);
//							}

}


/*
 * @brief Function definition to measure from PTC-150
 * @param None
 * @return Boolean value '1' denotes that the output is higher than the set threshold value and '0' denotes that the output is lower than the threshold
 * \todo Update the threshold according to VIN & R1
 * */
bool_t Measure_from_PTC_150(void){
	bool_t result;
	if (kCMP_OutputAssertEventFlag == (kCMP_OutputAssertEventFlag & CMP_GetStatusFlags(CMP_BASE)))
		{
			result=1;
		}
	else
		{
			result=0;
		}
	return result;
}


/*
 * @brief Function definition to initialize the Inbuilt ADC
 * @param None
 * @return None
 * */
void InbuiltADC_Init(void){
    ADC16_GetDefaultConfig(&adc16ConfigStruct);
  //  adc16ConfigStruct.resolution = kADC16_Resolution16Bit;
#ifdef BOARD_ADC_USE_ALT_VREF
    adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif
    ADC16_Init(PT_1000_ADC16_BASE, &adc16ConfigStruct);
    ADC16_EnableHardwareTrigger(PT_1000_ADC16_BASE, false); /* Make sure the software trigger is used. */
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    ADC16_DoAutoCalibration(PT_1000_ADC16_BASE);
#endif
    adc16ChannelConfigStruct.channelNumber                        = PT_1000_ADC16_USER_CHANNEL;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false; //make a note of this.
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

}

/*
 * @brief Initializes SPI0 instance for Communication with the ADS8588 board
 * @param None
 * @return None
 * */
/*
 * @brief Function definition to initialize the Inbuilt ADC
 * @param None
 * @return None
 * */

/*
 * @brief Initializes SPI0 instance for Communication with the ADS8588 board
 * @param None
 * @return None
 * */
void ads_spi_edma_init(void)
{
    /* DMA Mux setting and EDMA init */
//	GPIO_PinWrite(GPIOC, 6, 0);//convsta pulled LOW
//	GPIO_PinWrite(GPIOC,9,1);//reset pulled HIGH
//	for(int ii=0;ii<100;ii++){
//		__asm volatile("nop");
//	}
//	GPIO_PinWrite(GPIOC,9,0);//reset pulled LOW
//	for(int kk=0;kk<5;kk++){
//		__asm volatile("nop");
//	}
//	GPIO_PinWrite(GPIOC, 6, 1);//convsta pulled HIGH
//	for(int kk=0;kk<50;kk++){
//		__asm volatile("nop");
//	}
    /* DMA MUX init */
//    DMAMUX_Init(ADS_DSPI_MASTER_DMA_MUX_BASEADDR);
//    DMAMUX_SetSource(ADS_DSPI_MASTER_DMA_MUX_BASEADDR, ads_masterRxChannel, ADS_DSPI_MASTER_DMA_RX_REQUEST_SOURCE);
//    DMAMUX_EnableChannel(ADS_DSPI_MASTER_DMA_MUX_BASEADDR, ads_masterRxChannel);
//
//#if (defined ADS_DSPI_MASTER_DMA_TX_REQUEST_SOURCE)
//    DMAMUX_SetSource(ADS_DSPI_MASTER_DMA_MUX_BASEADDR, ads_masterTxChannel, ADS_DSPI_MASTER_DMA_TX_REQUEST_SOURCE);
//    DMAMUX_EnableChannel(ADS_DSPI_MASTER_DMA_MUX_BASEADDR, ads_masterTxChannel);
//#endif

    /* EDMA init*/
    /*
     * ads_userConfig.enableRoundRobinArbitration = false;
     * ads_userConfig.enableHaltOnError = true;
     * ads_userConfig.enableContinuousLinkMode = false;
     * ads_userConfig.enableDebugMode = false;
     */
//    EDMA_GetDefaultConfig(&ads_userConfig);
//    EDMA_Init(ADS_DSPI_MASTER_DMA_BASEADDR, &ads_userConfig);
    uint32_t srcClock_Hz;

    /*Master config*/
    ads_masterConfig.whichCtar = kDSPI_Ctar0;
    ads_masterConfig.ctarConfig.baudRate = ADS_TRANSFER_BAUDRATE;
    ads_masterConfig.ctarConfig.bitsPerFrame = 7U;
    ads_masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
    ads_masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
    ads_masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
    ads_masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 1000000000U / ADS_TRANSFER_BAUDRATE;
    ads_masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 1000000000U / ADS_TRANSFER_BAUDRATE;
    ads_masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000000000U / ADS_TRANSFER_BAUDRATE;
    ads_masterConfig.whichPcs = ADS_DSPI_MASTER_PCS_FOR_INIT;
    ads_masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;
    ads_masterConfig.enableContinuousSCK = false;
    ads_masterConfig.enableRxFifoOverWrite = false;
    ads_masterConfig.enableModifiedTimingFormat = false;
    ads_masterConfig.samplePoint = kDSPI_SckToSin0Clock;
    srcClock_Hz = ADS_MASTER_CLK_FREQ;
    DSPI_MasterInit(ADS_DSPI_MASTER_BASEADDR, &ads_masterConfig, srcClock_Hz);

    /* Set up dspi master */
//    memset(&(ads_dspiEdmaMasterRxRegToRxDataHandle), 0, sizeof(ads_dspiEdmaMasterRxRegToRxDataHandle));
//    memset(&(ads_dspiEdmaMasterTxDataToIntermediaryHandle), 0, sizeof(ads_dspiEdmaMasterTxDataToIntermediaryHandle));
//    memset(&(ads_dspiEdmaMasterIntermediaryToTxRegHandle), 0, sizeof(ads_dspiEdmaMasterIntermediaryToTxRegHandle));
//
//    EDMA_CreateHandle(&(ads_dspiEdmaMasterRxRegToRxDataHandle), ADS_DSPI_MASTER_DMA_BASEADDR, ads_masterRxChannel);
//    EDMA_CreateHandle(&(ads_dspiEdmaMasterTxDataToIntermediaryHandle), ADS_DSPI_MASTER_DMA_BASEADDR,
//                      ads_masterIntermediaryChannel);
//    EDMA_CreateHandle(&(ads_dspiEdmaMasterIntermediaryToTxRegHandle), ADS_DSPI_MASTER_DMA_BASEADDR, ads_masterTxChannel);
//
//    DSPI_MasterTransferCreateHandleEDMA(ADS_DSPI_MASTER_BASEADDR, &ads_g_dspi_edma_m_handle, ADS_DSPI_MasterUserCallback,
//                                        NULL, &ads_dspiEdmaMasterRxRegToRxDataHandle,
//                                        &ads_dspiEdmaMasterTxDataToIntermediaryHandle,
//                                        &ads_dspiEdmaMasterIntermediaryToTxRegHandle);
}




/*
 * @brief Used for receiving the current and voltage value from the ADS8588 board using SPI Protocol
 * @param starting address value for the array which will store the current voltage values
 * @return starting address
 * @attention The syntax is with a pointer. This was made in this format keeping in mind the problem of dangling pointer.
 *
 * */
uint8_t *ads_spi_transfer_block(uint8_t *rrxx)
{

//	status_t a;
//	GPIO_PinWrite(GPIOC, 6, 0);//convsta pulled LOW
///**\todo:- delay should be introduced using a timer.*/
//	GPIO_PinWrite(GPIOC, 6, 1);//convsta pulled HIGH
//	for(int kk=0;kk<40;kk++){
//	    __asm volatile("nop");
//	}
//    ads_masterXfer.txData = ads_masterTxData;
//    ads_masterXfer.rxData = ads_masterRxData;
//    ads_masterXfer.dataSize = ADS_TRANSFER_SIZE;
//    ads_masterXfer.configFlags = kDSPI_MasterCtar0 | ADS_DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;
//    if(GPIO_PinRead(GPIOB, 21)==0){//Busy signal is read
//    	//If zero start DSPI Transfer(both send and receive)
//    	a=DSPI_MasterTransferEDMA(ADS_DSPI_MASTER_BASEADDR, &ads_g_dspi_edma_m_handle, &ads_masterXfer);
//    	//PRINTF("%ld\n",a);
//    	for(int kk=0;kk<1;kk++){
//    	    __asm volatile("nop");
//    	}
//    	if (kStatus_Success != a)
//    	{
//    		PRINTF("There is error when start DSPI_MasterTransferEDMA \r\n ");
//    	}
//    	rrxx=ads_masterRxData;
//    }
//	 GPIO_PinWrite(GPIOB, 21, 1);//reset
//	for (i = 0U; i < 5; i++) {
//		__asm volatile("nop");
//	}
//	 GPIO_PinWrite(GPIOB, 21, 0);
	for (i = 0U; i < 5; i++) {
		__asm volatile("nop");
	}
	 GPIO_PinWrite(GPIOC, 17, 1);//convst
	 for (i = 0U; i < 5; i++) {
	    	 			__asm volatile("nop");
	    	 		}
  /* Start master transfer, receive data from slave */
	     ads_masterXfer.txData = NULL;
	     ads_masterXfer.rxData = ads_masterRxData;
	     ads_masterXfer.dataSize = ADS_TRANSFER_SIZE;
	     ads_masterXfer.configFlags = kDSPI_MasterCtar0 | ADS_DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;
      DSPI_MasterTransferBlocking(ADS_DSPI_MASTER_BASEADDR, &ads_masterXfer);
	for (i = 0U; i < 5; i++) {
		__asm volatile("nop");
	}
   GPIO_PinWrite(GPIOC, 17, 0);
   for (i = 0U; i < 5; i++) {
      	 			__asm volatile("nop");
      	 		}
 //   if(GPIO_PinRead(GPIOB, 21)==0){//Busy signal is read
//   for (i = 0U; i < 5; i++) {
//     	 			__asm volatile("nop");
//     	 		}
	p_ads_rx = ads_masterRxData;
//	g_Ia = *(p_ads_rx + 0) << 8 | *(p_ads_rx + 1);
//	g_Va = *(p_ads_rx + 0) << 7 | *(p_ads_rx+1);
	g_Vdc = *(p_ads_rx + 2) << 7 | *(p_ads_rx + 3);
		if (g_Vdc>8191){
			g_Vdc=( g_Vdc-16384 );
		}
	g_Ia = *(p_ads_rx + 4) << 7 | *(p_ads_rx + 5);
	if (g_Ia>8191){
		g_Ia=( g_Ia-16384 );
	}
	g_Ib = *(p_ads_rx + 6) << 7 | *(p_ads_rx + 7);
	if (g_Ib>8191){
		g_Ib=(g_Ib-16384 );
	}
	g_Ic = *(p_ads_rx + 8) << 7 | *(p_ads_rx + 9);
	if (g_Ic>8191){
		g_Ic=(g_Ic -16384 );
	}

//	g_Vb = *(p_ads_rx + 10) << 8 | *(p_ads_rx + 11);
//	g_Vc = *(p_ads_rx + 12) << 8 | *(p_ads_rx + 13);
//	g_Vdc = *(p_ads_rx + 14) << 8 | *(p_ads_rx+15);
  }
//    return 0;
//}


/*
 * @brief Function definition used a callback to check if the EDMA Transfer is successful or not for ADS Board
 *
 * @param base DSPI peripheral base address
 * @param handle DSPI handle pointer to dspi_master_edma_handle_t
 * @param status using the status flag verifies if the EDMA Transfer is successful or not
 * @param userData A callback function parameter
 * @return None
 * */
void ADS_DSPI_MasterUserCallback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData)
{
    if (status == kStatus_Success)
    {
        //PRINTF("This is ADS DSPI master edma transfer completed callback. \r\n\r\n");
    }
}


/*
 * @brief Function definition for DAC
 * */
void DAC_init(void){
	DAC_GetDefaultConfig(&dacConfigStruct);
	DAC_Init(DAC0, &dacConfigStruct);
	DAC_Enable(DAC0, true);
	DAC_SetBufferReadPointer(DAC0, 0U);
//	DAC_Init(DAC1, &dacConfigStruct);
//	DAC_Enable(DAC1, true);
//	DAC_SetBufferReadPointer(DAC1, 0U);
}



/*
 *@brief Function definition to view the change in duty cycle or the read V & I values in a DSO through the inbuilt DAC Of MCU
 *@param none
 *@return none
 * */
void DAC_Update(int dac1, int dac2)
{
	DAC_SetBufferValue(DAC0, 0U, (dac1));
	DAC_SetBufferValue(DAC1, 0U, (dac2));
//	DAC_SetBufferValue(DAC1, 0U,  (s_foc_acimvar.s_iabc.fltB/3.3F)*0xFFF);

}
void RMS_Mean_Calc(int num_samples, float mean_val[],
		float sob[], int channel) {

	mean_val[channel] = sob[channel] / num_samples; //Updating the Moving Mean value
	//Inverse square root computation using fast inverse square root algorithm
	//refer to the document for more information
//	float g = sos[channel] / num_samples;
//	float y = g; //Converting (g) to a single precision floating-point number (y)
//	long i = *((long*) &y); //Converting (y) to its integer representation (i) using single precision bit representation
//	i = 0x5f3759df - (i >> 1); //Applying the Fast Inverse Square Root (FSQRT)
//	float h = *((float*) &i); //Convert the resulting (i) back to a single precision floating-point number (h)
//	//Updating the Moving RMS value
//	rms_val[channel] = h * g;
}

