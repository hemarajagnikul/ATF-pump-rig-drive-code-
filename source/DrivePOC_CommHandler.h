/**
 ******************************************************************************
 * @file           : DrivePOC_CommHandler.h
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

/**
 * @defgroup group1 Communication Handler
 * @brief Communication Handler group
 * @details Group pertaining to all communication in/out of the NXP controller
 * @{
 */

/*******************************************************************************
 * Header inclusions
 ******************************************************************************/
#include "fsl_dspi_edma.h"
#include "fsl_dspi.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "fsl_cmp.h"
#include "DrivePOC_Common_Header.h"

extern mcs_acim_open_loop_str s_open_loop;
extern float g_drive_phase_currents[25],g_v_across_pt_1000_1,g_v_across_pt_1000_2;
extern int16_t g_Ia,g_Ib,g_Ic,rpm_dir;
extern int rtd_val_ch0,rtd_val_ch1,rtd_val_ch2,rtd_val_ch3,rtd_val_ch4,rtd_val_ch5;
//extern int16_t g_Ia=0;
//extern int16_t g_Ib=0;
//extern int16_t g_Ic=0;
//**********************************************************************************************************
// -- SENSOR TYPES --
//**********************************************************************************************************
//!@name RTD
//!@{
#define SENSOR_TYPE_LSB 27
#define SENSOR_TYPE__RTD_PT_10 (uint32_t) 0xA << SENSOR_TYPE_LSB
#define SENSOR_TYPE__RTD_PT_50 (uint32_t) 0xB << SENSOR_TYPE_LSB
#define SENSOR_TYPE__RTD_PT_100 (uint32_t) 0xC << SENSOR_TYPE_LSB
#define SENSOR_TYPE__RTD_PT_200 (uint32_t) 0xD << SENSOR_TYPE_LSB
#define SENSOR_TYPE__RTD_PT_500 (uint32_t) 0xE << SENSOR_TYPE_LSB
#define SENSOR_TYPE__RTD_PT_1000 (uint32_t) 0xF << SENSOR_TYPE_LSB
#define SENSOR_TYPE__RTD_PT_1000_375 (uint32_t) 0x10 << SENSOR_TYPE_LSB
#define SENSOR_TYPE__RTD_NI_120 (uint32_t) 0x11 << SENSOR_TYPE_LSB
#define SENSOR_TYPE__RTD_CUSTOM (uint32_t) 0x12 << SENSOR_TYPE_LSB
//!@}


//!@name  Sense Resistor
//!@{
#define SENSOR_TYPE__SENSE_RESISTOR (uint32_t) 0x1D << SENSOR_TYPE_LSB
#define SENSOR_TYPE__NONE (uint32_t) 0x0 << SENSOR_TYPE_LSB
#define SENSOR_TYPE__ACTIVE_ANALOG (uint32_t) 0x1F << SENSOR_TYPE_LSB
//!@}

//!@name Direct ADC
//!@{
#define SENSOR_TYPE__DIRECT_ADC (uint32_t) 0x1E << SENSOR_TYPE_LSB
//!@}

//!@name Thermistor
//!@{
#define SENSOR_TYPE__THERMISTOR_44004_2P252K_25C (uint32_t) 0x13 << SENSOR_TYPE_LSB
#define SENSOR_TYPE__THERMISTOR_44005_3K_25C (uint32_t) 0x14 << SENSOR_TYPE_LSB
#define SENSOR_TYPE__THERMISTOR_44007_5K_25C (uint32_t) 0x15 << SENSOR_TYPE_LSB
#define SENSOR_TYPE__THERMISTOR_44006_10K_25C (uint32_t) 0x16 << SENSOR_TYPE_LSB
#define SENSOR_TYPE__THERMISTOR_44008_30K_25C (uint32_t) 0x17 << SENSOR_TYPE_LSB
#define SENSOR_TYPE__THERMISTOR_YSI_400_2P252K_25C (uint32_t) 0x18 << SENSOR_TYPE_LSB
#define SENSOR_TYPE__THERMISTOR_1003K_1K_25C (uint32_t) 0x19 << SENSOR_TYPE_LSB
#define SENSOR_TYPE__THERMISTOR_CUSTOM_STEINHART_HART (uint32_t) 0x1A << SENSOR_TYPE_LSB
#define SENSOR_TYPE__THERMISTOR_CUSTOM_TABLE (uint32_t) 0x1B << SENSOR_TYPE_LSB
//!@}

//!@name Thermocouple
//!@{
#define SENSOR_TYPE__TYPE_J_THERMOCOUPLE (uint32_t) 0x1 << SENSOR_TYPE_LSB
#define SENSOR_TYPE__TYPE_K_THERMOCOUPLE (uint32_t) 0x2 << SENSOR_TYPE_LSB
#define SENSOR_TYPE__TYPE_E_THERMOCOUPLE (uint32_t) 0x3 << SENSOR_TYPE_LSB
#define SENSOR_TYPE__TYPE_N_THERMOCOUPLE (uint32_t) 0x4 << SENSOR_TYPE_LSB
#define SENSOR_TYPE__TYPE_R_THERMOCOUPLE (uint32_t) 0x5 << SENSOR_TYPE_LSB
#define SENSOR_TYPE__TYPE_S_THERMOCOUPLE (uint32_t) 0x6 << SENSOR_TYPE_LSB
#define SENSOR_TYPE__TYPE_T_THERMOCOUPLE (uint32_t) 0x7 << SENSOR_TYPE_LSB
#define SENSOR_TYPE__TYPE_B_THERMOCOUPLE (uint32_t) 0x8 << SENSOR_TYPE_LSB
#define SENSOR_TYPE__CUSTOM_THERMOCOUPLE (uint32_t) 0x9 << SENSOR_TYPE_LSB
//!@}

//!@name Off-Chip Diode
//!@{
#define SENSOR_TYPE__OFF_CHIP_DIODE (uint32_t) 0x1C << SENSOR_TYPE_LSB
//!@}

//!@name rtd - rsense channel
//!@{
#define RTD_RSENSE_CHANNEL_LSB 22
#define RTD_RSENSE_CHANNEL__NONE (uint32_t) 0x0 << RTD_RSENSE_CHANNEL_LSB
#define RTD_RSENSE_CHANNEL__1 (uint32_t) 0x1 << RTD_RSENSE_CHANNEL_LSB
#define RTD_RSENSE_CHANNEL__2 (uint32_t) 0x2 << RTD_RSENSE_CHANNEL_LSB
#define RTD_RSENSE_CHANNEL__3 (uint32_t) 0x3 << RTD_RSENSE_CHANNEL_LSB
#define RTD_RSENSE_CHANNEL__4 (uint32_t) 0x4 << RTD_RSENSE_CHANNEL_LSB
#define RTD_RSENSE_CHANNEL__5 (uint32_t) 0x5 << RTD_RSENSE_CHANNEL_LSB
#define RTD_RSENSE_CHANNEL__6 (uint32_t) 0x6 << RTD_RSENSE_CHANNEL_LSB
#define RTD_RSENSE_CHANNEL__7 (uint32_t) 0x7 << RTD_RSENSE_CHANNEL_LSB
#define RTD_RSENSE_CHANNEL__8 (uint32_t) 0x8 << RTD_RSENSE_CHANNEL_LSB
#define RTD_RSENSE_CHANNEL__9 (uint32_t) 0x9 << RTD_RSENSE_CHANNEL_LSB
#define RTD_RSENSE_CHANNEL__10 (uint32_t) 0xA << RTD_RSENSE_CHANNEL_LSB
//!@}

//!@name rtd - num wires
//!@{
#define RTD_NUM_WIRES_LSB 20
#define RTD_NUM_WIRES__2_WIRE (uint32_t) 0x0 << RTD_NUM_WIRES_LSB
#define RTD_NUM_WIRES__3_WIRE (uint32_t) 0x1 << RTD_NUM_WIRES_LSB
#define RTD_NUM_WIRES__4_WIRE (uint32_t) 0x2 << RTD_NUM_WIRES_LSB
#define RTD_NUM_WIRES__4_WIRE_KELVIN_RSENSE (uint32_t) 0x3 << RTD_NUM_WIRES_LSB
//!@}

//!@name rtd - excitation mode
//!@{
#define RTD_EXCITATION_MODE_LSB 18
#define RTD_EXCITATION_MODE__NO_ROTATION_NO_SHARING (uint32_t) 0x0 << RTD_EXCITATION_MODE_LSB
#define RTD_EXCITATION_MODE__NO_ROTATION_SHARING (uint32_t) 0x1 << RTD_EXCITATION_MODE_LSB
#define RTD_EXCITATION_MODE__ROTATION_SHARING (uint32_t) 0x2 << RTD_EXCITATION_MODE_LSB
//!@}

//!@name rtd - excitation current
//!@{
#define RTD_EXCITATION_CURRENT_LSB 14
#define RTD_EXCITATION_CURRENT__EXTERNAL (uint32_t) 0x0 << RTD_EXCITATION_CURRENT_LSB
#define RTD_EXCITATION_CURRENT__5UA (uint32_t) 0x1 << RTD_EXCITATION_CURRENT_LSB
#define RTD_EXCITATION_CURRENT__10UA (uint32_t) 0x2 << RTD_EXCITATION_CURRENT_LSB
#define RTD_EXCITATION_CURRENT__25UA (uint32_t) 0x3 << RTD_EXCITATION_CURRENT_LSB
#define RTD_EXCITATION_CURRENT__50UA (uint32_t) 0x4 << RTD_EXCITATION_CURRENT_LSB
#define RTD_EXCITATION_CURRENT__100UA (uint32_t) 0x5 << RTD_EXCITATION_CURRENT_LSB
#define RTD_EXCITATION_CURRENT__250UA (uint32_t) 0x6 << RTD_EXCITATION_CURRENT_LSB
#define RTD_EXCITATION_CURRENT__500UA (uint32_t) 0x7 << RTD_EXCITATION_CURRENT_LSB
#define RTD_EXCITATION_CURRENT__1MA (uint32_t) 0x8 << RTD_EXCITATION_CURRENT_LSB
//!@}

//!@name rtd - standard
//!@{
#define RTD_STANDARD_LSB 12
#define RTD_STANDARD__EUROPEAN (uint32_t) 0x0 << RTD_STANDARD_LSB
#define RTD_STANDARD__AMERICAN (uint32_t) 0x1 << RTD_STANDARD_LSB
#define RTD_STANDARD__JAPANESE (uint32_t) 0x2 << RTD_STANDARD_LSB
#define RTD_STANDARD__ITS_90 (uint32_t) 0x3 << RTD_STANDARD_LSB
//!@}

//!@name rtd-custom
//!@{
#define RTD_CUSTOM_ADDRESS_LSB 6/*rtd - custom address*/
#define RTD_CUSTOM_LENGTH_1_LSB 0/*rtd - custom length-1*/
#define RTD_CUSTOM_VALUES_LSB 31/*rtd - custom values*/
//!@}

//!@name sense resistor
//!@{
#define SENSE_RESISTOR_VALUE_LSB 0
//!@}

//!@name active analog - differential
//!@{
#define ACTIVE_ANALOG_DIFFERENTIAL_LSB 26
#define ACTIVE_ANALOG_DIFFERENTIAL (uint32_t) 0x0 << ACTIVE_ANALOG_DIFFERENTIAL_LSB
#define ACTIVE_ANALOG_SINGLE_ENDED (uint32_t) 0x1 << ACTIVE_ANALOG_DIFFERENTIAL_LSB
#define ACTIVE_ANALOG_CUSTOM_ADDRESS_LSB 6  /*active analog - custom address*/
#define ACTIVE_ANALOG_CUSTOM_LENGTH_1_LSB 0 /*active analog - custom length-1*/
#define ACTIVE_ANALOG_CUSTOM_VALUES_LSB 31  /*active analog - custom values*/
//!@}


//!@name Direct ADC - differential
//!@{
#define DIRECT_ADC_DIFFERENTIAL_LSB 26
#define DIRECT_ADC_DIFFERENTIAL (uint32_t) 0x0 << DIRECT_ADC_DIFFERENTIAL_LSB
#define DIRECT_ADC_SINGLE_ENDED (uint32_t) 0x1 << DIRECT_ADC_DIFFERENTIAL_LSB
//!@}

//!@name Direct ADC - custom
//!@{
#define DIRECT_ADC_CUSTOM_LSB 25
#define DIRECT_ADC_CUSTOM__NO (uint32_t) 0x0 << DIRECT_ADC_CUSTOM_LSB
#define DIRECT_ADC_CUSTOM__YES (uint32_t) 0x1 << DIRECT_ADC_CUSTOM_LSB
#define DIRECT_ADC_CUSTOM_ADDRESS_LSB 6 /*Direct ADC - custom address*/
#define DIRECT_ADC_CUSTOM_LENGTH_1_LSB 0 /*Direct ADC - custom length-1*/
#define DIRECT_ADC_CUSTOM_VALUES_LSB 31 /*Direct ADC - custom values*/
//!@}


//!@name thermistor - rsense channel
//!@{
#define THERMISTOR_RSENSE_CHANNEL_LSB 22
#define THERMISTOR_RSENSE_CHANNEL__NONE (uint32_t) 0x0 << THERMISTOR_RSENSE_CHANNEL_LSB
#define THERMISTOR_RSENSE_CHANNEL__1 (uint32_t) 0x1 << THERMISTOR_RSENSE_CHANNEL_LSB
#define THERMISTOR_RSENSE_CHANNEL__2 (uint32_t) 0x2 << THERMISTOR_RSENSE_CHANNEL_LSB
#define THERMISTOR_RSENSE_CHANNEL__3 (uint32_t) 0x3 << THERMISTOR_RSENSE_CHANNEL_LSB
#define THERMISTOR_RSENSE_CHANNEL__4 (uint32_t) 0x4 << THERMISTOR_RSENSE_CHANNEL_LSB
#define THERMISTOR_RSENSE_CHANNEL__5 (uint32_t) 0x5 << THERMISTOR_RSENSE_CHANNEL_LSB
#define THERMISTOR_RSENSE_CHANNEL__6 (uint32_t) 0x6 << THERMISTOR_RSENSE_CHANNEL_LSB
#define THERMISTOR_RSENSE_CHANNEL__7 (uint32_t) 0x7 << THERMISTOR_RSENSE_CHANNEL_LSB
#define THERMISTOR_RSENSE_CHANNEL__8 (uint32_t) 0x8 << THERMISTOR_RSENSE_CHANNEL_LSB
#define THERMISTOR_RSENSE_CHANNEL__9 (uint32_t) 0x9 << THERMISTOR_RSENSE_CHANNEL_LSB
#define THERMISTOR_RSENSE_CHANNEL__10 (uint32_t) 0xA << THERMISTOR_RSENSE_CHANNEL_LSB
//!@}

//!@name thermistor - differential
//!@{
#define THERMISTOR_DIFFERENTIAL_LSB 21
#define THERMISTOR_DIFFERENTIAL (uint32_t) 0x0 << THERMISTOR_DIFFERENTIAL_LSB
#define THERMISTOR_SINGLE_ENDED (uint32_t) 0x1 << THERMISTOR_DIFFERENTIAL_LSB
//!@}

//!@name thermistor - excitation mode
//!@{
#define THERMISTOR_EXCITATION_MODE_LSB 19
#define THERMISTOR_EXCITATION_MODE__NO_SHARING_NO_ROTATION (uint32_t) 0x0 << THERMISTOR_EXCITATION_MODE_LSB
#define THERMISTOR_EXCITATION_MODE__SHARING_ROTATION (uint32_t) 0x1 << THERMISTOR_EXCITATION_MODE_LSB
#define THERMISTOR_EXCITATION_MODE__SHARING_NO_ROTATION (uint32_t) 0x2 << THERMISTOR_EXCITATION_MODE_LSB
//!@}

//!@name thermistor - excitation current
//!@{
#define THERMISTOR_EXCITATION_CURRENT_LSB 15
#define THERMISTOR_EXCITATION_CURRENT__INVALID (uint32_t) 0x0 << THERMISTOR_EXCITATION_CURRENT_LSB
#define THERMISTOR_EXCITATION_CURRENT__250NA (uint32_t) 0x1 << THERMISTOR_EXCITATION_CURRENT_LSB
#define THERMISTOR_EXCITATION_CURRENT__500NA (uint32_t) 0x2 << THERMISTOR_EXCITATION_CURRENT_LSB
#define THERMISTOR_EXCITATION_CURRENT__1UA (uint32_t) 0x3 << THERMISTOR_EXCITATION_CURRENT_LSB
#define THERMISTOR_EXCITATION_CURRENT__5UA (uint32_t) 0x4 << THERMISTOR_EXCITATION_CURRENT_LSB
#define THERMISTOR_EXCITATION_CURRENT__10UA (uint32_t) 0x5 << THERMISTOR_EXCITATION_CURRENT_LSB
#define THERMISTOR_EXCITATION_CURRENT__25UA (uint32_t) 0x6 << THERMISTOR_EXCITATION_CURRENT_LSB
#define THERMISTOR_EXCITATION_CURRENT__50UA (uint32_t) 0x7 << THERMISTOR_EXCITATION_CURRENT_LSB
#define THERMISTOR_EXCITATION_CURRENT__100UA (uint32_t) 0x8 << THERMISTOR_EXCITATION_CURRENT_LSB
#define THERMISTOR_EXCITATION_CURRENT__250UA (uint32_t) 0x9 << THERMISTOR_EXCITATION_CURRENT_LSB
#define THERMISTOR_EXCITATION_CURRENT__500UA (uint32_t) 0xA << THERMISTOR_EXCITATION_CURRENT_LSB
#define THERMISTOR_EXCITATION_CURRENT__1MA (uint32_t) 0xB << THERMISTOR_EXCITATION_CURRENT_LSB
#define THERMISTOR_EXCITATION_CURRENT__AUTORANGE (uint32_t) 0xC << THERMISTOR_EXCITATION_CURRENT_LSB
#define THERMISTOR_EXCITATION_CURRENT__INVALID_ (uint32_t) 0xD << THERMISTOR_EXCITATION_CURRENT_LSB
#define THERMISTOR_EXCITATION_CURRENT__INVALID__ (uint32_t) 0xE << THERMISTOR_EXCITATION_CURRENT_LSB
#define THERMISTOR_EXCITATION_CURRENT__EXTERNAL (uint32_t) 0xF << THERMISTOR_EXCITATION_CURRENT_LSB
//!@}

//!@name thermistor-address
//!@{
#define THERMISTOR_CUSTOM_ADDRESS_LSB 6 /* thermistor - custom address*/
#define THERMISTOR_CUSTOM_LENGTH_1_LSB 0 /*thermistor - custom length-1*/
#define THERMISTOR_CUSTOM_VALUES_LSB 31  /*thermistor - custom values*/
//!@}





//!@name Thermocouple - cold junction ch
#define TC_COLD_JUNCTION_CH_LSB 22
#define TC_COLD_JUNCTION_CH__NONE (uint32_t) 0x0 << TC_COLD_JUNCTION_CH_LSB
#define TC_COLD_JUNCTION_CH__1 (uint32_t) 0x1 << TC_COLD_JUNCTION_CH_LSB
#define TC_COLD_JUNCTION_CH__2 (uint32_t) 0x2 << TC_COLD_JUNCTION_CH_LSB
#define TC_COLD_JUNCTION_CH__3 (uint32_t) 0x3 << TC_COLD_JUNCTION_CH_LSB
#define TC_COLD_JUNCTION_CH__4 (uint32_t) 0x4 << TC_COLD_JUNCTION_CH_LSB
#define TC_COLD_JUNCTION_CH__5 (uint32_t) 0x5 << TC_COLD_JUNCTION_CH_LSB
#define TC_COLD_JUNCTION_CH__6 (uint32_t) 0x6 << TC_COLD_JUNCTION_CH_LSB
#define TC_COLD_JUNCTION_CH__7 (uint32_t) 0x7 << TC_COLD_JUNCTION_CH_LSB
#define TC_COLD_JUNCTION_CH__8 (uint32_t) 0x8 << TC_COLD_JUNCTION_CH_LSB
#define TC_COLD_JUNCTION_CH__9 (uint32_t) 0x9 << TC_COLD_JUNCTION_CH_LSB
#define TC_COLD_JUNCTION_CH__10 (uint32_t) 0xA << TC_COLD_JUNCTION_CH_LSB
//!@}

//!@name thermocouple - differential
//!@{
#define TC_DIFFERENTIAL_LSB 21
#define TC_DIFFERENTIAL (uint32_t) 0x0 << TC_DIFFERENTIAL_LSB
#define TC_SINGLE_ENDED (uint32_t) 0x1 << TC_DIFFERENTIAL_LSB
//!@}

//!@name thermocouple - open ckt detect
//!@{
#define TC_OPEN_CKT_DETECT_LSB 20
#define TC_OPEN_CKT_DETECT__NO (uint32_t) 0x0 << TC_OPEN_CKT_DETECT_LSB
#define TC_OPEN_CKT_DETECT__YES (uint32_t) 0x1 << TC_OPEN_CKT_DETECT_LSB
//!@}

//!@name thermocouple - open ckt detect current
//!@{
#define TC_OPEN_CKT_DETECT_CURRENT_LSB 18
#define TC_OPEN_CKT_DETECT_CURRENT__10UA (uint32_t) 0x0 << TC_OPEN_CKT_DETECT_CURRENT_LSB
#define TC_OPEN_CKT_DETECT_CURRENT__100UA (uint32_t) 0x1 << TC_OPEN_CKT_DETECT_CURRENT_LSB
#define TC_OPEN_CKT_DETECT_CURRENT__500UA (uint32_t) 0x2 << TC_OPEN_CKT_DETECT_CURRENT_LSB
#define TC_OPEN_CKT_DETECT_CURRENT__1MA (uint32_t) 0x3 << TC_OPEN_CKT_DETECT_CURRENT_LSB
#define TC_CUSTOM_ADDRESS_LSB 6 /* tc - custom address*/
#define TC_CUSTOM_LENGTH_1_LSB 0/* tc - custom length-1*/
#define TC_CUSTOM_VALUES_LSB 31 /*tc - custom values*/
//!@}

//!@name off-chip diode - differential
//!@{
#define DIODE_DIFFERENTIAL_LSB 26
#define DIODE_DIFFERENTIAL (uint32_t) 0x0 << DIODE_DIFFERENTIAL_LSB
#define DIODE_SINGLE_ENDED (uint32_t) 0x1 << DIODE_DIFFERENTIAL_LSB
//!@}
//!@name diode - num readings
//!@{
#define DIODE_NUM_READINGS_LSB 25
#define DIODE_NUM_READINGS__2 (uint32_t) 0x0 << DIODE_NUM_READINGS_LSB
#define DIODE_NUM_READINGS__3 (uint32_t) 0x1 << DIODE_NUM_READINGS_LSB
//!@}

//!@name diode - averaging on
//!@{
#define DIODE_AVERAGING_ON_LSB 24
#define DIODE_AVERAGING_OFF (uint32_t) 0x0 << DIODE_AVERAGING_ON_LSB
#define DIODE_AVERAGING_ON (uint32_t) 0x1 << DIODE_AVERAGING_ON_LSB
//!@}

//!@name diode - current
//!@{
#define DIODE_CURRENT_LSB 22
#define DIODE_CURRENT__10UA_40UA_80UA (uint32_t) 0x0 << DIODE_CURRENT_LSB
#define DIODE_CURRENT__20UA_80UA_160UA (uint32_t) 0x1 << DIODE_CURRENT_LSB
#define DIODE_CURRENT__40UA_160UA_320UA (uint32_t) 0x2 << DIODE_CURRENT_LSB
#define DIODE_CURRENT__80UA_320UA_640UA (uint32_t) 0x3 << DIODE_CURRENT_LSB
#define DIODE_IDEALITY_FACTOR_LSB 0/**diode - ideality factor(eta)*/
//!@}

//!@name GLOBAL CONFIGURATION CONSTANTS
//!@{
#define REJECTION__50_60_HZ (uint8_t) 0x0
#define REJECTION__60_HZ    (uint8_t) 0x1
#define REJECTION__50_HZ    (uint8_t) 0x2
#define TEMP_UNIT__C        (uint8_t) 0x0
#define TEMP_UNIT__F        (uint8_t) 0x4
#define ENABLE_KELVIN_3_WIRE_RTD_MODE                (uint8_t) 0x10
#define ENABLE_KELVIN_2_WIRE_RTD_MODE                (uint8_t) 0x20
#define ENABLE_KELVIN_DIFFERENTIAL_THERMISTOR_MODE   (uint8_t) 0x40
#define DISABLE_MINUS_999                            (uint8_t) 0x80
//!@}

//!@name STATUS BYTE CONSTANTS
//!@{
#define SENSOR_HARD_FAILURE (uint8_t) 0x80
#define ADC_HARD_FAILURE    (uint8_t) 0x40
#define CJ_HARD_FAILURE     (uint8_t) 0x20
#define CJ_SOFT_FAILURE     (uint8_t) 0x10
#define SENSOR_ABOVE        (uint8_t) 0x8
#define SENSOR_BELOW        (uint8_t) 0x4
#define ADC_RANGE_ERROR     (uint8_t) 0x2
#define VALID               (uint8_t) 0x1
//!@}

//!@name ADDRESS BASE
//!@{
#define COMMAND_STATUS_REGISTER          (uint16_t) 0x0000
#define CH_ADDRESS_BASE                  (uint16_t) 0x0200
#define VOUT_CH_BASE                     (uint16_t) 0x0060
#define READ_CH_BASE                     (uint16_t) 0x0010
#define CONVERSION_RESULT_MEMORY_BASE    (uint16_t) 0x0010
//!@}

//!@name DATA to be sent in the MOSI line of MCU
//!@{
#define WRITE_TO_RAM            (uint8_t) 0x02
#define READ_FROM_RAM           (uint8_t) 0x03
#define CONVERSION_CONTROL_BYTE (uint8_t) 0x80
//!@}

//!@name OUTPUT TYPE
//!@{
#define VOLTAGE                 (uint8_t) 0x01
#define TEMPERATURE             (uint8_t) 0x02
#define CODE                    (uint8_t) 0x03
//!@}




//!@name LTC DSPI EDMA Peripheral
//!@{
#define LTC_DSPI_MASTER_BASEADDR SPI0
#define LTC_DSPI_MASTER_DMA_MUX_BASE DMAMUX_BASE
#define LTC_DSPI_MASTER_DMA_BASE DMA_BASE
#define LTC_DSPI_MASTER_DMA_RX_REQUEST_SOURCE kDmaRequestMux0SPI1
#define LTC_DSPI_MASTER_CLK_SRC DSPI1_CLK_SRC
#define LTC_DSPI_MASTER_CLK_FREQ CLOCK_GetFreq(DSPI0_CLK_SRC)
#define LTC_DSPI_MASTER_PCS_FOR_INIT kDSPI_Pcs0
#define LTC_DSPI_MASTER_PCS_FOR_TRANSFER kDSPI_MasterPcs0
#define LTC_DSPI_MASTER_DMA_MUX_BASEADDR ((DMAMUX_Type *)(LTC_DSPI_MASTER_DMA_MUX_BASE))
#define LTC_DSPI_MASTER_DMA_BASEADDR ((DMA_Type *)(LTC_DSPI_MASTER_DMA_BASE))
#define LTC_TRANSFER_SIZE 8U        /* Transfer dataSize */
#define LTC_TRANSFER_BAUDRATE 4000000U /* Transfer baudrate - 2M */
//!@}


//!@name ADS DSPI EDMA Peripheral
//!@{
#define NUM_CHANNEL 5
#define ADS_ADC_RANGE 10.0F
#define ADS_DSPI_MASTER_BASEADDR SPI1
#define ADS_DSPI_MASTER_DMA_MUX_BASE DMAMUX_BASE
#define ADS_DSPI_MASTER_DMA_BASE DMA_BASE
#define ADS_DSPI_MASTER_DMA_RX_REQUEST_SOURCE kDmaRequestMux0SPI1Rx
#define ADS_DSPI_MASTER_DMA_TX_REQUEST_SOURCE kDmaRequestMux0SPI1Tx
#define ADS_MASTER_CLK_SRC DSPI1_CLK_SRC
#define ADS_MASTER_CLK_FREQ DSPI1_CLK_SRC
#define ADS_DSPI_MASTER_PCS_FOR_INIT kDSPI_Pcs1
#define ADS_DSPI_MASTER_PCS_FOR_TRANSFER kDSPI_MasterPcs1
#define ADS_TRANSFER_SIZE 2U*NUM_CHANNEL        /* Transfer dataSize */
#define ADS_TRANSFER_BAUDRATE 40000000U /* Transfer baudrate - 20M */
#define ADS_DSPI_MASTER_DMA_MUX_BASEADDR ((DMAMUX_Type *)(ADS_DSPI_MASTER_DMA_MUX_BASE))
#define ADS_DSPI_MASTER_DMA_BASEADDR ((DMA_Type *)(ADS_DSPI_MASTER_DMA_BASE))

//!@name On-board Comparator for PTC-150
//!@{
#define CMP_BASE CMP0
#define CMP_USER_CHANNEL 1U
#define CMP_DAC_CHANNEL 7U
#define CMP_THRESHOLD 31U
//!@}

//!@name Inbuilt ADC & PT-1000 Sensor parameters
//!@{
#define PT_1000_ADC16_BASE          ADC0
#define PT_1000_ADC16_CHANNEL_GROUP 0U
#define PT_1000_ADC16_USER_CHANNEL  12U /* PTB0, ADC0_SE8 */
#define FSL_FEATURE_ADC16_MAX_RESOLUTION (16)
#define ALPHA 0.00385F
#define PT_1000_A	 3.908300E-3F
#define PT_1000_B	 -5.775E-7
#define PT_1000_C	 –4.183000E-12
#define POTENTIAL_DIVIDER_INPUT 3.0F
#define RESISTANCE_0_DEG 1000.0F
#define POTENTIAL_DIVIDER_R1 6800.0F
//!@}

/*******************************************************************************
 * Function prototypes
 ******************************************************************************/


/**
 * @brief Function definition to read out of the FTM1 registers for Quadrature decoding
 * @param  None
 * @return None
 *
 * */
volatile uint32_t ReadFromEncoder(void);

/**
 * @brief Function definition to print debug data on the Serial terminal
 * over UART @ 115200 bps baudrate.
 *
 * @param  None
 * @return None
 *
 * */
//void PrintDebugInfo(void);

/**
 * @brief Function definition to initialize the Communication Handler
 * of the Drive POC controller for ADC and PWM
 *
 * @param  None
 * @return None
 *
 * */
void DrivePOC_Comm_Handler_Init(void);

/**
 * @brief Initialize PWM Signals
 * @param  None
 * @return None
 */
void PWM_Init(void);

/**
 * @brief Initialize Comparator
 * @param  None
 * @return None
 */
void Comparator_Init(void);


/**
 * @brief Function definition to disable PWM
 *
 * @param  None
 * @return None
 *
 * */
void DrivePOC_Comm_Handler_PWMDis(void);

/**
 * @brief Update duty cycle for PWM
 *
 * @param  dutyCycle - Duty cycle value in GMCLIB_3COOR_T_FLT
 * @return None
 *
 * */
void DrivePOC_UpdateDutyCyc(GMCLIB_3COOR_T_FLT dutyCycle);

/**
 * @brief Function definition used a callback to check if the EDMA Transfer is successful or not for LTC Board
 *
 * @param base DSPI peripheral base address
 * @param handle DSPI handle pointer to dspi_master_edma_handle_t
 * @param status using the status flag verifies if the EDMA Transfer is successful or not
 * @param userData A callback function parameter
 * @return None
 *
 */
void LTC_DSPI_MasterUserCallback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData);

/**
 * @brief Function definition to get the start address corresponding to the channel number across which sensor is connected
 *
 * @param base_address base address of the memory location
 * @param channel_number represents the nth channel where the sensor gets connected to that particular channel and (n-1)th channel where n>1
 * @return start address corresponding to that channel
 * */
uint16_t ltc_get_start_address(uint16_t base_address, uint8_t channel_number);

/**
 * @brief Function definiton to transfer 4 bytes
 * @details This function is used only while the Channel Assignment Data(Memory address Data specific to the channel) is being sent
 * from the MCU to the LTC Board and also while receiving the measured temperature/voltage data from the LTC Board to the MCU
 * @param ram_read_or_write Read or Write signal is sent first from the MCU to the LTC Board
 * @param start_address Channel specific address
 * @see ltc_get_start_address()
 * @param input_data Represents the data which is sent as an input to the LTC Board from the MCU through MOSI Line
 * @see ltc_spi_transfer_block()
 * */
uint32_t ltc_transfer_four_bytes(uint8_t ram_read_or_write, uint16_t start_address, uint32_t input_data);

/**
 * @brief Function definition for SPI Transfer
 * @detail The communication is Half Duplex Mode
 * @param TRANSFER_SIZEE number of bytes getting transferred
 * @param ttxx array pointer to send TRANSFER_SIZEE of bytes from MCU to LTC Board
 * @param rrxx array point to receive TRANSFER_SIZEE of bytes from LTC Board to MCU
 * @note Had given the receive byte address also as an input because, without this the received value will
 * disappear once the code exits this function block and this will lead to dangling pointer issue
 *
 */
uint8_t *ltc_spi_transfer_block(uint8_t TRANSFER_SIZEE, uint8_t *ttxx, uint8_t *rrxx);


/*
 * @brief configures channel
 * @detail It is used in the Calibration phase of Motor state machine
 * @param channel_number Channel number at which the sensor is connected
 * @param channel_assignment_data Channel specific data
 * @return None
 */
void ltc_configure_channels(uint8_t channel_number,uint32_t channel_assignment_data);

/**
 * @brief Function which creates a delay in milli second
 * @detail uses NOP - No Operation inside a for loop to create delays.
 * @note Can standardize this function using Counter to provide delays
 * @param delay_in_ms Delay value in milliseconds
 */
void delay(int delay_in_ms);

/**
 * @brief Function definition for transfer of a single byte data
 * @param ram_read_or_write Read or Write Signal
 * @param start_address Address Specific to Channel
 * @param input_data The data that has to be sent via the MOSI line of the MCU to the LTC Board (i.e.) input to LTC Board
 * @note This function is used while transferring Global Configuration Parameters
 * */
uint8_t ltc_transfer_byte(uint8_t ram_read_or_write, uint16_t start_address, uint8_t input_data);

/**
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
float_t ltc_measure_channel(uint8_t channel_number, uint8_t channel_output);

/**
 * @brief Function definition which performs the Initiate conversion action
 * @details A conversion is initiated by writing a measurement command into RAM memory location 0x000 and
 * this function block takes care of this
 * @param channel_number Channel number at which the sensor is connected
 * @attention If the sensor is connected between n and n-1 th channel the channel number to be chosen is n and not n-1
 * @return None
 */
void ltc_convert_channel(uint8_t channel_number);

/**
 * @brief Function definition to check if the value 0x40 is being returned by the LTC Board in MOSI line of the LTC Board(MISO Line of the MCU)
 * which in turn indicates that the transfer of Initiate Conversion Command is a success
 * @return None
 * @see ltc_convert_channel()
 */
void ltc_wait_for_process_to_finish(void);

/**
 * @brief Function which receives the Temperature or the Voltage value measured in the MISO line of the MCU
 * \todo Update the function based on Interrupt pin of LTC2986
 * @param channel_number Channel number at which the sensor is connected
 * @attention If the sensor is connected between n and n-1 th channel the channel number to be chosen is n and not n-1
 * @param channel_output Specify the kind of output, if the output we need is Voltage or Temperature or Resistance
 * */
uint32_t ltc_get_result( uint8_t channel_number, uint8_t channel_output);

/**
 * @brief Function definition to print the output in the console window
 * @param raw_conversion_result 32 bit output obtained in the MISO line of the MCU
 * @note Of this 32 bit first 8 bit represent the kind of Fault - Refer to Page-num-36 in the LTC Board. This shows
 * the fault bits for the Temperature sensor if it is of RTD kind
 * @param channel_output Specify the kind of output, if the output we need is Voltage or Temperature or Resistance
 * */
void ltc_print_conversion_result(uint32_t raw_conversion_result, uint8_t channel_output);

/**
 * @brief Function definition to calculate if the value needed is in terms of Voltage or Resistance instead of Temperature
 * @param channel_number Channel number at which the sensor is connected
 * @attention If the sensor is connected between n and n-1 th channel the channel number to be chosen is n and not n-1
 * @return None
 * */
void ltc_read_voltage_or_resistance_results(uint8_t channel_number);

/**
 * @brief Function to print the Fault byte in the console window indicating the kind of fault as per the Datasheet
 * @param fault_byte The first 8-bits obtained in the MISO Line of the MCU after the Initiate Conversion command indicates the Fault byte
 * @return None
 * */
void ltc_print_fault_data(uint8_t fault_byte);

/**
 * @brief Initializes the DSPI & EDMA Peripheral
 * @return None
 * */
void ltc_spi_edma_init(void);

/**
 * @brief Initializes the Comparator Peripherals for PTC-150
 * @return None
 * */
void comparator_init(void);


/**
 * @brief Function definition to initialize the Inbuilt ADC
 * @param None
 * @return None
 * */
void InbuiltADC_Init(void);

/**
 * @brief Function definition to collect the Temperature value
 * @param None
 * @return None
 * */
float_t Collect_Data_from_PT_1000(void);

/**
 * @brief Function definition to measure from PTC-150
 * @param None
 * @return Boolean value '1' denotes that the output is higher than the set threshold value and '0' denotes that the output is lower than the threshold
 * \todo Update the threshold according to VIN & R1
 * */
bool_t Measure_from_PTC_150(void);


/**
 * @brief Initializes SPI0 instance for Communication with the ADS8588 board
 * @param None
 * @return None
 * */
void ads_spi_edma_init(void);

/**
 * @brief Used for receiving the current and voltage value from the ADS8588 board using SPI Protocol
 * @param starting address value for the array which will store the current voltage values
 * @return starting address
 * @attention The syntax is with a pointer. This was made in this format keeping in mind the problem of dangling pointer.
 *
 * */
uint8_t *ads_spi_transfer_block(uint8_t *rrxx);

/**
 * @brief Function definition used a callback to check if the EDMA Transfer is successful or not for ADS Board
 *
 * @param base DSPI peripheral base address
 * @param handle DSPI handle pointer to dspi_master_edma_handle_t
 * @param status using the status flag verifies if the EDMA Transfer is successful or not
 * @param userData A callback function parameter
 * @return None
 * */
void ADS_DSPI_MasterUserCallback(SPI_Type *base, dspi_master_edma_handle_t *handle, status_t status, void *userData);

/**
 * @brief Function definition for DAC
 * */
void DAC_init(void);

/*
 *@brief Function defintion for updating DAC Value
 * */
void DAC_Update(int, int);

//Closes the @defgroup block. Always kept last.

/**@}*/
