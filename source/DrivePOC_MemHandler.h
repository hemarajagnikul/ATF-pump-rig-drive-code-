/**
 ******************************************************************************
 * @file           : DrivePOC_MemHandler.h
 * @brief          : Functionalities pertaining to all Memory and storage
 *                   operations of the Drive POC Controller implementation
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
 * @defgroup group2 Memory Handler
 * @brief Memory Handler group
 * @details Group pertaining to all Memory and storage operations
 * of the Drive POC Controller implementation
 * @{
 */

/*******************************************************************************
 * Header inclusions
 ******************************************************************************/
#include "fsl_debug_console.h"
#include "gmclib_types.h"
#include "DrivePOC_Control_Loop.h"

//!@name Sine Frequency
//!@{
#define FSINE_800 800
#define FSINE_669 669
#define FSINE_600 600
#define FSINE_500 500
#define FSINE_400 400
#define FSINE_300 300
#define FSINE_200 200
#define FSINE_100 100
//!@}


//!@name Switching Frequency 50kHz Data
//!@{
#define FSWITCHING_800HZ_50KHZ 50400
#define FSWITCHING_669HZ_50KHZ 50175
#define FSWITCHING_600HZ_50KHZ 50400
#define FSWITCHING_500HZ_50KHZ 49500
#define FSWITCHING_400HZ_50KHZ 50400
#define FSWITCHING_300HZ_50KHZ 50400
#define FSWITCHING_200HZ_50KHZ 49800
#define FSWITCHING_100HZ_50KHZ 50100
#define SINE_LUT_800HZ_50KHZ	21
#define SINE_LUT_669HZ_50KHZ	25
#define SINE_LUT_600HZ_50KHZ	28
#define SINE_LUT_500HZ_50KHZ	33
#define SINE_LUT_400HZ_50KHZ	42
#define SINE_LUT_300HZ_50KHZ	56
#define SINE_LUT_200HZ_50KHZ	83
#define SINE_LUT_100HZ_50KHZ	167
//!@}

//!@name Switching Frequency 40kHz Data
//!@{
#define FSWITCHING_800HZ_40KHZ 40800
#define FSWITCHING_669HZ_40KHZ 40140
#define FSWITCHING_600HZ_40KHZ 39600
#define FSWITCHING_500HZ_40KHZ 40500
#define FSWITCHING_400HZ_40KHZ 39600
#define FSWITCHING_300HZ_40KHZ 39600
#define FSWITCHING_200HZ_40KHZ 40200
#define FSWITCHING_100HZ_40KHZ 40200
#define SINE_LUT_800HZ_40KHZ    17
#define SINE_LUT_669HZ_40KHZ	20
#define SINE_LUT_600HZ_40KHZ	22
#define SINE_LUT_500HZ_40KHZ	27
#define SINE_LUT_400HZ_40KHZ	33
#define SINE_LUT_300HZ_40KHZ	44
#define SINE_LUT_200HZ_40KHZ	67
#define SINE_LUT_100HZ_40KHZ	134
//!@}


//!@name Switching Frequency 30kHz Data
//!@{
#define FSWITCHING_800HZ_30KHZ 31200
#define FSWITCHING_669HZ_30KHZ 30105
#define FSWITCHING_600HZ_30KHZ 30600
#define FSWITCHING_500HZ_30KHZ 30000
#define FSWITCHING_400HZ_30KHZ 30000
#define FSWITCHING_300HZ_30KHZ 29700
#define FSWITCHING_200HZ_30KHZ 30000
#define FSWITCHING_100HZ_30KHZ 30000
#define SINE_LUT_800HZ_30KHZ	13
#define SINE_LUT_669HZ_30KHZ	15
#define SINE_LUT_600HZ_30KHZ	17
#define SINE_LUT_500HZ_30KHZ	20
#define SINE_LUT_400HZ_30KHZ	25
#define SINE_LUT_300HZ_30KHZ	33
#define SINE_LUT_200HZ_30KHZ	50
#define SINE_LUT_100HZ_30KHZ	100
//!@}

//!@name Switching Frequency 20kHz Data
//!@{
#define FSWITCHING_1000HZ_20KHZ 18000
#define FSWITCHING_800HZ_20KHZ 19200
#define FSWITCHING_669HZ_20KHZ 20070
#define FSWITCHING_600HZ_20KHZ 19800
#define FSWITCHING_500HZ_20KHZ 19500
#define FSWITCHING_400HZ_20KHZ 20400
#define FSWITCHING_300HZ_20KHZ 19800
#define FSWITCHING_200HZ_20KHZ 20400
#define FSWITCHING_100HZ_20KHZ 20100
#define SINE_LUT_1000HZ_20KHZ	6
#define SINE_LUT_800HZ_20KHZ	8
#define SINE_LUT_669HZ_20KHZ	10
#define SINE_LUT_600HZ_20KHZ	11
#define SINE_LUT_500HZ_20KHZ	13
#define SINE_LUT_400HZ_20KHZ	17
#define SINE_LUT_300HZ_20KHZ	22
#define SINE_LUT_200HZ_20KHZ	37
#define SINE_LUT_100HZ_20KHZ	67
//!@}


//!@name Switching Frequency 10kHz Data
//!@{
#define FSWITCHING_800HZ_10KHZ 9600
#define FSWITCHING_669HZ_10KHZ 10035
#define FSWITCHING_600HZ_10KHZ 10800
#define FSWITCHING_500HZ_10KHZ 10500
#define FSWITCHING_400HZ_10KHZ 9600
#define FSWITCHING_300HZ_10KHZ 9900
#define FSWITCHING_200HZ_10KHZ 10200
#define FSWITCHING_100HZ_10KHZ 10200

#define SINE_LUT_800HZ_10KHZ	4
#define SINE_LUT_669HZ_10KHZ	5
#define SINE_LUT_600HZ_10KHZ	6
#define SINE_LUT_500HZ_10KHZ	7
#define SINE_LUT_400HZ_10KHZ	8
#define SINE_LUT_300HZ_10KHZ	11
#define SINE_LUT_200HZ_10KHZ	17
#define SINE_LUT_100HZ_10KHZ	34

#define SINE_LUT_27P7HZ_10KHZ	68
//!@}

//!@name Interrupt Timing values
//!@{
#define T_SWITCHING_50KHZ 1000000/50000
#define T_SWITCHING_40KHZ 1000000/40000
#define T_SWITCHING_30KHZ 1000000/30000
#define T_SWITCHING_14KHZ 1000000/14000
#define T_SWITCHING_20KHZ 1000000/10000
//#define T_SWITCHING_16KHZ 1000000/30000
#define T_SWITCHING_10KHZ 1000000/10000
//!@}

//!@name LTC Interrupt Frequency
//!@{
#define LTC_SWITCHING_FREQ_10KHZ 10000
#define LTC_SWITCHING_FREQ_20KHZ 20000
#define LTC_SWITCHING_FREQ_30KHZ 30000
#define LTC_SWITCHING_FREQ_40KHZ 40000
#define LTC_SWITCHING_FREQ_50KHZ 50000
//!@}


//!@name ADS Board Parameters
//!@{
#define ADS_BOARD_POSITIVE_VOLTAGE_LIMIT 0x07FFF
#define ADS_BOARD_ZERO_VAL 0.0F
//!@}

extern float_t EDBA,EDBB,EDBC,K0,O0,O1,EDB_max,EDB_min,offset_add,offset_add1,sector;
/*******************************************************************************
 * Function prototypes
 ******************************************************************************/
/**
 * @brief Function definition to update Encoder speed into Memory handler
 * @param  float rpm_recd - Speed of encoder calculated using input pulses in RPM
 * @return True, if successful
 * @return False, if failed
 *
 * */
bool DrivePOC_MH_UpdateEncoderSpeed(float rpm_recd);


/**
 * @brief Function definition to return Encoder speed from Memory handler
 * @param  None
 * @return float - Speed of encoder in RPM stored in Memory Handler
 *
 * */
float DrivePOC_MH_GetEncoderSpeed(void);


/**
 * @brief stores the value of temperature measured using PT-1000 RTD in the memory
 * @param Temperature value from ADC
 * @return None
 * */
void Store_Temperature_from_PT_1000(void);



/**
 *@brief Function definition that gets the value of duty cycle from the duty cycle array.
 *@param None
 *@return None
 * */
void Get_Duty_Cycle(void);


/**
 * @brief Function scales the value of Current using the Sensor scaling factors
 * @param None
 * @return None
 * */
void DrivePOC_MH_UpdateVIvalues(void);


/**
 * @brief Function definition to allocate the variables to corresponding structures
 * @param A global pointer variable
 * @return None
 * */
void DrivePOC_MH_GetVIvalues(void);


/**
 * @brief Function Defintion to give duty cycle during ramp up
 * */
void Get_V_F_Duty_Cycle(void);


/**
 * @brief Function Declaration for Deceleration case
 */
void Get_Deceleration_Duty_Cycle(void);


/**
 * @brief Function Declaration to get Start Up state
 */
bool Get_Start_Up_status(void);


/**
 * @brief Function Declaration to get Stop done signal
 */
bool Get_Stop_status(void);

/**@}*/
bool is_motor_stopped();

extern int get_out;
extern int motor_run;
extern float_t mod_sf;
extern float_t freq;
