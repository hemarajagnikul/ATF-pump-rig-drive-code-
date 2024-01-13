/**
 ******************************************************************************
 * @file           : Drive_Parameters.h
 * @brief          : File containing the motor and drive constant parameters
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
#include "math.h"
#include "stdlib.h"
/*This file will contain the motor and drive parameters*/


//!@name MOTOR PARAMETERS
//!@{
#define PI 3.14f
#define LM 0.001296f			/**<Magnetizing Inductance in henry*/
#define RS 0.0054f				/**<Stator resistance in ohm*/
#define RR 0.0043f				/**<Rotor resistance in ohm*/
#define LLS 0.000021f			/**<Stator Leakage Inductance in henry*/
#define LLR 0.000011f			/**<Rotor Leakage Inductance in henry*/
#define J 0.009370048f			/**<Moment of Inertia in kg*m*m*/
#define B 0.0007f				/**<Friction in Nms*/
#define POLE 2					/**<Number of poles*/
#define POLE_PAIR 1				/**<Number of pole pairs*/
//#define F_NOM	669				/**<Nominal frequency in Hz*/
#define V_NOM_LL_RMS	345		/**<Rated line to line voltage in volt*/
#define I_NOM_LL_RMS	153		/**<Rated line to line current in ampere*/
#define V_NOM_PHASE_RMS	199		/**<Rated phase voltage in volt*/
#define I_NOM_PHASE_RMS 153		/**<Rated phase current in ampere*/
#define RATED_PF	0.87f		/**<Rated power factor*/
#define MOTOR_EFFICIENCY 0.934f /**<efficiency*/
#define MIN_FREQ 1			/**<Minimum frequency in Hz*/
#define MAX_FREQ 667			/**<Maximum frequency in Hz*/
#define LR 0.001307f			/**<Rotor inductance*/
#define FC 16					/**<Flux filter cut off frequency*/
#define WC 628.32f				/**<Speed filter cut off frequency*/
#define VOUT_INVERTER_MAX 9000 	/**<Inverter Output Voltage Maximum value*/
#define VIN_INVERTER_MAX 9000	/**<Inverter Input Voltage Maximum value*/
#define VOUT_INVERTER_MIN 20	/**<Inverter Output Voltage Minimum value*/
#define VIN_INVERTER_MIN 20		/**<Inverter Input Voltage Minimum value*/
#define IOUT_INVERTER_MAX 10000 /**<Inverter Output Current Maximum value*/
#define IIN_INVERTER_MAX  10000 /**<Inverter Input Current Maximum value*/
#define IOUT_INVERTER_MIN 2		/**<Inverter Output Current Minimum value*/
#define IIN_INVERTER_MIN  2		/**<Inverter Input Current Minimum value*/
#define MOTOR_MAX_TEMPERATURE 120/**<Allowable maximum temperature for the Motor Windings in degree Celsius*/
//!@}
//#define motor_run 0

//!@name BATTERY SIDE PARAMETERS
//!@{
#define VDC_NOM	560			/**<DC Nominal Voltage in volt*/
#define VDC_MIN	500			/**<DC Minimum Voltage in volt*/
#define VDC_MAX 1000		/**<DC Maximum Voltage in volt*/
//!@}

//!@name SAMPLING RATES
//!@{
#define TVECT 0.00001f/**< Fast loop running rate*/
#define TSC 0.0004f/**< Slow loop running rate*/
#define TS 0.00001f/**<Sampling time for V&I */


///\todo:-Ts value has to be updated after identifying latencies.
//!@}


//!@name SENSOR PARAMETERS
//!@{
///\todo:-The current sensor scale factor values have to be updated.
#define CURRENT_SENSOR_SCALE_FACTOR_PHASE_A 1.0F
#define CURRENT_SENSOR_SCALE_FACTOR_PHASE_B 1.0F
#define CURRENT_SENSOR_SCALE_FACTOR_PHASE_C 1.0F
#define CURRENT_SENSOR_SCALE_FACTOR_DC 1.0F
#define VOLTAGE_SENSOR_SCALE_FACTOR_PHASE_A 140.0F
#define VOLTAGE_SENSOR_SCALE_FACTOR_PHASE_B 140.0F
#define VOLTAGE_SENSOR_SCALE_FACTOR_PHASE_C 140.0F
#define VOLTAGE_SENSOR_SCALE_FACTOR_DC 140.0F
//!@}



//!@name User Inputs
//!@{
#define LUT_TEMP_VAL SINE_LUT_27P7HZ_10KHZ
//#define SWITCHING_FREQUENCY 16000
//#define SWITCHING_TIME 1000000/SWITCHING_FREQUENCY
#define LTC_FACTOR LTC_SWITCHING_FREQ_50KHZ
#define MODULATION_INDEX 1.0F
//#define M (FREQ_NEEDED)/MAX_FREQ
//#define LUT_FREQUENCY ((float)(SWITCHING_FREQUENCY/(3*68)))
//#define FREQ_NEEDED (float) 300
//#define RAMP_TIME 10U
#define BIT_MAX 4096
//#define SWITCHINGTIME (float_t) 1/SWITCHING_FREQUENCY
//#define FREQ_MAP (float_t)((float_t)BIT_MAX/(float_t)FREQ_NEEDED)
//#define MAX_NUM_OF_LUT_SIZE SWITCHING_FREQUENCY/LUT_FREQUENCY
//#define START_UP_OK (int)(RAMP_TIME*SWITCHING_FREQUENCY)
//#define DECELERATION_TIME 30
//#define STOP_OK	(int)(DECELERATION_TIME*SWITCHING_FREQUENCY)
//!@}
