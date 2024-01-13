/**
 ******************************************************************************
 * @file           : DrivePOC_MemHandler.c
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


/*******************************************************************************
 * Header inclusions
 ******************************************************************************/

#include "Drive_Parameters.h"
#include "DrivePOC_Controller_NXP.h"
#include "DrivePOC_MemHandler.h"
#include "DrivePOC_CommHandler.h"
#include "DrivePOC_FaultHandler.h"
#include "fsl_gpio.h"
#include "fsl_dac.h"
//#include "DrivePOC_Control_Loop.h"
//#include "DrivePOC_control_Loop.h"
/* Structure containing data of three phase stationary system */

static float g_encoder_speed;
float_t g_temperature;
int g_phase_shift_A=0;
int g_phase_shift_B=LUT_TEMP_VAL;
int g_phase_shift_C=LUT_TEMP_VAL*2;
int accu=0;
int count=0;
float_t freq_prev=(float_t)MIN_FREQ;
float_t EDBA,EDBB,EDBC,K0,O0=0.5,O1=0.5,EDB_max,EDB_min,offset_add,sector;
float_t freq;
int rem=0;
float_t	mod_sf=1;
int start_check=0;
int end_check=0;
bool start_ok=false;
bool stop_ok=false;
int bit_max=BIT_MAX;
//int motor_run=0;
//float_t accel_del_freq=(FREQ_NEEDED-MIN_FREQ)*SWITCHINGTIME/(RAMP_TIME);
//float_t decel_del_freq=(FREQ_NEEDED-MIN_FREQ)*SWITCHINGTIME/(DECELERATION_TIME);
//float_t f_freq_map=((float_t)BIT_MAX/(float_t)FREQ_NEEDED);


//!@name Duty Cycle LUTs for 50kHz Switching Frequency
//!@{
float_t duty_cycle_50000hz_800hz[SINE_LUT_800HZ_50KHZ*3]={0.500000F,0.549784F,0.599073F,0.647378F,0.694217F,0.739127F,0.781660F,0.821394F,0.857933F,0.890916F,0.920013F,0.944936F,0.965437F,0.981312F,0.992404F,0.998602F,0.999845F,0.996120F,0.987464F,0.973964F,0.955753F,0.933013F,0.905969F,0.874891F,0.840086F,0.801902F,0.760718F,0.716942F,0.671010F,0.623379F,0.574521F,0.524923F,0.475077F,0.425479F,0.376621F,
		0.328990F,0.283058F,0.239282F,0.198098F,0.159914F,0.125109F,0.094031F,0.066987F,0.044247F,0.026036F,0.012536F,0.003880F,0.000155F,0.001398F,0.007596F,0.018688F,0.034563F,0.055064F,0.079987F,0.109084F,0.142067F,0.178606F,0.218340F,0.260873F,0.305783F,0.352622F,0.400927F,0.450216F};

float_t duty_cycle_50000hz_669hz[SINE_LUT_669HZ_50KHZ*3]={0.500000F,0.541839F,0.583384F,0.624345F,0.664433F,0.703368F,0.740877F,0.776696F,0.810574F,0.842274F,0.871572F,0.898265F,0.922164F,0.943102F,0.960932F,0.975528F,0.986789F,0.994636F,0.999013F,0.999890F,0.997261F,0.991144F,0.981581F,0.968641F,0.952414F,0.933013F,0.910575F,0.885257F,0.857236F,0.826710F,0.793893F,0.759014F,0.722318F,0.684062F,0.644516F,
		0.603956F,0.562667F,0.520938F,0.479062F,0.437333F,0.396044F,0.355484F,0.315938F,0.277682F,0.240986F,0.206107F,0.173290F,0.142764F,0.114743F,0.089425F,0.066987F,0.047586F,0.031359F,0.018419F,0.008856F,0.002739F,0.000110F,0.000987F,0.005364F,0.013211F,0.024472F,0.039068F,0.056898F,0.077836F,0.101735F,0.128428F,0.157726F,0.189426F,0.223304F,0.259123F,0.296632F,0.335567F,0.375655F,0.416616F,0.458161F};
//!@}

//!@name Duty Cycle LUTs for 40kHz Switching Frequency
//!@{
float_t duty_cycle_40000hz_800hz[SINE_LUT_800HZ_40KHZ*3]={0.500000F,0.561444F,0.621957F,0.680621F,0.736547F,0.788887F,0.836848F,0.879702F,0.916801F,0.947582F,0.971577F,0.988424F,0.997867F,0.999763F,0.994083F,0.980913F,0.960453F,0.933013F,0.899009F,0.858956F,0.813462F,0.763216F,0.708980F,0.651576F,0.591875F,0.530780F,0.469220F,0.408125F,0.348424F,0.291020F,0.236784F,0.186538F,0.141044F,0.100991F,0.066987F,
		0.039547F,0.019087F,0.005917F,0.000237F,0.002133F,0.011576F,0.028423F,0.052418F,0.083199F,0.120298F,0.163152F,0.211113F,0.263453F,0.319379F,0.378043F,0.438556F};

float_t duty_cycle_40000hz_669hz[SINE_LUT_669HZ_40KHZ*3]={0.500000F,0.552264F,0.603956F,0.654508F,0.703368F,0.750000F,0.793893F,0.834565F,0.871572F,0.904508F,0.933013F,0.956773F,0.975528F,0.989074F,0.997261F,1.000000F,0.997261F,0.989074F,0.975528F,0.956773F,0.933013F,0.904508F,0.871572F,0.834565F,0.793893F,0.750000F,0.703368F,0.654508F,0.603956F,0.552264F,0.500000F,0.447736F,0.396044F,0.345492F,0.296632F,
		0.250000F,0.206107F,0.165435F,0.128428F,0.095492F,0.066987F,0.043227F,0.024472F,0.010926F,0.002739F,0.000000F,0.002739F,0.010926F,0.024472F,0.043227F,0.066987F,0.095492F,0.128428F,0.165435F,0.206107F,0.250000F,0.296632F,0.345492F,0.396044F,0.447736F};
//!@}

//!@name Duty Cycle LUTs for 30kHz Switching Frequency
//!@{
float_t duty_cycle_30000hz_800hz[SINE_LUT_800HZ_30KHZ*3]={0.500000F,0.580206F,0.658334F,0.732362F,0.800371F,0.860601F,0.911492F,0.951725F,0.980259F,0.996354F,0.999594F,0.989895F,0.967508F,0.933013F,0.887302F,0.831561F,0.767233F,0.695983F,0.619658F,0.540233F,0.459767F,0.380342F,0.304017F,0.232767F,0.168439F,0.112698F,0.066987F,0.032492F,0.010105F,0.000406F,0.003646F,0.019741F,0.048275F,0.088508F,0.139399F,
		0.199629F,0.267638F,0.341666F,0.419794F};

float_t duty_cycle_30000hz_669hz[SINE_LUT_669HZ_30KHZ*3]={0.500000F,0.569587F,0.637819F,0.703368F,0.764960F,0.821394F,0.871572F,0.914519F,0.949397F,0.975528F,0.992404F,0.999695F,0.997261F,0.985148F,0.963592F,0.933013F,0.894005F,0.847329F,0.793893F,0.734736F,0.671010F,0.603956F,0.534878F,0.465122F,0.396044F,0.328990F,0.265264F,0.206107F,0.152671F,0.105995F,0.066987F,0.036408F,0.014852F,0.002739F,0.000305F,
		0.007596F,0.024472F,0.050603F,0.085481F,0.128428F,0.178606F,0.235040F,0.296632F,0.362181F,0.430413F};
//!@}

//!@name Duty Cycle LUTs for 20kHz Switching Frequency
//!@{
float_t duty_cycle_20000hz_1000hz[SINE_LUT_1000HZ_20KHZ*3]={0.5F,0.6710101F,0.8213938F,0.9330127F,0.9924039F,0.9924039F,0.9330127F,0.8213938F,0.6710101F,0.5000000F,0.3289899F,0.1786062F,0.6698730F,0.7596123F,0.007596123F,0.06698730F,0.1786062F,0.3289899F};

float_t duty_cycle_20000hz_800hz[SINE_LUT_800HZ_20KHZ*3]={0.500000F,0.629410F,0.750000F,0.853553F,0.933013F,0.982963F,1.000000F,0.982963F,0.933013F,0.853553F,0.750000F,0.629410F,0.500000F,0.370590F,0.250000F,0.146447F,0.066987F,0.017037F,0.000000F,0.017037F,0.066987F,0.146447F,0.250000F,0.370590F};

float_t duty_cycle_20000hz_669hz[SINE_LUT_669HZ_20KHZ*3]={0.500000F,0.603956F,0.703368F,0.793893F,0.871572F,0.933013F,0.975528F,0.997261F,0.997261F,0.975528F,0.933013F,0.871572F,0.793893F,0.703368F,0.603956F,0.500000F,0.396044F,0.296632F,0.206107F,0.128428F,0.066987F,0.024472F,0.002739F,0.002739F,0.024472F,0.066987F,0.128428F,0.206107F,0.296632F,0.396044F};
//!@}


//!@name Duty Cycle LUTs for 10kHz Switching Frequency
//!@{
float_t duty_cycle_10000hz_800hz[SINE_LUT_800HZ_10KHZ*3]={0.500000F,0.750000F,0.933013F,1.000000F,0.933013F,0.750000F,0.500000F,0.250000F,0.066987F,0.000000F,0.066987F,0.250000F};

float_t duty_cycle_10000hz_669hz[SINE_LUT_669HZ_10KHZ*3]={0.500000F,0.703368F,0.871572F,0.975528F,0.997261F,0.933013F,0.793893F,0.603956F,0.396044F,0.206107F,0.066987F,0.002739F,0.024472F,0.128428F,0.296632F};
//!@}

float_t duty_cycle_10000hz_100hz[SINE_LUT_100HZ_10KHZ*3]={0.5,0.5307804531,0.5614441453,0.5918747589,0.6219568601,0.6515763371,0.6806208331,0.7089801724,0.7365467784,0.7632160814,0.7888869157,0.8134619029,0.8368478218,0.8589559615,0.8797024583,0.8990086136,0.9168011926,0.9330127019,0.9475816457,0.960452759,0.9715772172,0.9809128216,0.9884241589,0.994082736,0.9978670881,0.9997628599,0.9997628599,0.9978670881,0.994082736,0.9884241589,0.9809128216,0.9715772172,0.960452759,0.9475816457,0.9330127019,0.9168011926,0.8990086136,0.8797024583,0.8589559615,0.8368478218,0.8134619029,0.7888869157,0.7632160814,0.7365467784,0.7089801724,0.6806208331,0.6515763371,0.6219568601,0.5918747589,0.5614441453,0.5307804531,0.5,0.4692195469,0.4385558547,0.4081252411,0.3780431399,0.3484236629,0.3193791669,0.2910198276,0.2634532216,0.2367839186,0.2111130843,0.1865380971,0.1631521782,0.1410440385,0.1202975417,0.1009913864,0.08319880739,0.06698729811,0.05241835432,0.03954724103,0.02842278276,0.01908717841,0.01157584112,0.005917263959,0.002132911852,0.0002371401433,0.0002371401433,0.002132911852,0.005917263959,0.01157584112,0.01908717841,0.02842278276,0.03954724103,0.05241835432,0.06698729811,0.08319880739,0.1009913864,0.1202975417,0.1410440385,0.1631521782,0.1865380971,0.2111130843,0.2367839186,0.2634532216,0.2910198276,0.3193791669,0.3484236629,0.3780431399,0.4081252411,0.4385558547,0.4692195469};
//float_t duty_cycle_10000hz_50hz[360] = {0.500000, 0.508726, 0.517450, 0.526168, 0.534878, 0.543578, 0.552264, 0.560935, 0.569587, 0.578217, 0.586824, 0.595404, 0.603956, 0.612476, 0.620961, 0.629410, 0.637819, 0.646186, 0.654508, 0.662784, 0.671010, 0.679184, 0.687303, 0.695366, 0.703368, 0.711309, 0.719186, 0.726995, 0.734736, 0.742405, 0.750000, 0.757519, 0.764960, 0.772320, 0.779596, 0.786788, 0.793893, 0.800908, 0.807831, 0.814660, 0.821394, 0.828030, 0.834565, 0.840999, 0.847329, 0.853553, 0.859670, 0.865677, 0.871572, 0.877355, 0.883022, 0.888573, 0.894005, 0.899318, 0.904508, 0.909576, 0.914519, 0.919335, 0.924024, 0.928584, 0.933013, 0.937310, 0.941474, 0.945503, 0.949397, 0.953154, 0.956773, 0.960252, 0.963592, 0.966790, 0.969846, 0.972759, 0.975528, 0.978152, 0.980631, 0.982963, 0.985148, 0.987185,0.989074, 0.990814, 0.992404, 0.993844, 0.995134, 0.996273, 0.997261, 0.998097, 0.998782, 0.999315, 0.999695, 0.999924, 1.000000, 0.999924, 0.999695, 0.999315, 0.998782, 0.998097, 0.997261, 0.996273, 0.995134, 0.993844, 0.992404, 0.990814, 0.989074, 0.987185, 0.985148, 0.982963, 0.980631, 0.978152, 0.975528, 0.972759, 0.969846, 0.966790, 0.963592, 0.960252, 0.956773, 0.953154, 0.949397, 0.945503, 0.941474, 0.937310, 0.933013, 0.928584, 0.924024, 0.919335, 0.914519, 0.909576, 0.904508, 0.899318, 0.894005, 0.888573, 0.883022, 0.877355, 0.871572, 0.865677, 0.859670, 0.853553, 0.847329, 0.840999, 0.834565, 0.828030, 0.821394, 0.814660, 0.807831, 0.800908, 0.793893, 0.786788, 0.779596, 0.772320, 0.764960, 0.757519, 0.750000, 0.742405, 0.734736, 0.726995, 0.719186, 0.711309,0.703368, 0.695366, 0.687303, 0.679184, 0.671010, 0.662784, 0.654508, 0.646186, 0.637819, 0.629410, 0.620961, 0.612476, 0.603956, 0.595404, 0.586824, 0.578217, 0.569587, 0.560935, 0.552264, 0.543578, 0.534878, 0.526168, 0.517450, 0.508726, 0.500000, 0.491274, 0.482550, 0.473832, 0.465122, 0.456422, 0.447736, 0.439065, 0.430413, 0.421783, 0.413176, 0.404596, 0.396044, 0.387524, 0.379039, 0.370590, 0.362181, 0.353814, 0.345492, 0.337216, 0.328990, 0.320816, 0.312697, 0.304634, 0.296632, 0.288691, 0.280814, 0.273005, 0.265264, 0.257595, 0.250000, 0.242481, 0.235040, 0.227680, 0.220404, 0.213212, 0.206107, 0.199092, 0.192169, 0.185340, 0.178606, 0.171970, 0.165435, 0.159001, 0.152671, 0.146447, 0.140330, 0.134323, 0.128428, 0.122645, 0.116978, 0.111427, 0.105995, 0.100682,0.095492, 0.090424, 0.085481, 0.080665, 0.075976, 0.071416, 0.066987, 0.062690, 0.058526, 0.054497, 0.050603, 0.046846, 0.043227, 0.039748, 0.036408, 0.033210, 0.030154, 0.027241, 0.024472, 0.021848, 0.019369, 0.017037, 0.014852, 0.012815, 0.010926, 0.009186, 0.007596, 0.006156, 0.004866, 0.003727, 0.002739, 0.001903, 0.001218, 0.000685, 0.000305, 0.000076, 0.000000, 0.000076, 0.000305, 0.000685, 0.001218, 0.001903, 0.002739, 0.003727, 0.004866, 0.006156, 0.007596, 0.009186, 0.010926, 0.012815, 0.014852, 0.017037, 0.019369, 0.021848, 0.024472, 0.027241, 0.030154, 0.033210, 0.036408, 0.039748, 0.043227, 0.046846, 0.050603, 0.054497, 0.058526, 0.062690, 0.066987, 0.071416, 0.075976, 0.080665, 0.085481, 0.090424, 0.095492, 0.100682, 0.105995, 0.111427, 0.116978, 0.122645,0.128428, 0.134323, 0.140330, 0.146447, 0.152671, 0.159001, 0.165435, 0.171970, 0.178606, 0.185340, 0.192169, 0.199092, 0.206107, 0.213212, 0.220404, 0.227680, 0.235040, 0.242481, 0.250000, 0.257595, 0.265264, 0.273005, 0.280814, 0.288691, 0.296632, 0.304634, 0.312697, 0.320816, 0.328990, 0.337216, 0.345492, 0.353814, 0.362181, 0.370590, 0.379039, 0.387524, 0.396044, 0.404596, 0.413176, 0.421783, 0.430413, 0.439065, 0.447736, 0.456422, 0.465122, 0.473832, 0.482550, 0.491274};
//float_t duty_cycle_10000hz_50hz[204] = {0.500000, 0.515398, 0.530780, 0.546134, 0.561444, 0.576696, 0.591875, 0.606967, 0.621957, 0.636831, 0.651576, 0.666177, 0.680621, 0.694893, 0.708980, 0.722869, 0.736547, 0.750000, 0.763216, 0.776182, 0.788887, 0.801317, 0.813462, 0.825309, 0.836848, 0.848067, 0.858956, 0.869504, 0.879702, 0.889540, 0.899009, 0.908098, 0.916801, 0.925109, 0.933013, 0.940506, 0.947582, 0.954233, 0.960453, 0.966236, 0.971577, 0.976471, 0.980913, 0.984898, 0.988424, 0.991487, 0.994083, 0.996210, 0.997867, 0.999052, 0.999763, 1.000000, 0.999763, 0.999052, 0.997867, 0.996210, 0.994083, 0.991487, 0.988424, 0.984898, 0.980913, 0.976471, 0.971577, 0.966236, 0.960453, 0.954233, 0.947582, 0.940506, 0.933013, 0.925109, 0.916801, 0.908098, 0.899009, 0.889540, 0.879702, 0.869504, 0.858956, 0.848067,0.836848, 0.825309, 0.813462, 0.801317, 0.788887, 0.776182, 0.763216, 0.750000, 0.736547, 0.722869, 0.708980, 0.694893, 0.680621, 0.666177, 0.651576, 0.636831, 0.621957, 0.606967, 0.591875, 0.576696, 0.561444, 0.546134, 0.530780, 0.515398, 0.500000, 0.484602, 0.469220, 0.453866, 0.438556, 0.423304, 0.408125, 0.393033, 0.378043, 0.363169, 0.348424, 0.333823, 0.319379, 0.305107, 0.291020, 0.277131, 0.263453, 0.250000, 0.236784, 0.223818, 0.211113, 0.198683, 0.186538, 0.174691, 0.163152, 0.151933, 0.141044, 0.130496, 0.120298, 0.110460, 0.100991, 0.091902, 0.083199, 0.074891, 0.066987, 0.059494, 0.052418, 0.045767, 0.039547, 0.033764, 0.028423, 0.023529, 0.019087, 0.015102, 0.011576, 0.008513, 0.005917, 0.003790, 0.002133, 0.000948, 0.000237, 0.000000, 0.000237, 0.000948,0.002133, 0.003790, 0.005917, 0.008513, 0.011576, 0.015102, 0.019087, 0.023529, 0.028423, 0.033764, 0.039547, 0.045767, 0.052418, 0.059494, 0.066987, 0.074891, 0.083199, 0.091902, 0.100991, 0.110460, 0.120298, 0.130496, 0.141044, 0.151933, 0.163152, 0.174691, 0.186538, 0.198683, 0.211113, 0.223818, 0.236784, 0.250000, 0.263453, 0.277131, 0.291020, 0.305107, 0.319379, 0.333823, 0.348424, 0.363169, 0.378043, 0.393033, 0.408125, 0.423304, 0.438556, 0.453866, 0.469220, 0.484602};/**< User Input - User has to change this value declared to this pointer corresponding to the Switching frequency and AC Sine Frequency he/she wish to operate*/

float_t duty_cycle_10000hz_50hz[204] = {0.5000,	0.5267,	0.5533,	0.5799,	0.6064,	0.6328,	0.6590,	0.6852,	0.7111,	0.7369,	0.7624,	0.7877,	0.8127,	0.8374,	0.8617,	0.8858,	0.9095,	0.9327,	0.9402,	0.9473,	0.9540,	0.9602,	0.9660,	0.9713,	0.9762,	0.9806,	0.9846,	0.9881,	0.9912,	0.9938,	0.9959,	0.9976,	0.9987,	0.9995,	0.9997,	0.9995,	0.9987,	0.9976,	0.9959,	0.9938,	0.9912,	0.9881,	0.9846,	0.9806,	0.9762,	0.9713,	0.9660,	0.9602,	0.9540,	0.9473,	0.9402,	0.9328,	0.9402,	0.9473,	0.9540,	0.9602,	0.9660,	0.9713,	0.9762,	0.9806,	0.9846,	0.9881,	0.9912,	0.9938,	0.9959,	0.9976,	0.9987,	0.9995,	0.9997,	0.9995,	0.9987,	0.9976,	0.9959,	0.9938,	0.9912,	0.9881,	0.9846,	0.9806,	0.9762,	0.9713,	0.9660,	0.9602,	0.9540,	0.9473,	0.9402,	0.9328,	0.9095,	0.8858,	0.8617,	0.8374,	0.8127,	0.7877,	0.7624,	0.7369,	0.7111,	0.6852,	0.6590,	0.6328,	0.6064,	0.5799,	0.5533,	0.5267,	0.5000,	0.4733,	0.4467,	0.4201,	0.3936,	0.3672,	0.3410,	0.3148,	0.2889,	0.2631,	0.2376,	0.2123,	0.1873,	0.1626,	0.1383,	0.1142,	0.0905,	0.0673,	0.0598,	0.0527,	0.0460,	0.0398,	0.0340,	0.0287,	0.0238,	0.0194,	0.0154,	0.0119,	0.0088,	0.0062,	0.0041,	0.0024,	0.0013,	0.0005,	0.0003,	0.0005,	0.0013,	0.0024,	0.0041,	0.0062,	0.0088,	0.0119,	0.0154,	0.0194,	0.0238,	0.0287,	0.0340,	0.0398,	0.0460,	0.0527,	0.0598,	0.0672,	0.0598,	0.0527,	0.0460,	0.0398,	0.0340,	0.0287,	0.0238,	0.0194,	0.0154,	0.0119,	0.0088,	0.0062,	0.0041,	0.0024,	0.0013,	0.0005,	0.0003,	0.0005,	0.0013,	0.0024,	0.0041,	0.0062,	0.0088,	0.0119,	0.0154,	0.0194,	0.0238,	0.0287,	0.0340,	0.0398,	0.0460,	0.0527,	0.0598,	0.0672,	0.0905,	0.1142,	0.1383,	0.1626,	0.1873,	0.2123,	0.2376,	0.2631,	0.2889,	0.3148,	0.3410,	0.3672,	0.3936,	0.4201,	0.4467,	0.4733};
float_t *duty_cycle=duty_cycle_10000hz_50hz;
//!@{
float_t offset_add1=0;


//!@}

/*******************************************************************************
 * Function definitions
 ******************************************************************************/
/*
 * @brief Function definition to update Encoder speed into Memory handler
 * @param Speed of encoder calculated using input pulses in x units
 * @return True, if successful, False, if failed
 *
 * */
bool DrivePOC_MH_UpdateEncoderSpeed(float rpm_recd)
{
	g_encoder_speed = rpm_recd;
	return true;
}


/*
 * @brief Function definition to return Encoder speed from Memory handler
 * @param None
 * @return Encoder speed value stored in Memory handler
 *
 * */
float DrivePOC_MH_GetEncoderSpeed(void)
{
	return g_encoder_speed;
}



/*
 * @brief stores the value of temperature measured using PT-1000 RTD in the memory
 * @param Temperature value from ADC
 * @return None
 * */
void Store_Temperature_from_PT_1000(void){
	g_temperature=Collect_Data_from_PT_1000();
	PRINTF("%lf\n",g_temperature);
}




/*
 * @brief Function Defintion to give duty cycle during ramp up
 * */
void Get_V_F_Duty_Cycle(void){
	// if(GPIO_PinRead(GPIOC, 8)==1){ write condition here
	stop_ok=false;
	if (motor_run == 0){
		stop_ok=true;
	}


	// Declare freq as integer.
	freq=freq_prev+accel_del_freq*f_freq_map;
	//PRINTF("\n%ld\n",START_UP_OK);

	// Declare this as integer, as no ceil is required.
	rem=((int)ceil(freq));
	int temp_bit_max = bit_max/((float)FREQ_NEEDED/LUT_FREQUENCY);//skipping indices
	accu=accu+rem;
//	if(accu>temp_bit_max){
		g_phase_shift_A=(g_phase_shift_A+(accu/temp_bit_max));
		g_phase_shift_B=(g_phase_shift_B+(accu/temp_bit_max));
		g_phase_shift_C=(g_phase_shift_C+(accu/temp_bit_max));
		accu=accu%temp_bit_max;

		g_phase_shift_A = (g_phase_shift_A)%(3*LUT_TEMP_VAL);
		g_phase_shift_B = (g_phase_shift_B)%(3*LUT_TEMP_VAL);
		g_phase_shift_C = (g_phase_shift_C)%(3*LUT_TEMP_VAL);
//	}
		//if (mod_scheme_select==0){
					mod_sf=(float_t)freq*0.00022*M; //0.00022 = (0.000244*0.9), 0.9 is limitation of modulation index
//				}
//				else{
//					mod_sf=1.145*(float_t)freq*0.000244*M;
//				}
     EDBA =(*(duty_cycle+g_phase_shift_A)-0.5F)*mod_sf;
     EDBB =(*(duty_cycle+g_phase_shift_C)-0.5F)*mod_sf;
     EDBC =(*(duty_cycle+g_phase_shift_B)-0.5F)*mod_sf;

//     if ((EDBA>EDBC)&&(EDBC>=EDBB)){
//    	 EDB_max=EDBA;
//    	 EDB_min=EDBB;
//    	 if (EDBC>0){ //30-60
//    		 K0=O0;
//    		 sector=300;
//    	 }
//    	 if (EDBC<=0){ //60-90
//    	      K0=O1;
//    	      sector=450;
//    	 }
//     }
//
//     if ((EDBA>=EDBB)&&(EDBB>EDBC)){
//    	 EDB_max=EDBA;
//    	 EDB_min=EDBC;
//    	 if (EDBB>=0){ //120-150
//    		 K0=O0;
//    		 sector = 750;
//    	 }
//    	 if (EDBB<0){ //90-120
//    	      K0=O1;
//    	      sector=600;
//    	 }
//     }
//
//     if ((EDBB>EDBA)&&(EDBA>=EDBC)){
//    	 EDB_max=EDBB;
//    	 EDB_min=EDBC;
//    	 if (EDBA>=0){ //150-180
//    		 K0=O0;
//    		 sector=900;
//    	 }
//    	 if (EDBA<0){ //180-210
//    	      K0=O1;
//    	      sector=1050;
//    	 }
//     }
//
//     if ((EDBB>=EDBC)&&(EDBC>EDBA)){
//    	 EDB_max=EDBB;
//    	 EDB_min=EDBA;
//    	 if (EDBC>=0){ //240-270
//    		 K0=O0;
//    		 sector=1350;
//    	 }
//    	 if (EDBC<0){ //210-240
//    	      K0=O1;
//    	      sector=1200;
//    	 }
//     }
//
//     if ((EDBC>EDBB)&&(EDBB>=EDBA)){
//    	 EDB_max=EDBC;
//    	 EDB_min=EDBA;
//    	 if (EDBB>0){ //270-300
//    		 K0=O0;
//    		 sector=1500;
//    	 }
//    	 if (EDBB<=0){ //300-330
//    	      K0=O1;
//    	      sector=1650;
//    	 }
//     }
//
//     if ((EDBC>=EDBA)&&(EDBA>EDBB)){
//    	 EDB_max=EDBC;
//    	 EDB_min=EDBB;
//    	 if (EDBA>0){ //0-30
//    		 K0=O0;
//    		 sector=150;
//
//    	 }
//    	 if (EDBA<=0){ //330-360
//    	      K0=O1;
//    	      sector=1800;
//    	 }
//     }
//
//     offset_add =0;// -(((1-(2*K0)))+(K0*2.0*EDB_max)+((1-K0)*2.0*EDB_min));
//     if (mod_scheme_select == 0){ //Sine PWM
//    	 offset_add1=0;
//     }
//     if (mod_scheme_select == 1){ //SVPWM
//    	 offset_add1 = offset_cal ( EDBA, EDBB, EDBC, 0.5, 0.5);
//         }
//     if (mod_scheme_select == 2){ //DPWM3
//    	 offset_add1 = offset_cal ( EDBA, EDBB, EDBC, 1, 0);
//         }
//     if (mod_scheme_select == 3){ //DPWM1
//         	 offset_add1 = offset_cal ( EDBA, EDBB, EDBC, 0, 1);
//              }
//     EDBA =(2*EDBA) ;
//     EDBB =(2*EDBB);
//     EDBC =(2*EDBC);


	//s_open_loop.s_dutyabc_flt.fltA=0.5F+((1.0*EDBA*M)*(float_t)freq/bit_max);
	//s_open_loop.s_dutyabc_flt.fltB=0.5F+((1.0*EDBB*M)*(float_t)freq/bit_max);
	//s_open_loop.s_dutyabc_flt.fltC=0.5F+((1.0*EDBC*M)*(float_t)freq/bit_max);

	s_open_loop.s_dutyabc_flt.fltA=0.5F+((EDBA));
		s_open_loop.s_dutyabc_flt.fltB=0.5F+((EDBB));
		 s_open_loop.s_dutyabc_flt.fltC=0.5F+((EDBC));


	start_check++;
	freq_prev=freq;
	if(g_phase_shift_A>3*LUT_TEMP_VAL-1){
		g_phase_shift_A=0;
	}
	if(g_phase_shift_B>3*LUT_TEMP_VAL-1){
		g_phase_shift_B=0;
	}
	if(g_phase_shift_C>3*LUT_TEMP_VAL-1){
		g_phase_shift_C=0;
	}
	if(start_check==START_UP_OK){
//		rem=ceil((int)((float_t)FREQ_NEEDED*((float_t)FREQ_MAP-0.001F)));
		rem = 4096;
		accu = 0;
		start_ok=true;
		start_check = 0;
	}
	//DAC_Update((uint16_t)(s_open_loop.s_dutyabc_flt.fltA*0xFFF),(uint16_t)(s_open_loop.s_dutyabc_flt.fltC*0xFFF));
}




/*
 *@brief Function definition that gets the value of duty cycle from the duty cycle array.
 *@param None
 *@return None
 * */
void Get_Duty_Cycle(void){
//	if(GPIO_PinRead(GPIOC, 8)==1){ // write condition here
	stop_ok=false;
	start_ok=false;
	if (motor_run == 0){
		stop_ok=true;
	}
	accu=accu+rem; //=4096
//	bit_max = 1024;
	int temp_bit_max = bit_max/((float)FREQ_NEEDED/LUT_FREQUENCY); //=4096
	if (accu > temp_bit_max) {
//		if (g_phase_shift_A == 101) {
//			g_phase_shift_A = 0;
//		}
//		if (g_phase_shift_B == 101) {
//			g_phase_shift_B = 0;
//		}
//		if (g_phase_shift_C == 101) {
//			g_phase_shift_C = 0;
//		}

		g_phase_shift_A = (g_phase_shift_A + (accu / temp_bit_max));
		g_phase_shift_B = (g_phase_shift_B + (accu / temp_bit_max));
		g_phase_shift_C = (g_phase_shift_C + (accu / temp_bit_max));
//		if (g_phase_shift_A == 3*LUT_TEMP_VAL) {
//			g_phase_shift_A = 3*LUT_TEMP_VAL-1;
//		}
//		if (g_phase_shift_B == 3*LUT_TEMP_VAL) {
//			g_phase_shift_B = 3*LUT_TEMP_VAL-1;
//		}
//		if (g_phase_shift_C == 3*LUT_TEMP_VAL) {
//			g_phase_shift_C = 3*LUT_TEMP_VAL-1;
//		}
		g_phase_shift_A = (g_phase_shift_A)%(3*LUT_TEMP_VAL);
		g_phase_shift_B = (g_phase_shift_B)%(3*LUT_TEMP_VAL);
		g_phase_shift_C = (g_phase_shift_C)%(3*LUT_TEMP_VAL);
		accu=accu%temp_bit_max;
	}
//	bit_max = 4096;
//	if (mod_scheme_select==0){
				mod_sf=(float_t)freq*0.00022*M;
//			}
//			else{
//				mod_sf=1.145*(float_t)freq*0.000244*M;
//			}
    EDBA =(*(duty_cycle+g_phase_shift_A)-0.5F)*mod_sf;
    EDBB =(*(duty_cycle+g_phase_shift_C)-0.5F)*mod_sf;
    EDBC =(*(duty_cycle+g_phase_shift_B)-0.5F)*mod_sf;


//    if ((EDBA>EDBC)&&(EDBC>=EDBB)){
//   	 EDB_max=EDBA;
//   	 EDB_min=EDBB;
//   	 if (EDBC>0){ //30-60
//   		 K0=O0;
//   		 sector=300;
//   	 }
//   	 if (EDBC<=0){ //60-90
//   	      K0=O1;
//   	      sector=450;
//   	 }
//    }
//
//    if ((EDBA>=EDBB)&&(EDBB>EDBC)){
//   	 EDB_max=EDBA;
//   	 EDB_min=EDBC;
//   	 if (EDBB>=0){ //120-150
//   		 K0=O0;
//   		 sector = 750;
//   	 }
//   	 if (EDBB<0){ //90-120
//   	      K0=O1;
//   	      sector=600;
//   	 }
//    }
//
//    if ((EDBB>EDBA)&&(EDBA>=EDBC)){
//   	 EDB_max=EDBB;
//   	 EDB_min=EDBC;
//   	 if (EDBA>=0){ //150-180
//   		 K0=O0;
//   		 sector=900;
//   	 }
//   	 if (EDBA<0){ //180-210
//   	      K0=O1;
//   	      sector=1050;
//   	 }
//    }
//
//    if ((EDBB>=EDBC)&&(EDBC>EDBA)){
//   	 EDB_max=EDBB;
//   	 EDB_min=EDBA;
//   	 if (EDBC>=0){ //240-270
//   		 K0=O0;
//   		 sector=1350;
//   	 }
//   	 if (EDBC<0){ //210-240
//   	      K0=O1;
//   	      sector=1200;
//   	 }
//    }
//
//    if ((EDBC>EDBB)&&(EDBB>=EDBA)){
//   	 EDB_max=EDBC;
//   	 EDB_min=EDBA;
//   	 if (EDBB>0){ //270-300
//   		 K0=O0;
//   		 sector=1500;
//   	 }
//   	 if (EDBB<=0){ //300-330
//   	      K0=O1;
//   	      sector=1650;
//   	 }
//    }
//
//    if ((EDBC>=EDBA)&&(EDBA>EDBB)){
//   	 EDB_max=EDBC;
//   	 EDB_min=EDBB;
//   	 if (EDBA>0){ //0-30
//   		 K0=O0;
//   		 sector=150;
//
//   	 }
//   	 if (EDBA<=0){ //330-360
//   	      K0=O1;
//   	      sector=1800;
//   	 }
//    }
//
//    offset_add =0;// -(((1-(2*K0)))+(K0*2.0*EDB_max)+((1-K0)*2.0*EDB_min));

 //   offset_add1 = offset_cal ( EDBA, EDBB, EDBC, O0, O1);
//    if (mod_scheme_select == 0){ //Sine PWM
//   	 offset_add1=0;
//    }
//    if (mod_scheme_select == 1){ //SVPWM
//   	 offset_add1 = offset_cal ( EDBA, EDBB, EDBC, 0.5, 0.5);
//        }
//    if (mod_scheme_select == 2){ //DPWM3
//   	 offset_add1 = offset_cal ( EDBA, EDBB, EDBC, 1, 0);
//        }
//    if (mod_scheme_select == 3){ //DPWM1
//     offset_add1 = offset_cal ( EDBA, EDBB, EDBC, 0, 1);
//        }
//    EDBA =(2*EDBA);
//    EDBB =(2*EDBB);
//    EDBC =(2*EDBC);


	//s_open_loop.s_dutyabc_flt.fltA=0.5F+((1.0*EDBA*M)*(float_t)freq/bit_max);
	//s_open_loop.s_dutyabc_flt.fltB=0.5F+((1.0*EDBB*M)*(float_t)freq/bit_max);
	//s_open_loop.s_dutyabc_flt.fltC=0.5F+((1.0*EDBC*M)*(float_t)freq/bit_max);

	s_open_loop.s_dutyabc_flt.fltA=0.5F+((EDBA));
	s_open_loop.s_dutyabc_flt.fltB=0.5F+((EDBB));
	s_open_loop.s_dutyabc_flt.fltC=0.5F+((EDBC));
//	s_open_loop.s_dutyabc_flt.fltA=0.5F+(1.154*(EDBA*M)*(float_t)freq/bit_max);
//	s_open_loop.s_dutyabc_flt.fltB=0.5F+(1.154*(EDBB*M)*(float_t)freq/bit_max);
//	s_open_loop.s_dutyabc_flt.fltC=0.5F+(1.154*(EDBC*M)*(float_t)freq/bit_max);


//     s_open_loop.s_dutyabc_flt.fltA=0.5F+((*(duty_cycle+g_phase_shift_A)-0.5F)*M)*(float_t)freq/bit_max;
//	 s_open_loop.s_dutyabc_flt.fltB=0.5F+((*(duty_cycle+g_phase_shift_B)-0.5F)*M)*(float_t)freq/bit_max;
//	 s_open_loop.s_dutyabc_flt.fltC=0.5F+((*(duty_cycle+g_phase_shift_C)-0.5F)*M)*(float_t)freq/bit_max;
	if(g_phase_shift_A>=3*LUT_TEMP_VAL-1){
		g_phase_shift_A=0;
	}
	if(g_phase_shift_B>=3*LUT_TEMP_VAL-1){
		g_phase_shift_B=0;
	}
	if(g_phase_shift_C>=3*LUT_TEMP_VAL-1){
		g_phase_shift_C=0;
	}

//	DAC_SetBufferValue(DAC0, 0U, (g_phase_shift_A*0xFFF)/(3*LUT_TEMP_VAL-1));
	//DAC_Update((uint16_t)(s_open_loop.s_dutyabc_flt.fltA*0xFFF),(uint16_t)(s_open_loop.s_dutyabc_flt.fltC*0xFFF));
//	GPIO_PinWrite(GPIOC, 9, 1);
}



/*
 * @brief Function Declaration for Decelearation case
 */
void Get_Deceleration_Duty_Cycle(void){
//	if((end_check-0)==(STOP_OK)){
//		//motor_run=2;
//		Get_Fault_Duty_Cycle();
//         get_out=1;
//   //      motor_sm_state = MOTOR_SM_READY;
//		 //exit(0);
//	}
//	if (get_out==0){
	start_check = 0;
	if (freq <= 0)
	{
		/**< Stop the motor*/
		Get_Fault_Duty_Cycle();
		get_out = 1;
		return;
	}
	freq=(freq_prev-decel_del_freq*f_freq_map);
	rem=((int)ceil(freq));
		int temp_bit_max = bit_max/(FREQ_NEEDED/LUT_FREQUENCY);
		accu=accu+rem;
	//	if(accu>temp_bit_max){
			g_phase_shift_A=(g_phase_shift_A+(accu/temp_bit_max));
			g_phase_shift_B=(g_phase_shift_B+(accu/temp_bit_max));
			g_phase_shift_C=(g_phase_shift_C+(accu/temp_bit_max));


			g_phase_shift_A = (g_phase_shift_A)%(3*LUT_TEMP_VAL);
			g_phase_shift_B = (g_phase_shift_B)%(3*LUT_TEMP_VAL);
			g_phase_shift_C = (g_phase_shift_C)%(3*LUT_TEMP_VAL);
			accu=accu%temp_bit_max;
	//	}

		//	if (mod_scheme_select==0){
						mod_sf=(float_t)freq*0.00022*M;
//					}
//					else{
//						mod_sf=1.145*(float_t)freq*0.000244*M;
//					}
		     EDBA =(*(duty_cycle+g_phase_shift_A)-0.5F)*mod_sf;
		     EDBB =(*(duty_cycle+g_phase_shift_C)-0.5F)*mod_sf;
		     EDBC =(*(duty_cycle+g_phase_shift_B)-0.5F)*mod_sf;


//		     if ((EDBA>EDBC)&&(EDBC>=EDBB)){
//		    	 EDB_max=EDBA;
//		    	 EDB_min=EDBB;
//		    	 if (EDBC>0){ //30-60
//		    		 K0=O0;
//		    		 sector=300;
//		    	 }
//		    	 if (EDBC<=0){ //60-90
//		    	      K0=O1;
//		    	      sector=450;
//		    	 }
//		     }
//
//		     if ((EDBA>=EDBB)&&(EDBB>EDBC)){
//		    	 EDB_max=EDBA;
//		    	 EDB_min=EDBC;
//		    	 if (EDBB>=0){ //120-150
//		    		 K0=O0;
//		    		 sector = 750;
//		    	 }
//		    	 if (EDBB<0){ //90-120
//		    	      K0=O1;
//		    	      sector=600;
//		    	 }
//		     }
//
//		     if ((EDBB>EDBA)&&(EDBA>=EDBC)){
//		    	 EDB_max=EDBB;
//		    	 EDB_min=EDBC;
//		    	 if (EDBA>=0){ //150-180
//		    		 K0=O0;
//		    		 sector=900;
//		    	 }
//		    	 if (EDBA<0){ //180-210
//		    	      K0=O1;
//		    	      sector=1050;
//		    	 }
//		     }
//
//		     if ((EDBB>=EDBC)&&(EDBC>EDBA)){
//		    	 EDB_max=EDBB;
//		    	 EDB_min=EDBA;
//		    	 if (EDBC>=0){ //240-270
//		    		 K0=O0;
//		    		 sector=1350;
//		    	 }
//		    	 if (EDBC<0){ //210-240
//		    	      K0=O1;
//		    	      sector=1200;
//		    	 }
//		     }
//
//		     if ((EDBC>EDBB)&&(EDBB>=EDBA)){
//		    	 EDB_max=EDBC;
//		    	 EDB_min=EDBA;
//		    	 if (EDBB>0){ //270-300
//		    		 K0=O0;
//		    		 sector=1500;
//		    	 }
//		    	 if (EDBB<=0){ //300-330
//		    	      K0=O1;
//		    	      sector=1650;
//		    	 }
//		     }
//
//		     if ((EDBC>=EDBA)&&(EDBA>EDBB)){
//		    	 EDB_max=EDBC;
//		    	 EDB_min=EDBB;
//		    	 if (EDBA>0){ //0-30
//		    		 K0=O0;
//		    		 sector=150;
//
//		    	 }
//		    	 if (EDBA<=0){ //330-360
//		    	      K0=O1;
//		    	      sector=1800;
//		    	 }
//		     }
//
//		     offset_add =0;// -(((1-(2*K0)))+(K0*2.0*EDB_max)+((1-K0)*2.0*EDB_min));

//		     offset_add1 = offset_cal ( EDBA, EDBB, EDBC, O0, O1);
//		     if (mod_scheme_select == 0){ //Sine PWM
//		    	 offset_add1=0;
//		     }
//		     if (mod_scheme_select == 1){ //SVPWM
//		    	 offset_add1 = offset_cal ( EDBA, EDBB, EDBC, 0.5, 0.5);
//		         }
//		     if (mod_scheme_select == 2){ //DPWM3
//		    	 offset_add1 = offset_cal ( EDBA, EDBB, EDBC, 1, 0);
//		         }
//		     if (mod_scheme_select == 3){ //DPWM1
//		         	 offset_add1 = offset_cal ( EDBA, EDBB, EDBC, 0, 1);
//		              }
//		     EDBA =(2*EDBA) ;
//		     EDBB =(2*EDBB);
//		     EDBC =(2*EDBC);


			//s_open_loop.s_dutyabc_flt.fltA=0.5F+((1.0*EDBA*M)*(float_t)freq/bit_max);
			//s_open_loop.s_dutyabc_flt.fltB=0.5F+((1.0*EDBB*M)*(float_t)freq/bit_max);
			//s_open_loop.s_dutyabc_flt.fltC=0.5F+((1.0*EDBC*M)*(float_t)freq/bit_max);

			    s_open_loop.s_dutyabc_flt.fltA=0.5F+((EDBA));
				s_open_loop.s_dutyabc_flt.fltB=0.5F+((EDBB));
				s_open_loop.s_dutyabc_flt.fltC=0.5F+((EDBC));


	// s_open_loop.s_dutyabc_flt.fltA=0.5F+((*(duty_cycle+g_phase_shift_A)-0.5F)*M)*(float_t)freq/bit_max;
	//s_open_loop.s_dutyabc_flt.fltB=0.5F+((*(duty_cycle+g_phase_shift_B)-0.5F)*M)*(float_t)freq/bit_max;
	// s_open_loop.s_dutyabc_flt.fltC=0.5F+((*(duty_cycle+g_phase_shift_C)-0.5F)*M)*(float_t)freq/bit_max;
	end_check++;
	freq_prev=freq;
	if(g_phase_shift_A>3*LUT_TEMP_VAL-1){
		g_phase_shift_A=0;
	}
	if(g_phase_shift_B>3*LUT_TEMP_VAL-1){
		g_phase_shift_B=0;
	}
	if(g_phase_shift_C>3*LUT_TEMP_VAL-1){
		g_phase_shift_C=0;
	}
// DAC_Update((uint16_t)(s_open_loop.s_dutyabc_flt.fltA*0xFFF),(uint16_t)(s_open_loop.s_dutyabc_flt.fltC*0xFFF));
}
//}


/*
 * @brief Function definition to allocate the variables to corresponding structures
 * @param A global pointer variable
 * @return None
 * */
void DrivePOC_MH_GetVIvalues(void)
{
	//GPIO_PinWrite(GPIOB, 11, 0);
	ads_spi_transfer_block(0);
	//GPIO_PinWrite(GPIOB, 11, 1);

//	PRINTF("\n I-Phase-A=%x,I-Phase-B=%x,I-Phase-C=%x,g_Idc=%x,V-Phase-A=%x,V-Phase-B=%x,V-Phase-C=%x,g_Vdc=%x\n",g_Ia,g_Ib,g_Ic,g_Idc,g_Va,g_Vb,g_Vc,g_Vdc);
	//delay(1);
}


/*
 * @brief Function scales the value of Current using the Sensor scaling factors
 * @param None
 * @return None
 * */
void DrivePOC_MH_UpdateVIvalues(void)
{

//		s_open_loop.s_iabc.fltA=((float_t)g_Ia*CURRENT_SENSOR_SCALE_FACTOR_PHASE_A)*ADS_ADC_RANGE/32767;
//
//		s_open_loop.s_iabc.fltB=((float_t)g_Ib*CURRENT_SENSOR_SCALE_FACTOR_PHASE_B)*ADS_ADC_RANGE/32767;
//
//		s_open_loop.s_iabc.fltC=((float_t)g_Ic*CURRENT_SENSOR_SCALE_FACTOR_PHASE_C)*ADS_ADC_RANGE/32767;
//
//		s_open_loop.fltidcbus =((float_t)g_Idc*CURRENT_SENSOR_SCALE_FACTOR_DC)*ADS_ADC_RANGE/32767;
//
//		s_open_loop.s_vabc.fltA=((float_t)g_Va*VOLTAGE_SENSOR_SCALE_FACTOR_PHASE_A)*ADS_ADC_RANGE/32767;
//
//		s_open_loop.s_vabc.fltB=((float_t)g_Vb*VOLTAGE_SENSOR_SCALE_FACTOR_PHASE_B)*ADS_ADC_RANGE/32767;
//
//		s_open_loop.s_vabc.fltC=((float_t)g_Vc*VOLTAGE_SENSOR_SCALE_FACTOR_PHASE_C)*ADS_ADC_RANGE/32767;
//
//		s_open_loop.fltudcbus =((float_t)g_Vdc*VOLTAGE_SENSOR_SCALE_FACTOR_DC)*ADS_ADC_RANGE/32767;

	//PRINTF("\nI-Phase-A = %f\n",s_foc_acimvar.s_iabc.fltA);
//	PRINTF("\n I-Phase-A=%f,I-Phase-B=%f,I-Phase-C=%f,g_Idc=%f,V-Phase-A=%f,V-Phase-B=%f,V-Phase-C=%f,g_Vdc=%f\n",s_open_loop.s_iabc.fltA,s_open_loop.s_iabc.fltB,s_open_loop.s_iabc.fltC,s_open_loop.fltidcbus,s_open_loop.s_vabc.fltA,s_open_loop.s_vabc.fltB,s_open_loop.s_vabc.fltC,s_open_loop.fltudcbus);
}


/*
 * @brief Function Declaration to get Start UP done signal
 */
bool Get_Start_Up_status(void){
	return start_ok;
}


/*
 * @brief Function Declaration to get Stop done signal
 */
bool Get_Stop_status(void){
	return stop_ok;
}


bool is_motor_stopped()
{
	return get_out;
}
