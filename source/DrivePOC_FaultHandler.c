/**
 ******************************************************************************
 * @file           : DrivePOC_FaultHandler.c
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



/*******************************************************************************
 * Header inclusions
 ******************************************************************************/
#include "DrivePOC_FaultHandler.h"
#include "DrivePOC_CommHandler.h"
#include "DrivePOC_MemHandler.h"
/*******************************************************************************
 * Function definitions
 ******************************************************************************/
/*
 * @brief Function definition that will make the Duty Cycle of all the switches to zero
 * @param None
 * @return None
 * */
float_t offset_calc, svdpwm;

void Get_Fault_Duty_Cycle(void){
	s_open_loop.s_dutyabc_flt.fltA=0.0f;
	s_open_loop.s_dutyabc_flt.fltB=0.0f;
	s_open_loop.s_dutyabc_flt.fltC=0.0f;
//	M1_MCDRV_PWM3PH_DIS(&g_sM1Pwm3ph);
}
//float_t offset_cal(float_t msa, float_t msb, float_t msc, float_t KO0, float_t KO1){
//			     if ((msa>msc)&&(msc>=msb)){
//			    	 EDB_max=msa;
//			    	 EDB_min=msb;
//			    	 if (msc>0){ //30-60
//			    		 K0=KO0;
//			    		 sector=300;
//			    	 }
//			    	 if (msc<=0){ //60-90
//			    	      K0=KO1;
//			    	      sector=450;
//			    	 }
//			     }
//
//			     if ((msa>=msb)&&(msb>msc)){
//			    	 EDB_max=msa;
//			    	 EDB_min=msc;
//			    	 if (msb>=0){ //120-150
//			    		 K0=KO0;
//			    		 sector = 750;
//			    	 }
//			    	 if (msb<0){ //90-120
//			    	      K0=KO1;
//			    	      sector=600;
//			    	 }
//			     }
//
//			     if ((msb>msa)&&(msa>=msc)){
//			    	 EDB_max=msb;
//			    	 EDB_min=msc;
//			    	 if (msa>=0){ //150-180
//			    		 K0=KO0;
//			    		 sector=900;
//			    	 }
//			    	 if (msa<0){ //180-210
//			    	      K0=KO1;
//			    	      sector=1050;
//			    	 }
//			     }
//
//			     if ((msb>=msc)&&(msc>msa)){
//			    	 EDB_max=msb;
//			    	 EDB_min=msa;
//			    	 if (msc>=0){ //240-270
//			    		 K0=KO0;
//			    		 sector=1350;
//			    	 }
//			    	 if (msc<0){ //210-240
//			    	      K0=KO1;
//			    	      sector=1200;
//			    	 }
//			     }
//
//			     if ((msc>msb)&&(msb>=msa)){
//			    	 EDB_max=msc;
//			    	 EDB_min=msa;
//			    	 if (msb>0){ //270-300
//			    		 K0=KO0;
//			    		 sector=1500;
//			    	 }
//			    	 if (msb<=0){ //300-330
//			    	      K0=KO1;
//			    	      sector=1650;
//			    	 }
//			     }
//
//			     if ((msc>=msa)&&(msa>msb)){
//			    	 EDB_max=msc;
//			    	 EDB_min=msb;
//			    	 if (msa>0){ //0-30
//			    		 K0=KO0;
//			    		 sector=150;
//
//			    	 }
//			    	 if (msa<=0){ //330-360
//			    	      K0=KO1;
//			    	      sector=1800;
//			    	 }
//			     }
//
//			     offset_calc = -(((1-(2*K0)))+(K0*2.0*EDB_max)+((1-K0)*2.0*EDB_min));
//        svdpwm=offset_calc;
//	return svdpwm;
//}
//

//float_t romesq_cal (float i_ph_a, float i_ph_b, float i_ph_c){
////	        i_ph_alpha = ((i_ph_a)-(0.5*(i_ph_b+i_ph_c)));
//         	i_ph_alpha = (1.5*(i_ph_a));
//			i_ph_beta = 0.866*(i_ph_b-i_ph_c);
//			i_ph_rms = 0.0286*sqrt((i_ph_alpha*i_ph_alpha)+(i_ph_beta*i_ph_beta)); // 0.471 = (2/3)*sqrt(1/2), where 2/3 is gain from abc to alpha-beta, sqrt(1/2) is conversion from amplitude to rms.
//// i_ph_rms 0.005 scaling factor came from, 1. digital value * 0.471, 2. (1)*(10/32768), 3.(2)/40.2=Rm, 4. (3)*2000`
//}
