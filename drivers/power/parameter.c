/*****************************************************************************
* Copyright(c) O2Micro, 2013. All rights reserved.
*	
* O2Micro OZ8806 battery gauge driver
* File: parameter.c

* Author: Eason.yuan
* $Source: /data/code/CVS
* $Revision: 4.00.00 $
*
* This program is free software and can be edistributed and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*	
* This Source Code Reference Design for O2MICRO OZ8806 access (\u201cReference Design\u201d) 
* is sole for the use of PRODUCT INTEGRATION REFERENCE ONLY, and contains confidential 
* and privileged information of O2Micro International Limited. O2Micro shall have no 
* liability to any PARTY FOR THE RELIABILITY, SERVICEABILITY FOR THE RESULT OF PRODUCT 
* INTEGRATION, or results from: (i) any modification or attempted modification of the 
* Reference Design by any party, or (ii) the combination, operation or use of the 
* Reference Design with non-O2Micro Reference Design.
*****************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include "parameter.h"

/*****************************************************************************
* Define section
* add all #define here
*****************************************************************************/
//#define OCV_DATA_NUM  11


/****************************************************************************
* extern variable declaration section
****************************************************************************/
extern int32_t res_divider_ratio;
extern uint8_t oz8806_cell_num;

/*****************************************************************************
* Global variables section - Exported
* add declaration of global variables that will be exported here
* e.g.
*	int8_t foo;
****************************************************************************/

one_latitude_data_t ocv_data[OCV_DATA_NUM] = {
	{3570, 00},{3580, 2},{3592, 4},{3605, 6},{3607, 8},{3617, 10},{3625, 12},
	{3632, 14},{3640, 16},{3647, 18},{3650, 20},{3653, 22},{3657, 24},{3662, 26},
	{3664, 28},{3665, 30},{3672, 32},{3680, 34},{3682, 36},{3687, 38},{3695, 40},
	{3702, 42},{3705, 44},{3715, 46},{3722, 48},{3727, 50},{3735, 52},{3740, 54},
	{3755, 56},{3765, 58},{3777, 60},{3785, 62},{3800, 64},{3815, 66},{3827, 68},
	{3840, 70},{3850, 72},{3862, 74},{3882, 76},{3900, 78},{3915, 80},{3932, 82},
	{3945, 84},{3955, 86},{3977, 88},{3992, 90},{4012, 92},{4030, 94},{4040, 96},
	{4045, 98},{4050, 100},
};

// changle
one_latitude_data_t	charge_data[CHARGE_DATA_NUM] = {                                                                                                               
	{200, 10000},{225, 9904},{250, 9747},{275, 9314},{300, 9387},{325, 9260},{350,9233},
	{375,9205},{400, 9179},{426, 9154},{450, 9127},{475, 9099},{501, 9073},{525, 9046},
	{550, 9020},{575, 8993},{600, 89661},{625, 8939},{650, 8913},{675, 8886},{701, 8858},
	{725, 8833},{750, 8803},{775, 8780},{806, 8748},{836, 8715},{870, 8680},{905, 8643},
	{935, 8613},{971, 8577},{1005, 8541},{1035, 8509},{1070, 8473},{1105, 8437},{1136, 8404},
	{1170, 8368},{1205, 8332},{1235, 8302},{1271, 8263},{1306, 8228},{1336, 8197},{1370, 8166},
	{1405, 8128},{1436, 8096},{1470, 8061},{1505, 8024},{1536, 7992},{1570, 7956},{1605, 7921},
	{1635, 7890},{1670, 7855},
};


one_latitude_data_t			cell_temp_data[TEMPERATURE_DATA_NUM] = {                                                                                                               
			{681,   115}, {766,   113}, {865,   105},
			{980,   100}, {1113,   95}, {1266,   90},
			{1451,   85}, {1668,   80}, {1924,   75},
			{2228,   70}, {2588,   65}, {3020,   60},
			{3536,   55}, {4160,   50}, {4911,   45},
			{5827,   40}, {6940,   35}, {8313,   30},
			{10000,  25}, {12090,  20}, {14690,  15},
			{17960,  10}, {22050,   5},	{27280,   0},
			{33900,  -5}, {42470, -10}, {53410, -15},
			{67770, -20},
};


//config_data_t config_data = {20,232000,3300,5,781,250,8000,4150,200,3500,0,0};
config_data_t config_data = {20,232000,3300,5,781,250,8000,4150,200,3500,0,0};
//
//config_data_t config_data = {22,232000,3300,5,781,250,6858,2100,100,1750,0,1};

/*
	int32_t		fRsense;		//= 20;			//Rsense value of chip, in mini ohm
	int32_t     temp_pull_up;  //230000;
	int32_t     temp_ref_voltage; //3300;3.3v
	int32_t		dbCARLSB;		//= 5.0;		//LSB of CAR, comes from spec
	int32_t		dbCurrLSB;		//781 (3.90625*100);	//LSB of Current, comes from spec
	int32_t		fVoltLSB;		//250 (2.5*100);	//LSB of Voltage, comes from spec

	int32_t		design_capacity;	//= 8000;		//design capacity of the battery
 	int32_t		charge_cv_voltage;	//= 4200;		//CV Voltage at fully charged
	int32_t		charge_end_current;	//= 100;		//the current threshold of End of Charged
	int32_t		discharge_end_voltage;	//= 3500;		//mV
	int32_t     board_offset;			//0; 				//mA, not more than caculate data
	uint8_t     debug;                                          // enable or disable O2MICRO debug information
}
*/



//  
int	XAxisElement[XAxis] = {3400,3435,3475,3500,3540,3565,3595,3620,3645,3665,3685,3705,3735,3775,3805,3830,3855,3905,3945,4100};	
 
int	YAxisElement[YAxis] = { 935,1275,1700};	//10000C (1C=DesignCapacity)

// RC table Z Axis value, in 10*'C format
int	ZAxisElement[ZAxis] = { -200,250,1150};

// contents of RC table, its unit is 10000C, 1C = DesignCapacity
int	RCtable[YAxis*ZAxis][XAxis]={


//temp = 10 ^C

{88,143,224,283,406,531,925,1563,1937,2438,3279,4077,4974,5817,6274,6640,7023,7886,8273,9976     },
{280,504,659,743,953,1289,1971,2381,3117,3970,4649,5236,5918,6604,7038,7414,7834,8494,8871,9999  },
{420,643,824,979,1549,2077,2729,3674,4590,5156,5637,6060,6567,7168,7621,8059,8375,8853,9366,9998 },

//temp = 25 ^C

{88,143,224,283,406,531,925,1563,1937,2438,3279,4077,4974,5817,6274,6640,7023,7886,8273,9976     },
{280,504,659,743,953,1289,1971,2381,3117,3970,4649,5236,5918,6604,7038,7414,7834,8494,8871,9999  },
{420,643,824,979,1549,2077,2729,3674,4590,5156,5637,6060,6567,7168,7621,8059,8375,8853,9366,9998 },


//temp = 35 ^C

{88,143,224,283,406,531,925,1563,1937,2438,3279,4077,4974,5817,6274,6640,7023,7886,8273,9976     },
{280,504,659,743,953,1289,1971,2381,3117,3970,4649,5236,5918,6604,7038,7414,7834,8494,8871,9999  },
{420,643,824,979,1549,2077,2729,3674,4590,5156,5637,6060,6567,7168,7621,8059,8375,8853,9366,9998 },

};			


parameter_data_t parameter_customer;

/*****************************************************************************
 * Description:
 *		bmu_init_chip
 * Parameters:
 *		description for each argument, new argument starts at new line
 * Return:
 *		what does this function returned?
 *****************************************************************************/
void bmu_init_parameter(struct i2c_client *client)
{
	parameter_customer.config = &config_data;
	parameter_customer.ocv = &ocv_data;
	parameter_customer.temperature = &cell_temp_data;
	parameter_customer.client = client;
	parameter_customer.ocv_data_num = OCV_DATA_NUM;
	parameter_customer.cell_temp_num = TEMPERATURE_DATA_NUM;
	parameter_customer.charge_pursue_step = 10;		
 	parameter_customer.discharge_pursue_step = 6;		
	parameter_customer.discharge_pursue_th = 10;
	parameter_customer.wait_method = 2;

	//oz8806_cell_num = 2;
	//res_divider_ratio = 353 ;  // note: multiplied by 1000 

	//r1 = 220k,r2 = 120k,so 120 * 1000 / 120 + 220 = 353
	//r2's voltage is the voltage which oz8806 sample.
	
	//For example :
	//Read oz8806 voltage is vin
	//then the whole voltage is  vin * 1000 / res_divider_ratio;
}










