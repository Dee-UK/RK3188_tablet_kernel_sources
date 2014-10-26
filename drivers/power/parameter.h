/*****************************************************************************
* Copyright(c) O2Micro, 2013. All rights reserved.
*	
* O2Micro OZ8806 battery gauge driver
* File: parameter.h

* Author: Eason.yuan
* $Source: /data/code/CVS
* $Revision: 4.00.01 $
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

#ifndef _PARAMETER_H_
#define _PARAMETER_H_


/****************************************************************************
* #include section
*  add #include here if any
***************************************************************************/

/****************************************************************************
* #define section
*  add constant #define here if any
***************************************************************************/
#define MUTEX_TIMEOUT           5000
#define MYDRIVER				"oz8806"  //"OZ8806"
#define OZ8806Addr				0x2F

#define DISCH_CURRENT_TH    -10

#define OCV_DATA_NUM  51
#define CHARGE_DATA_NUM 51
#define TEMPERATURE_DATA_NUM 28

#define	num_0      	0
#define num_1		1		
#define num_32768  	32768
#define num_5		5
#define num_0x2c	0x2c
#define num_0x40    0x40
#define num_0x20    0x20
#define num_1000    1000
#define num_50      50
#define num_100     100
#define num_10      10
#define num_7       7
#define num_2       2
#define num_1000    1000
#define num_0x2f    0x2f
#define num_0x28    0x28
#define num_0x07	0x07
#define num_6		6
#define num_10		10
#define num_20		20
#define num_9		9
#define num_95		95
#define num_0x4c	0x4c
#define num_0x40	0x40
#define num_3		3
#define num_99		99
#define num_15		15
#define num_25		25
#define num_0x80	0x80

#define XAxis		20
#define YAxis		3
#define ZAxis		3


//#define PEC_CHECK
 
/****************************************************************************
* Struct section
*  add struct #define here if any
***************************************************************************/
typedef struct tag_one_latitude_data{
	int32_t			x;//
	int32_t			y;//				
}one_latitude_data_t;


//config struct
typedef struct	 tag_config_data {
	int32_t		fRsense;		//= 20;			//Rsense value of chip, in mini ohm
	int32_t     temp_pull_up;  //230000;
	int32_t     temp_ref_voltage; //1800;1.8v
	int32_t		dbCARLSB;		//= 5.0;		//LSB of CAR, comes from spec
	int32_t		dbCurrLSB;		//781 (3.90625*100);	//LSB of Current, comes from spec
	int32_t		fVoltLSB;		//250 (2.5*100);	//LSB of Voltage, comes from spec

	int32_t		design_capacity;	//= 7000;		//design capacity of the battery
 	int32_t		charge_cv_voltage;	//= 4200;		//CV Voltage at fully charged
	int32_t		charge_end_current;	//= 100;		//the current threshold of End of Charged
	int32_t		discharge_end_voltage;	//= 3550;		//mV
	int32_t     board_offset;			//0; 				//mA, not more than caculate data
	uint8_t         debug;                                          // enable or disable O2MICRO debug information

}config_data_t;


typedef struct	 tag_parameter_data {
	int32_t            	 	ocv_data_num;
	int32_t              	cell_temp_num;
	one_latitude_data_t  	*ocv;
	one_latitude_data_t	 	*temperature;
	config_data_t 		 	*config;
		
	struct i2c_client 	 	*client;
	
	uint8_t 	  			charge_pursue_step;		
 	uint8_t  				discharge_pursue_step;		
	uint8_t  				discharge_pursue_th;	
	uint8_t             	wait_method;

}parameter_data_t;


typedef struct tag_bmu {
	int32_t		PowerStatus;	
	int32_t		fRC;			//= 0;		//Remaining Capacity, indicates how many mAhr in battery
	int32_t		fRSOC;			//50 = 50%;	//Relative State Of Charged, present percentage of battery capacity
	int32_t		fVolt;			//= 0;						//Voltage of battery, in mV
	int32_t		fCurr;			//= 0;		//Current of battery, in mA; plus value means charging, minus value means discharging
	int32_t		fPrevCurr;		//= 0;						//last one current reading
	int32_t		fOCVVolt;		//= 0;						//Open Circuit Voltage
	int32_t		fCellTemp;		//= 0;						//Temperature of battery
	int32_t	    fRCPrev;
	int32_t		sCaMAH;			//= 0;						//adjusted residual capacity				
	int32_t     i2c_error_times;
}bmu_data_t;

typedef struct tag_gas_gauge {
	
	int32_t  overflow_data;
	uint8_t  discharge_end;
 	uint8_t  charge_end;
	uint8_t  charge_fcc_update;
 	uint8_t  discharge_fcc_update;

	int32_t  sCtMAH ;    //becarfull this must int32_t
	int32_t  fcc_data;
	int32_t  discharge_sCtMAH ;//becarfull this must int32_t
	uint8_t  charge_wait_times;
	uint8_t  discharge_wait_times;
	uint8_t  charge_count;
	uint8_t  discharge_count;
	uint32_t bmu_tick;
	uint32_t charge_tick;

	uint8_t charge_table_num;
	uint8_t rc_x_num;
	uint8_t rc_y_num;
	uint8_t rc_z_num;

    uint8_t  charge_strategy; 
	int32_t  charge_sCaUAH;
	int32_t  charge_ratio;  //this must be static 
 	uint8_t  charge_table_flag;
	int32_t  charge_end_current_th2;
	uint16_t charge_max_ratio;

	uint8_t discharge_strategy;
	int32_t discharge_sCaUAH;
	int32_t discharge_ratio;  //this must be static 
	uint8_t discharge_table_flag;
	int32_t	discharge_current_th;

	int32_t dsg_end_voltage_hi;
	int32_t dsg_end_voltage_th1;
	int32_t dsg_end_voltage_th2;
	uint8_t dsg_count_2;

	uint8_t ocv_flag;
}gas_gauge_t;

/****************************************************************************
* extern variable declaration section
***************************************************************************/
extern bmu_data_t *batt_info;
extern parameter_data_t parameter_customer;
extern unsigned long kernel_memaddr;

extern int	XAxisElement[XAxis];
extern int  YAxisElement[YAxis];
extern int	ZAxisElement[ZAxis];
extern int	RCtable[YAxis*ZAxis][XAxis];
extern one_latitude_data_t	charge_data[CHARGE_DATA_NUM];
extern uint8_t charger_finish;


/****************************************************************************
*  section
*  add function prototype here if any
***************************************************************************/
extern void bmu_init_chip(parameter_data_t *paramter_data);
extern void bmu_wake_up_chip(void);
extern void bmu_power_down_chip(void);
extern void bmu_polling_loop(void);
extern void bmu_init_parameter(struct i2c_client *client);

#endif














