#ifndef __LCD_LD089WU1__
#define __LCD_LD089WU1__

#if defined(CONFIG_MIPI_DSI)
#include "../transmitter/mipi_dsi.h"
#endif

#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/io.h>

#if  defined(CONFIG_RK610_LVDS) || defined(CONFIG_RK616_LVDS)
#define SCREEN_TYPE	    	SCREEN_LVDS
#else
#define SCREEN_TYPE	    	SCREEN_RGB
#endif
#define LVDS_FORMAT         0     //mipi lcd don't need it, so 0 would be ok.
#define OUT_FACE	    	OUT_P888  //OUT_P888


#define DCLK	          	128000000 // from stock dmesg - //150*1000000
#define LCDC_ACLK         	530000000           //29 lcdc axi DMA ?¦Ì?¨º 

/* Timing */
#define H_PW			16
#define H_BP			40
#define H_VD			1920
#define H_FP			24

#define V_PW			10
#define V_BP			10
#define V_VD			1200
#define V_FP			16

#define LCD_WIDTH       203    //uint mm the lenth of lcd active area
#define LCD_HEIGHT      136

/* Other */
#if defined(CONFIG_RK610_LVDS) || defined(CONFIG_RK616_LVDS) || defined(CONFIG_MIPI_DSI)
#define DCLK_POL	1
#else
#define DCLK_POL	0
#endif
#define DEN_POL		0
#define VSYNC_POL	0
#define HSYNC_POL	0

#define SWAP_RB		0
#define SWAP_RG		0
#define SWAP_GB		0

#define RK_SCREEN_INIT 	1

#define mipi_dsi_init(data) 				dsi_set_regs(data, ARRAY_SIZE(data))
#define mipi_dsi_send_dcs_packet(data) 		dsi_send_dcs_packet(data, ARRAY_SIZE(data))
#define mipi_dsi_post_init(data)			dsi_set_regs(data, ARRAY_SIZE(data))
#define data_lane  4

static struct rk29lcd_info *gLcd_info = NULL;
int rk_lcd_init(void);
int rk_lcd_standby(u8 enable);

static unsigned int pre_initialize[] = {
	
	0x00B10000 | ((V_PW & 0Xff) << 8) | (H_PW & 0Xff),
	0x00B20000 | (((V_BP+V_PW) & 0Xff) << 8) | ((H_BP+H_PW) & 0Xff),
	0x00B30000 | ((V_FP & 0Xff) << 8) | (H_FP & 0Xff),
	0x00B40000 | H_VD,
	0x00B50000 | V_VD,
	0x00B60000 | (VPF_24BPP) | (VM_BM << 2),

		//Add by Lincp
	0x00c91e0a,
	0x00ca390f,
	0x00cb022F,
	0x00cc0f18,
	0x00d9ffa0, 
	0x00de0000 | (data_lane -1),    //4 lanes
	0x00d60004,	
	0x00B90000,
	0x00bac027,   //pll    //960m 0x00bac026
	0x00Bb000a,
	0x00B90001,	
	0x00c40001,
	};

static unsigned int post_initialize[] = {
	0x00B90000,
	0x00B7034b,
	0x00B90001,
	0x00B80000,
	0x00BC0000,
	0x00c00100,      //software reset ssd2828
};

static unsigned char dcs_exit_sleep_mode[] = {0x11};
static unsigned char dcs_set_display_on[] = {0x29};
static unsigned char dcs_enter_sleep_mode[] = {0x10};
static unsigned char dcs_set_display_off[] = {0x28};

int lcd_io_init(void)
{
	int ret = 0;
	if(!gLcd_info)
		return -1;
	ret = gpio_request(gLcd_info->reset_pin, NULL);
	if (ret != 0) {
		gpio_free(gLcd_info->reset_pin);
		printk("%s: request LCD_RST_PIN error\n", __func__);
		return -EIO;
	}
	
	gpio_direction_output(gLcd_info->reset_pin, !GPIO_LOW);
	return ret;
}

int lcd_io_deinit(void)
{
	int ret = 0;
	if(!gLcd_info)
		return -1;
	gpio_direction_input(gLcd_info->reset_pin);
	gpio_free(gLcd_info->reset_pin);
	return ret;
}

int lcd_reset(void) {
	int ret = 0;
	if(!gLcd_info)
		return -1;
	gpio_set_value(gLcd_info->reset_pin, GPIO_LOW);
	msleep(10);
	gpio_set_value(gLcd_info->reset_pin, !GPIO_LOW);
	msleep(2);
	return ret;
}

int rk_lcd_init(void)
{
	lcd_reset();	
	msleep(10);
   	mipi_dsi_init(pre_initialize);
	mipi_dsi_send_dcs_packet(dcs_exit_sleep_mode);
	msleep(10);
	mipi_dsi_send_dcs_packet(dcs_set_display_on);
	msleep(10);
	mipi_dsi_post_init(post_initialize);   

    return 0;

}

int rk_lcd_standby(u8 enable)
{
	if(enable) {

		printk("rk_lcd_standby...\n");
		mipi_dsi_send_dcs_packet(dcs_set_display_off);
		msleep(2);		
		mipi_dsi_send_dcs_packet(dcs_enter_sleep_mode);
		msleep(100);
		dsi_power_off();
		if(gLcd_info == NULL)
		{
			//printk("rk_lcd_standby... line = %d, gLcd_info = NULL\n", __LINE__);
		}
		//		printk("rk_lcd_standby...  line = %d\n", __LINE__);
		gpio_set_value(RK30_PIN0_PB0, 0);
		gpio_set_value(RK30_PIN0_PA7, 0);
				//printk("rk_lcd_standby...  line = %d , 3_D4 = %d ,0_A7 = %d\n", __LINE__,
				//	gpio_get_value(RK30_PIN3_PD4),gpio_get_value(RK30_PIN0_PA7));
	} else {
		dsi_power_up();
		rk_lcd_init();
	}

    return 0;
}

#endif  
