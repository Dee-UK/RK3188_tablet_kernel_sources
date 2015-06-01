#ifndef __LCD_PIPO_MIPI_U8__
#define __LCD_PIPO_MIPI_U8__

#if defined(CONFIG_MIPI_DSI)
#include "../transmitter/mipi_dsi.h"
#endif

#include <linux/delay.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/io.h>

#if  defined(CONFIG_RK610_LVDS) || defined(CONFIG_RK616_LVDS)
#define SCREEN_TYPE	    	SCREEN_LVDS
#else
#define SCREEN_TYPE	    	SCREEN_MIPI
#endif
#define LVDS_FORMAT         0     //mipi lcd don't need it, so 0 would be ok.
#define OUT_FACE	    	OUT_P666  //OUT_P888


#define DCLK	          	698*100000 
#define LCDC_ACLK         	500000000           //29 lcdc axi DMA ?¦Ì?¨º

/* Timing */
#define H_PW			64  
#define H_BP			56 
#define H_VD			768 
#define H_FP			60 

#define V_PW			14
#define V_BP			30 
#define V_VD			1024 
#define V_FP			36 

#define LCD_WIDTH         119  
#define LCD_HEIGHT         159

/* Other */
#if defined(CONFIG_RK610_LVDS) || defined(CONFIG_RK616_LVDS) || defined(CONFIG_MIPI_DSI)
#define DCLK_POL	1
#else
#define DCLK_POL	0
#endif
#define DEN_POL		0
#define VSYNC_POL	1
#define HSYNC_POL	1

#define SWAP_RB		0
#define SWAP_RG		0
#define SWAP_GB		0

#define RK_SCREEN_INIT 	1

/* about mipi */
#define MIPI_DSI_LANE 			4
#define MIPI_DSI_HS_CLK 		1000*1000000

#if defined(RK_SCREEN_INIT)
static struct rk29lcd_info *gLcd_info = NULL;

//temporary workarounds added by D33
#define dcs_enter_sleep_mode  		0x10
#define dcs_exit_sleep_mode  		0x11
#define dcs_set_display_off  		0x28
#define dcs_set_display_on  		0x29

#define MIPI_RST_PIN 			RK30_PIN0_PD4

int gTmp = 1;
int rk_lcd_init(void) {

	int ret = 0;
	int dsi_active = 0;

	u8 dcs[16] = {0};
	dsi_active = dsi_is_active();
	if (dsi_active != 1)
	{
		printk("mipi reset pin error\n");
		return dsi_active;
	}
	/*below is changeable*/

	if(gTmp == 1)
	{
		ret = gpio_request(MIPI_RST_PIN, "mipi reset pin");
		if( ret != 0 )
		{
			gpio_free(MIPI_RST_PIN);
			printk("mipi reset pin error\n");
			//return -EIO; commented out by D33
		} else
		{
			gpio_set_value(MIPI_RST_PIN, !GPIO_LOW);
			msleep(10);
			gpio_set_value(MIPI_RST_PIN, GPIO_LOW);
			msleep(10);
	
			gpio_set_value(MIPI_RST_PIN, GPIO_HIGH);
			msleep(20);
	
			dsi_enable_hs_clk(1);

			dcs[0] = LPDT;
			dcs[1] = dcs_exit_sleep_mode; 
			dsi_send_dcs_packet(dcs, 2);
			msleep(1);
			dcs[0] = LPDT;
			dcs[1] = dcs_set_display_on;
			dsi_send_dcs_packet(dcs, 2);
			msleep(10);
   			//dsi_enable_command_mode(0);
			dsi_enable_video_mode(1);
		
			printk("++++++++++++++++%s:%d\n", __func__, __LINE__);
		}
		gTmp++;		
	}

}

int rk_lcd_standby(u8 enable) {

	int dsi_active;
	u8 dcs[16] = {0};
	dsi_active = dsi_is_active();
	if (dsi_active != 1)
	{
		printk("mipi reset pin error\n");
		return dsi_active;
	}
	
	if(enable) {
		//dsi_enable_video_mode(0);
		/*below is changeable*/
		dcs[0] = LPDT;
		dcs[1] = dcs_set_display_off; 
		dsi_send_dcs_packet(dcs, 2);
		msleep(1);
		dcs[0] = LPDT;
		dcs[1] = dcs_enter_sleep_mode; 
		dsi_send_dcs_packet(dcs, 2);
		msleep(1);

		printk("++++enable++++++++++++%s:%d\n", __func__, __LINE__);
	
	} else {
		/*below is changeable*/
		rk_lcd_init();
		printk("++++++++++++++++%s:%d\n", __func__, __LINE__);	
	}
}
int lcd_io_init(void)
{
	int ret = 0;

	if(!gLcd_info)
		return -1;

	ret = gpio_request(gLcd_info->reset_pin, NULL);
	if (ret != 0) {
		gpio_free(gLcd_info->reset_pin);
		printk("%s: request LCD_RST_PIN error\n", __func__);
	} else
	{
		gpio_direction_output(gLcd_info->reset_pin, !GPIO_LOW);
	}
	
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
#endif //RK_SCREEN_INT
#endif  
