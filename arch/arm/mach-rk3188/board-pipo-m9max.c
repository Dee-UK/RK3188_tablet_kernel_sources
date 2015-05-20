/*
 *
 * Copyright (C) 2012 ROCKCHIP, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/mmc/host.h>
#include <linux/ion.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <mach/dvfs.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/hardware/gic.h>

#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <linux/rk_fb.h>
#include <linux/regulator/machine.h>
#include <linux/rfkill-rk.h>
#include <linux/sensor-dev.h>
#include <linux/regulator/act8846.h>

#include <plat/efuse.h>
#include <linux/regulator/rk29-pwm-regulator.h>
#include <plat/ddr.h>
#include <linux/ct36x.h>
#include <linux/power/cw2015_battery.h>
#include "../../../drivers/video/rockchip/hdmi/rk_hdmi.h"
#include "../../../drivers/spi/rk29_spim.h"
#include <linux/l3g4200d.h>
#include "../../../drivers/misc/gps/rk_gps/rk_gps.h"
//#include <linux/gps.h>
#include "../mach-rk30/board-rk3168-ds1006h-camera.c"
#include <plat/key.h>
#include "../../../drivers/video/rockchip/transmitter/mipi_dsi.h"


#if defined(CONFIG_MT6229)
#include <linux/mt6229.h>
#endif

#if defined(CONFIG_TOUCHSCREEN_GT9XX)
#include "../../../drivers/input/touchscreen/gt9xx/gt9xx.h"

static int tp_probe_flag = 0;
extern int get_probe_state(void)
{
       return tp_probe_flag;
}

extern int set_probe_state(int state)
{
       tp_probe_flag = state;
}
#endif //goodix GT9xx

/* Android Parameter */ 
static int ap_mdm = 0; 
module_param(ap_mdm, int, 0644); 
static int ap_has_alsa = 0; 
module_param(ap_has_alsa, int, 0644);
static int ap_data_only = 2; 
module_param(ap_data_only, int, 0644); 
static int ap_has_earphone = 0;
module_param(ap_has_earphone, int, 0644);

static struct rk29_keys_button key_button[] = {
	{
		.desc	= "play",
		.code	= KEY_POWER,
		.gpio	= RK30_PIN0_PA4, 
		.active_low = PRESS_LEV_LOW,
		.wakeup	= 1,
	},
		{
		.desc	= "menu",
		.code	= EV_MENU,
		.adc_value	= 159,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
		.code_long_press = KEY_VOLUMEDOWN,
	},
	{
		.desc	= "esc",
		.code	= KEY_BACK,
		.adc_value	= 1,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
		.code_long_press = KEY_VOLUMEUP,
	},

};
struct rk29_keys_platform_data rk29_keys_pdata = {
	.buttons	= key_button,
	.nbuttons	= ARRAY_SIZE(key_button),
	.chn	= 1,  //chn: 0-7, if do not use ADC,set 'chn' -1
};


extern int get_harware_version()
{
        return 2;
}

#if defined(CONFIG_TOUCHSCREEN_CT36X) || defined(CONFIG_CT36X_TS)

static struct ct36x_platform_data ct36x_info = {
	.model   = 363,
	.x_max   = 1920,
	.y_max   = 1200,
	.rst_io = {
		.gpio		= RK30_PIN0_PB6,
		.active_low	= GPIO_HIGH,
	},
	.irq_io = {
		.gpio		= RK30_PIN1_PB7,
		.active_low	= GPIO_HIGH,
	},
	.orientation = {1, 0, 0, 1}, //1001 - left/right /up down 1010 xy swap

};

#endif

#if defined(CONFIG_GYRO_L3G20D)
#include <linux/l3g4200d.h>
#define L3G20D_INT_PIN	RK30_PIN0_PB4

static int l3g20d_init_platform_hw(void)
{	
		return 0;
}

static struct sensor_platform_data l3g20d_info = {
		.type = SENSOR_TYPE_GYROSCOPE,
		.irq_enable = 1,
		.poll_delay_ms = 0, //30,
		.orientation = {1, 0, 0, 0, -1, 0, 0, 0, -1},
		.init_platform_hw = l3g20d_init_platform_hw,
		.x_min = 40,//x_min,y_min,z_min = (0-100) according to hardware
		.y_min = 40,
		.z_min = 20,
};

#endif

static struct spi_board_info board_spi_devices[] = {
};

/***********************************************************
*	rk30  backlight
************************************************************/
#define PWM_ID            3
#define PWM_MODE	  PWM3


#define PWM_EFFECT_VALUE  1 //possibly 0?

#define BL_EN_PIN         RK30_PIN0_PA2
#define BL_EN_VALUE       GPIO_HIGH

#define HUB_RST_PIN RK30_PIN0_PC2

static int rk29_backlight_io_init(void)
{
	int ret = 0, hubret = 0;

	iomux_set(PWM_MODE);
	
	ret = gpio_request(BL_EN_PIN, "bl_en");
	if (ret != 0) {
		gpio_free(BL_EN_PIN);
	} else {
		gpio_direction_output(BL_EN_PIN, 0);
		gpio_set_value(BL_EN_PIN, BL_EN_VALUE);
	}

	hubret = gpio_request(HUB_RST_PIN, "hub_rst");
	if (hubret != 0) {
		gpio_free(HUB_RST_PIN);
	} else
	{
		gpio_direction_output(HUB_RST_PIN, 0);
		gpio_set_value(HUB_RST_PIN, GPIO_HIGH);
	}

	return ret;
}

static int rk29_backlight_io_deinit(void)
{
	int ret = 0;

	gpio_direction_output(BL_EN_PIN, !BL_EN_VALUE);
	gpio_free(BL_EN_PIN);

	int pwm_gpio = iomux_mode_to_gpio(PWM_MODE);

	ret = gpio_request(pwm_gpio, "bl_pwm");
	if (ret != 0) {
		gpio_free(pwm_gpio);
	} else
	{
		gpio_direction_output(pwm_gpio, GPIO_LOW);
	}

	return ret;
}

static int rk29_backlight_pwm_suspend(void)
{
	int ret, pwm_gpio;

	pwm_gpio = iomux_mode_to_gpio(PWM_MODE);//PWM_MODE = PWM3

	ret = gpio_request(pwm_gpio, "bl_pwm");
	if (ret != 0) {
		gpio_free(pwm_gpio);
		printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
	} else
	{
		gpio_direction_output(pwm_gpio, GPIO_LOW);
	}
	
	gpio_direction_output(BL_EN_PIN, !BL_EN_VALUE);//d33 - !BL_EN_VALUE replaces 0
	gpio_set_value(BL_EN_PIN, !BL_EN_VALUE);
	
	return ret;
}

static int rk29_backlight_pwm_resume(void)
{
	int pwm_gpio = iomux_mode_to_gpio(PWM_MODE);

	gpio_free(pwm_gpio);
	iomux_set(PWM_MODE);

	msleep(30);
	gpio_direction_output(BL_EN_PIN, 1);
	gpio_set_value(BL_EN_PIN, BL_EN_VALUE);

	return 0;
}

static struct rk29_bl_info rk29_bl_info = {

        .min_brightness = 20,
        .max_brightness = 150,
	.pre_div = 40 * 1000,  // pwm output clk: 40k;
        .brightness_mode =BRIGHTNESS_MODE_CONIC,
	.pwm_id = PWM_ID,
	.bl_ref = PWM_EFFECT_VALUE,
	.io_init = rk29_backlight_io_init,
	.io_deinit = rk29_backlight_io_deinit,
	.pwm_suspend = rk29_backlight_pwm_suspend,
	.pwm_resume = rk29_backlight_pwm_resume,
};

static struct platform_device rk29_device_backlight = {
	.name	= "rk29_backlight",
	.id 	= -1,
	.dev	= {
		.platform_data  = &rk29_bl_info,
	}
};

/*MMA8452 gsensor*/
#if defined (CONFIG_GS_MMA8452)
#define MMA8452_INT_PIN   RK30_PIN0_PB7

static int mma8452_init_platform_hw(void)
{
	return 0;
}

static struct sensor_platform_data mma8452_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 30,
        .init_platform_hw = mma8452_init_platform_hw,
        .orientation = {-1, 0, 0, 0, 0, -1, 0, 1, 0},
};
#endif
#if defined (CONFIG_GS_LIS3DH)
#define LIS3DH_INT_PIN   RK30_PIN0_PB7

static int lis3dh_init_platform_hw(void)
{

        return 0;
}

static struct sensor_platform_data lis3dh_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 30,
        .init_platform_hw = lis3dh_init_platform_hw,
	.orientation = {0, -1, 0, -1, 0, 0, 0, 0, -1}, //M9PRO

};

#endif //(CONFIG_GS_LIS3DH)

#if defined (CONFIG_COMPASS_AK8963)
static struct sensor_platform_data akm8963_info =
{
       .type = SENSOR_TYPE_COMPASS,
       .irq_enable = 1,
       //.poll_delay_ms = 0,
       .layout = 8,	//M7Pro has  5
       .m_layout =
       {
			   {
						{0, 1, 0},
						{-1, 0, 0},
						{0, 0, 1},
				},
               

               {
                       {1, 0, 0},
                       {0, 1, 0},
                       {0, 0, 1},
               },
               {//gsensor
                       {0, -1, 0},
                       {1, 0, 0},
                       {0, 0, -1},
               },

              {
                       {1, 0, 0},
                       {0, 1, 0},
                       {0, 0, 1},
               },
       }
};

#endif

#if defined (CONFIG_COMPASS_AK8975)
static struct sensor_platform_data akm8975_info =
{
	.type = SENSOR_TYPE_COMPASS,
	.irq_enable = 1,
	.poll_delay_ms = 30,
	.m_layout = 
	{
		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},

		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},

		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},

		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},
	}
};

#endif

#if defined(CONFIG_MT6229)
static int mt6229_io_init(void)
{
	 return 0;
}

static int mt6229_io_deinit(void)
{
		return 0;
}
 
struct rk29_mt6229_data rk29_mt6229_info = {
	.io_init = mt6229_io_init,
  	.io_deinit = mt6229_io_deinit,
	.modem_power_en = RK30_PIN0_PC6,
	.bp_power = RK30_PIN2_PD5,
	//.bp_reset = RK30_PIN2_PD5,
	.modem_usb_en = RK30_PIN0_PC7, 
	.modem_uart_en = RK30_PIN2_PD4,

	.bp_wakeup_ap = RK30_PIN0_PC5,
	.ap_ready = RK30_PIN0_PC4,
	};
struct platform_device rk29_device_mt6229 = {	
        .name = "mt6229",	
    	.id = -1,	
	.dev		= {
		.platform_data = &rk29_mt6229_info,
	}    	
    };
#endif

#if defined(CONFIG_GYRO_L3G4200D)

#include <linux/l3g4200d.h>
#if defined (CONFIG_PIPO_M6PRO)
#define L3G4200D_INT_PIN  RK30_PIN1_PA7
#else
#define L3G4200D_INT_PIN  RK30_PIN0_PB4
#endif

static int l3g4200d_init_platform_hw(void)
{
	return 0;
}

static struct sensor_platform_data l3g4200d_info = {
	.type = SENSOR_TYPE_GYROSCOPE,
	.irq_enable = 1,
	.poll_delay_ms = 0,
	.orientation = {1, 0, 0 , 0 , -1, 0, 0, 0, -1},
	.init_platform_hw = l3g4200d_init_platform_hw,
	.x_min = 40,//x_min,y_min,z_min = (0-100) according to hardware
	.y_min = 40,
	.z_min = 20,
};

#endif


#define LCD_CS_PIN         INVALID_GPIO//RK30_PIN0_PB0
#define LCD_CS_VALUE       GPIO_HIGH

#define LCD_EN_PIN         RK30_PIN1_PB2
#define LCD_EN_VALUE       GPIO_LOW

#define LCD_PWR_PIN         INVALID_GPIO //RK30_PIN1_PB5
#define LCD_PWR_VALUE       GPIO_HIGH

static int rk_fb_io_init(struct rk29_fb_setting_info *fb_setting)
{
	int ret = 0;
	if(LCD_CS_PIN !=INVALID_GPIO)
	{
		ret = gpio_request(LCD_CS_PIN, "lcd cs");
		if (ret != 0)
		{
			gpio_free(LCD_CS_PIN);
			printk(KERN_ERR "request lcd cs pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(LCD_CS_PIN, LCD_CS_VALUE);
		}
	}

	if(LCD_EN_PIN !=INVALID_GPIO)
	{
		ret = gpio_request(LCD_EN_PIN, "lcd en");
		if (ret != 0)
		{
			gpio_free(LCD_EN_PIN);
			printk(KERN_ERR "request lcd en pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(LCD_EN_PIN, LCD_EN_VALUE);
		}
	}

	if(LCD_PWR_PIN !=INVALID_GPIO)
	{
		ret = gpio_request(LCD_PWR_PIN, "lcd pwr");
		if (ret != 0)
		{
			gpio_free(LCD_PWR_PIN);
			printk(KERN_ERR "request lcd pwr pin fail!\n");
			return -1;
		}
		else
		{
			gpio_direction_output(LCD_PWR_PIN, LCD_PWR_VALUE);
		}
	}

	return 0;
}

static int rk_fb_io_disable(void)
{
	if(LCD_CS_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_CS_PIN, !LCD_CS_VALUE);
	}


	if(LCD_EN_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_EN_PIN, !LCD_EN_VALUE);
	}

	if(LCD_PWR_PIN !=INVALID_GPIO)
	{
		gpio_direction_output(LCD_PWR_PIN, !LCD_PWR_VALUE);
	}

	return 0;
}
static int rk_fb_io_enable(void)
{
	if(LCD_CS_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_CS_PIN, LCD_CS_VALUE);
	}


	if(LCD_EN_PIN !=INVALID_GPIO)
	{
		gpio_set_value(LCD_EN_PIN, LCD_EN_VALUE);
	}

	if(LCD_PWR_PIN !=INVALID_GPIO)
	{
		gpio_direction_output(LCD_PWR_PIN, LCD_PWR_VALUE);
	}

	return 0;
}

struct rk29fb_info lcdc0_screen_info = {
	.prop           = EXTEND,       //extend display device
       .lcd_info  = NULL,
       .set_screen_info = hdmi_init_lcdc,
};

struct rk29fb_info lcdc1_screen_info = {
	.prop	   = PRMRY,		//primary display device
	.io_init   = rk_fb_io_init,
	.io_disable = rk_fb_io_disable,
	.io_enable = rk_fb_io_enable,
	.set_screen_info = set_lcd_info,

};



static struct resource resource_fb[] = {
	[0] = {
		.name  = "fb0 buf",
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = "ipp buf",  //for rotate
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.name  = "fb2 buf",
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device device_fb = {
	.name		= "rk-fb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resource_fb),
	.resource	= resource_fb,
};

static struct resource resource_mali[] = {
	[0] = {
	.name  = "ump buf",
	.start = 0,
	.end   = 0,
	.flags = IORESOURCE_MEM,
	},

};

static struct platform_device device_mali= {
	.name		= "mali400_ump",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resource_mali),
	.resource	= resource_mali,
};

static struct resource resource_lcdc0[] = {
	[0] = {
		.name  = "lcdc0 reg",
		.start = RK30_LCDC0_PHYS,
		.end   = RK30_LCDC0_PHYS + RK30_LCDC0_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	
	[1] = {
		.name  = "lcdc0 irq",
		.start = IRQ_LCDC0,
		.end   = IRQ_LCDC0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device device_lcdc0 = {
	.name		  = "rk30-lcdc",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(resource_lcdc0),
	.resource	  = resource_lcdc0,
	.dev 		= {
		.platform_data = &lcdc0_screen_info,
	},
};

static struct resource resource_lcdc1[] = {
	[0] = {
		.name  = "lcdc1 reg",
		.start = RK30_LCDC1_PHYS,
		.end   = RK30_LCDC1_PHYS + RK30_LCDC1_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = "lcdc1 irq",
		.start = IRQ_LCDC1,
		.end   = IRQ_LCDC1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device device_lcdc1 = {
	.name		  = "rk30-lcdc",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(resource_lcdc1),
	.resource	  = resource_lcdc1,
	.dev 		= {
		.platform_data = &lcdc1_screen_info,
	},
};

#define RK_HDMI_RST_PIN		RK30_PIN3_PB2
#define RK_HDMI_POWER_EN_PIN	INVALID_GPIO


static int rk_hdmi_power_init(void)
{
	int ret_en = 0;
	int ret_rst = 0;

	printk("func %s, line %d: \n", __FUNCTION__, __LINE__);

	if(RK_HDMI_POWER_EN_PIN != INVALID_GPIO) {
		ret_en = gpio_request(RK_HDMI_POWER_EN_PIN, "hdmi_enable");
		if (ret_en !=0) {
			printk("func %s, line %d: request gpio hdmi power en pin fail\n", __FUNCTION__, __LINE__);
			gpio_free(RK_HDMI_POWER_EN_PIN);
		}
		gpio_direction_output(RK_HDMI_POWER_EN_PIN, GPIO_LOW);
		gpio_set_value(RK_HDMI_POWER_EN_PIN, GPIO_LOW);
	}

	msleep(100);

	if(RK_HDMI_RST_PIN != INVALID_GPIO) {
		ret_rst = gpio_request(RK_HDMI_RST_PIN, "hdmi_reset");
		if (ret_rst != 0) {
			printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
			gpio_free(RK_HDMI_RST_PIN);
		}
		gpio_direction_output(RK_HDMI_RST_PIN, GPIO_LOW);
		gpio_set_value(RK_HDMI_RST_PIN, GPIO_LOW);
		msleep(100);
		gpio_set_value(RK_HDMI_RST_PIN, GPIO_HIGH);
		msleep(50);
	}

	return ret_rst + ret_en;
}

static struct rk_hdmi_platform_data rk_hdmi_pdata = {
	.io_init = rk_hdmi_power_init,
};

#define ION_RESERVE_SIZE        (80 * SZ_1M)
#define ION_RESERVE_SIZE_120M   (120 * SZ_1M)

static struct ion_platform_data rk30_ion_pdata = {
	.nr = 1,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = ION_NOR_HEAP_ID,
			.name = "norheap",
			//.size = ION_RESERVE_SIZE,
		}
	},
};

static struct platform_device device_ion = {
	.name = "ion-rockchip",
	.id = 0,
	.dev = {
		.platform_data = &rk30_ion_pdata,
	},
};
/**************************************************************************************************
 * SDMMC devices,  include the module of SD,MMC,and sdio.noted by xbw at 2012-03-05
**************************************************************************************************/

#include "board-rk3188-ds1006h-sdmmc-config.c"
#include "../plat-rk/rk-sdmmc-ops.c"
#include "../plat-rk/rk-sdmmc-wifi.c"

#ifdef CONFIG_SDMMC0_RK29
static int rk29_sdmmc0_cfg_gpio(void)
{
#ifdef CONFIG_SDMMC_RK29_OLD
	iomux_set(MMC0_CMD);
	iomux_set(MMC0_CLKOUT);
	iomux_set(MMC0_D0);
	iomux_set(MMC0_D1);
	iomux_set(MMC0_D2);
	iomux_set(MMC0_D3);

	iomux_set_gpio_mode(iomux_mode_to_gpio(MMC0_DETN));

	gpio_request(RK30_PIN3_PA7, "sdmmc-power");
	gpio_direction_output(RK30_PIN3_PA7, GPIO_LOW);

#else
	rk29_sdmmc_set_iomux(0, 0xFFFF);

    #if defined(CONFIG_SDMMC0_RK29_SDCARD_DET_FROM_GPIO)
        #if SDMMC_USE_NEW_IOMUX_API
        iomux_set_gpio_mode(iomux_gpio_to_mode(RK29SDK_SD_CARD_DETECT_N));
        #else
        rk30_mux_api_set(RK29SDK_SD_CARD_DETECT_PIN_NAME, RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO);
        #endif
    #else
        #if SDMMC_USE_NEW_IOMUX_API       
        iomux_set(MMC0_DETN);
        #else
        rk30_mux_api_set(RK29SDK_SD_CARD_DETECT_PIN_NAME, RK29SDK_SD_CARD_DETECT_IOMUX_FMUX);
        #endif
    #endif	

#if defined(CONFIG_SDMMC0_RK29_WRITE_PROTECT)
	gpio_request(SDMMC0_WRITE_PROTECT_PIN, "sdmmc-wp");
	gpio_direction_input(SDMMC0_WRITE_PROTECT_PIN);
#endif

#endif

	return 0;
}

#define CONFIG_SDMMC0_USE_DMA
struct rk29_sdmmc_platform_data default_sdmmc0_data = {
	.host_ocr_avail =
	    (MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
	     MMC_VDD_33_34 | MMC_VDD_34_35 | MMC_VDD_35_36),
	.host_caps =
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
	.io_init = rk29_sdmmc0_cfg_gpio,

#if !defined(CONFIG_SDMMC_RK29_OLD)
	.set_iomux = rk29_sdmmc_set_iomux,
#endif
#ifdef USE_SDMMC_DATA4_DATA7	
    .emmc_is_selected = NULL,
#endif
	.dma_name = "sd_mmc",
#ifdef CONFIG_SDMMC0_USE_DMA
	.use_dma = 1,
#else
	.use_dma = 0,
#endif

#if defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC) && defined(CONFIG_USE_SDMMC0_FOR_WIFI_DEVELOP_BOARD)
    .status = rk29sdk_wifi_mmc0_status,
    .register_status_notify = rk29sdk_wifi_mmc0_status_register,
#endif

#if defined(RK29SDK_SD_CARD_PWR_EN) || (INVALID_GPIO != RK29SDK_SD_CARD_PWR_EN)
    .power_en = RK29SDK_SD_CARD_PWR_EN,
    .power_en_level = RK29SDK_SD_CARD_PWR_EN_LEVEL,
#else
    .power_en = INVALID_GPIO,
    .power_en_level = GPIO_LOW,
#endif    
	.enable_sd_wakeup = 0,

#if defined(CONFIG_SDMMC0_RK29_WRITE_PROTECT)
	.write_prt = SDMMC0_WRITE_PROTECT_PIN,
	.write_prt_enalbe_level = SDMMC0_WRITE_PROTECT_ENABLE_VALUE;
#else
	.write_prt = INVALID_GPIO,
#endif

    .det_pin_info = {    
    #if defined(RK29SDK_SD_CARD_DETECT_N) || (INVALID_GPIO != RK29SDK_SD_CARD_DETECT_N)  
        .io             = RK29SDK_SD_CARD_DETECT_N, //INVALID_GPIO,
        .enable         = RK29SDK_SD_CARD_INSERT_LEVEL,
        #ifdef RK29SDK_SD_CARD_DETECT_PIN_NAME
        .iomux          = {
            .name       = RK29SDK_SD_CARD_DETECT_PIN_NAME,
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO
            .fgpio      = RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO,
            #endif
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FMUX
            .fmux       = RK29SDK_SD_CARD_DETECT_IOMUX_FMUX,
            #endif
        },
        #endif
    #else
        .io             = INVALID_GPIO,
        .enable         = GPIO_LOW,
    #endif    
    }, 

};
#endif // CONFIG_SDMMC0_RK29

#ifdef CONFIG_SDMMC1_RK29
#define CONFIG_SDMMC1_USE_DMA
static int rk29_sdmmc1_cfg_gpio(void)
{
#if defined(CONFIG_SDMMC_RK29_OLD)
	iomux_set(MMC1_CMD);
	iomux_set(MMC1_CLKOUT);
	iomux_set(MMC1_D0);
	iomux_set(MMC1_D1);
	iomux_set(MMC1_D2);
	iomux_set(MMC1_D3);
#else

#if defined(CONFIG_SDMMC1_RK29_WRITE_PROTECT)
	gpio_request(SDMMC1_WRITE_PROTECT_PIN, "sdio-wp");
	gpio_direction_input(SDMMC1_WRITE_PROTECT_PIN);
#endif

#endif

	return 0;
}

struct rk29_sdmmc_platform_data default_sdmmc1_data = {
	.host_ocr_avail =
	    (MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
	     MMC_VDD_33_34),

#if !defined(CONFIG_USE_SDMMC1_FOR_WIFI_DEVELOP_BOARD)
	.host_caps = (MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ |
		      MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
#else
	.host_caps =
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
#endif

	.io_init = rk29_sdmmc1_cfg_gpio,

#if !defined(CONFIG_SDMMC_RK29_OLD)
	.set_iomux = rk29_sdmmc_set_iomux,
#endif
#ifdef USE_SDMMC_DATA4_DATA7	
	.emmc_is_selected = NULL,
#endif
	.dma_name = "sdio",
#ifdef CONFIG_SDMMC1_USE_DMA
	.use_dma = 1,
#else
	.use_dma = 0,
#endif

#if defined(CONFIG_WIFI_CONTROL_FUNC) || defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC)
    .status = rk29sdk_wifi_status,
    .register_status_notify = rk29sdk_wifi_status_register,
#endif

#if defined(CONFIG_SDMMC1_RK29_WRITE_PROTECT)
	.write_prt = SDMMC1_WRITE_PROTECT_PIN,
	    .write_prt_enalbe_level = SDMMC1_WRITE_PROTECT_ENABLE_VALUE;
#else
	.write_prt = INVALID_GPIO,
#endif

    #if defined(CONFIG_RK29_SDIO_IRQ_FROM_GPIO)
        .sdio_INT_gpio = RK29SDK_WIFI_SDIO_CARD_INT,
    #endif

    .det_pin_info = {    
#if defined(CONFIG_USE_SDMMC1_FOR_WIFI_DEVELOP_BOARD)
     #if defined(RK29SDK_SD_CARD_DETECT_N) || (INVALID_GPIO != RK29SDK_SD_CARD_DETECT_N)  
        .io             = RK29SDK_SD_CARD_DETECT_N,
     #else
         .io             = INVALID_GPIO,
     #endif   

        .enable         = RK29SDK_SD_CARD_INSERT_LEVEL,
        #ifdef RK29SDK_SD_CARD_DETECT_PIN_NAME
        .iomux          = {
            .name       = RK29SDK_SD_CARD_DETECT_PIN_NAME,
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO
            .fgpio      = RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO,
            #endif
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FMUX
            .fmux       = RK29SDK_SD_CARD_DETECT_IOMUX_FMUX,
            #endif
        },
        #endif
 #else
        .io             = INVALID_GPIO,
        .enable         = GPIO_LOW,
#endif
    },
   
	.enable_sd_wakeup = 0,
};
#endif //endif--#ifdef CONFIG_SDMMC1_RK29

#ifdef CONFIG_SDMMC2_RK29
static int rk29_sdmmc2_cfg_gpio(void)
{
    ;
}

struct rk29_sdmmc_platform_data default_sdmmc2_data = {
	.host_ocr_avail =
	    (MMC_VDD_165_195|MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 | MMC_VDD_33_34),

	.host_caps = (MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA| MMC_CAP_NONREMOVABLE  |
	        MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED | MMC_CAP_UHS_SDR12 |MMC_CAP_UHS_SDR25 |MMC_CAP_UHS_SDR50),

	.io_init = rk29_sdmmc2_cfg_gpio,
	.set_iomux = rk29_sdmmc_set_iomux,
	.emmc_is_selected = sdmmc_is_selected_emmc,

	//.power_en = INVALID_GPIO,
   // .power_en_level = GPIO_LOW,

	.dma_name = "emmc",
	.use_dma = 1,

};
#endif//endif--#ifdef CONFIG_SDMMC2_RK29

/**************************************************************************************************
 * the end of setting for SDMMC devices
**************************************************************************************************/

//cw015 battery
/*
   note the follow array must set depend on the battery that you use
   you must send the battery to cellwise-semi the contact information:
   name: chen gan; tel:13416876079; E-mail: ben.chen@cellwise-semi.com
 */
static u8 config_info[SIZE_BATINFO] = {

//2- ?e��-_DS1006_JSH101_MB7600_ProfileV3LT_20 130705.txt
    0x15, 0x17, 0x5B, 0x58, 0x57,
	0x55, 0x4F, 0x4C, 0x4A, 0x49,
	0x47, 0x45, 0x41, 0x37, 0x2E,
	0x26, 0x1D, 0x19, 0x15, 0x17,
	0x20, 0x3C, 0x48, 0x43, 0x35,
	0x5A, 0x0A, 0xE1, 0x1C, 0x39,
    0x44, 0x6D, 0x78, 0x6B, 0x66,
	0x6B, 0x42, 0x1B, 0x52, 0x4D,
	0x0F, 0x22, 0x52, 0x87, 0x8F,
	0x91, 0x94, 0x52, 0x82, 0x8C,
	0x92, 0x96, 0x2E, 0x82, 0xA5,
	0xCB, 0x2F, 0x7D, 0x72, 0xA5,
    0xB5, 0xC1, 0x27, 0x0A

};

static struct cw_bat_platform_data cw_bat_platdata = {
	.dc_det_pin	= RK30_PIN0_PB2,
	.bat_low_pin    = RK30_PIN0_PB1,
 	.chg_ok_pin	= RK30_PIN0_PA6,//INVALID_GPIO??

	.dc_det_level	= GPIO_LOW,
	.bat_low_level  = GPIO_LOW,   
	.chg_ok_level	= GPIO_HIGH,

	.cw_bat_config_info     = config_info,
};

// bluetooth rfkill device, its driver in net/rfkill/rfkill-rk.c
static struct rfkill_rk_platform_data rfkill_rk_platdata = {
    .type               = RFKILL_TYPE_BLUETOOTH,

    .poweron_gpio       = { // BT_REG_ON
        .io             = RK30_PIN3_PD1,
        .enable         = GPIO_HIGH,
        .iomux          = {
            .name       = "bt_poweron",
            .fgpio      = GPIO3_D1,
        },
    },

    .reset_gpio         = { // BT_RST
        .io             = INVALID_GPIO,//RK30_PIN3_PD1, 
        .enable         = GPIO_LOW,
        .iomux          = {
            .name       = "bt_reset",
            .fgpio      = GPIO3_D1,
       },
   }, 

    .wake_gpio          = { // BT_WAKE, use to control bt's sleep and wakeup
        .io             = RK30_PIN3_PC6, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_HIGH,
        .iomux          = {
            .name       = "bt_wake",
            .fgpio      = GPIO3_C6,
        },
    },

    .wake_host_irq      = { // BT_HOST_WAKE, for bt wakeup host when it is in deep sleep
        .gpio           = {
            .io         = RK30_PIN0_PA5, // d33 standard setting
            .enable     = GPIO_LOW,      // set GPIO_LOW for falling, set 0 for rising
            .iomux      = {
                .name   = "bt_host_wake",
            },
        },
    },

    .rts_gpio           = { // UART_RTS, enable or disable BT's data coming
        .io             = RK30_PIN1_PA3, // set io to INVALID_GPIO for disable it
        .enable         = GPIO_LOW,
        .iomux          = {
            .name       = "bt_rts",
            .fgpio      = GPIO1_A3,
            .fmux       = UART0_RTSN,
        },
    },
};

static struct platform_device device_rfkill_rk = {
    .name   = "rfkill_rk",
    .id     = -1,
    .dev    = {
        .platform_data = &rfkill_rk_platdata,
    },
};


#define GPS_SPI_CLK RK30_PIN1_PA6 //low
#define GPS_SPI_CLK_GPIO GPIO1_A6

#define GPS_SPI_MOSI RK30_PIN1_PA5 //low
#define GPS_SPI_MOSI_GPIO GPIO1_A5

#define GPS_SPI_CS RK30_PIN1_PA7 //low
#define GPS_SPI_CS_GPIO GPIO1_A7

//GPS
int rk_gps_io_init(void)
{
	int ret;
	printk("%s \n", __FUNCTION__);

	iomux_set(GPIO1_B5);//VCC_EN 
	iomux_set(GPS_SPI_CLK_GPIO);//VCC_EN
	iomux_set(GPS_SPI_MOSI_GPIO);//VCC_EN 
	iomux_set(GPS_SPI_CS_GPIO);//VCC_EN  
	
	ret = gpio_request(RK30_PIN1_PB5, "gps_en");
	if (ret != 0 ) {
		gpio_free(RK30_PIN1_PB5);
		printk(KERN_ERR "GPS vcc request failed");
	} else {
		gpio_direction_output(RK30_PIN1_PB5, GPIO_LOW);
	}

	iomux_set(GPS_RFCLK);//GPS_CLK
	iomux_set(GPS_MAG);//GPS_MAG
	iomux_set(GPS_SIG);//GPS_SIGN

	ret = gpio_request(GPS_SPI_CLK, "gps_spi_clk");
	if (ret != 0 ) {
		gpio_free(GPS_SPI_CLK);
		printk(KERN_ERR "GPS_SPI_CLK request failed");
	} else {
		gpio_direction_output(GPS_SPI_CLK, GPIO_LOW);
	}

	ret = gpio_request(GPS_SPI_MOSI, "gps_spi_mosi");
	if (ret != 0 ) {
		gpio_free(GPS_SPI_MOSI);
		printk(KERN_ERR "GPS_SPI_MOSI request failed");
	} else {
		gpio_direction_output(GPS_SPI_MOSI, GPIO_LOW);
	}

	ret = gpio_request(GPS_SPI_CS, "gps_spi_cs");
	if (ret != 0 ) {
		gpio_free(GPS_SPI_CS);
		printk(KERN_ERR "GPS_SPI_CS request failed");
	} else {
		gpio_direction_output(GPS_SPI_CS, GPIO_LOW);
	}
	
	return 0;
}

int rk_gps_power_up(void)
{
	printk("%s \n", __FUNCTION__);

	return 0;
}

int rk_gps_power_down(void)
{
	printk("%s \n", __FUNCTION__);

	return 0;
}

int rk_gps_reset_set(int level)
{
	return 0;
}
int rk_enable_hclk_gps(void)
{
	struct clk *gps_aclk = NULL;
	gps_aclk = clk_get(NULL, "aclk_gps");
	if(gps_aclk) {
		clk_enable(gps_aclk);
		clk_put(gps_aclk);
		printk("%s \n", __FUNCTION__);
	}
	else
		printk("get gps aclk fail\n");
	return 0;

}

int rk_disable_hclk_gps(void)
{
	struct clk *gps_aclk = NULL;
	gps_aclk = clk_get(NULL, "aclk_gps");
	if(gps_aclk) {
		//TO wait long enough until GPS ISR is finished.
		msleep(5);
		clk_disable(gps_aclk);
		clk_put(gps_aclk);
		printk("%s \n", __FUNCTION__);
	}	
	else
		printk("get gps aclk fail\n");
	return 0;

};

struct rk_gps_data rk_gps_info = {
	.io_init = rk_gps_io_init,
	.power_up = rk_gps_power_up,
	.power_down = rk_gps_power_down,
	.reset = rk_gps_reset_set,
	.enable_hclk_gps = rk_enable_hclk_gps,
	.disable_hclk_gps = rk_disable_hclk_gps,
	.GpsSign = RK30_PIN1_PB3,
	.GpsMag = RK30_PIN1_PB2,        
	.GpsClk = RK30_PIN1_PB4, 
	.GpsVCCEn = RK30_PIN1_PB5, 
	.GpsSpi_CSO = GPS_SPI_CS, 
	.GpsSpiClk = GPS_SPI_CLK, 
	.GpsSpiMOSI = GPS_SPI_MOSI,
	.GpsIrq = IRQ_GPS,
	.GpsSpiEn = 1,
	.GpsAdcCh = 2,
	.u32GpsPhyAddr = RK30_GPS_PHYS,
	.u32GpsPhySize = RK30_GPS_SIZE,
};

struct platform_device rk_device_gps = {
	.name = "gps_hv5820b",
	.id = -1,
	.dev = {
		.platform_data = &rk_gps_info,
	}
};

static struct mt6622_platform_data mt6622_platdata = {
	.power_gpio	= { // BT_REG_ON
		.io	= RK30_PIN3_PC7, // RK30_PIN3_PD5, 
		.enable         = GPIO_HIGH,
		.iomux          = {
			.name       = NULL,
			},
		},

	.reset_gpio	= { // BT_RST
		.io	= RK30_PIN3_PD1,// RK30_PIN0_PD7,
		.enable         = GPIO_HIGH,
		.iomux          = {
			.name       = NULL,
			},
		},

	.irq_gpio	= {
		.io	= RK30_PIN0_PA5,//RK30_PIN3_PD2,
		.enable         = GPIO_HIGH,
		.iomux          = {
			.name       = NULL,
			},
		}
};

static struct platform_device device_mt6622 = {
		    .name   = "mt6622",
			.id     = -1,
			.dev    = {
			       .platform_data = &mt6622_platdata,
			},
};	


static struct platform_device *devices[] __initdata = {
	&device_ion,
	&rk29sdk_wifi_device,
	&device_rfkill_rk,
	&rk_device_gps,
	&device_mt6622,
#if defined(CONFIG_MT6229)
	&rk29_device_mt6229,
#endif
	&device_mali,
};


static int rk_platform_add_display_devices(void)
{
	struct platform_device *fb = NULL;  //fb
	struct platform_device *lcdc0 = NULL; //lcdc0
	struct platform_device *lcdc1 = NULL; //lcdc1
	struct platform_device *bl = NULL; //backlight
	fb = &device_fb;
	lcdc0 = &device_lcdc0,
	lcdc1 = &device_lcdc1,
	bl = &rk29_device_backlight,
	__rk_platform_add_display_devices(fb,lcdc0,lcdc1,bl);

	return 0;
	
}

// i2c
static struct i2c_board_info __initdata i2c0_info[] = {
#if defined (CONFIG_SND_SOC_RT5623)
	{
		.type			= "rt5623",
		.addr			= 0x1a,
		.flags			= 0,
	},
#endif

#if defined (CONFIG_GS_MMA8452)
	{
		.type	        = "gs_mma8452",
		.addr	        = 0x1d,
		.flags	        = 0,
		.irq	        = MMA8452_INT_PIN,
		.platform_data = &mma8452_info,
	},
#endif

#if defined (CONFIG_GS_LIS3DH)
	{
		.type	        = "gs_lis3dh",
		.addr	        = 0x19,   //0x19(SA0-->VCC), 0x18(SA0-->GND)
		.flags	        = 0,
		.irq	        = LIS3DH_INT_PIN,
		.platform_data = &lis3dh_info,
	},
#endif
#if defined (CONFIG_COMPASS_AK8963)
	{
		.type          = "ak8963",
		.addr          = 0x0d,
		.flags         = 0,
		.irq           = RK30_PIN3_PD7,	
		.platform_data = &akm8963_info,
	},
#endif

#if defined (CONFIG_COMPASS_AK8975)
	{
		.type          = "ak8975",
		.addr          = 0x0d,
		.flags         = 0,
		.irq           = RK30_PIN3_PD7,	
		.platform_data = &akm8975_info,
		.irq           = RK30_PIN3_PD7,	
		.platform_data = &akm8975_info,
	},
#endif
#if defined (CONFIG_GYRO_L3G4200D)
	{
		.type          = "l3g4200d_gryo",
		.addr          = 0x69,
		.flags         = 0,
		.irq           = L3G4200D_INT_PIN,
		.platform_data = &l3g4200d_info,
	},
#endif

#if defined (CONFIG_GYRO_L3G20D)
         {
                 .type          = "l3g20d_gryo",
                 .addr          = 0x6B,
                 .flags         = 0,
                 .irq           = L3G20D_INT_PIN,
                 .platform_data = &l3g20d_info,
         },
#endif

#if defined (CONFIG_SND_SOC_RT5631)
        {
                .type                   = "rt5631",
                .addr                   = 0x1a,
                .flags                  = 0,
        },
#endif
};

int __sramdata g_pmic_type =  0;

#define PMU_POWER_SLEEP RK30_PIN0_PA1
#define PMU_VSEL RK30_PIN3_PD3
#define ACT8846_HOST_IRQ                RK30_PIN0_PB3

static struct pmu_info  act8846_dcdc_info[] = {
	{
		.name          = "act_dcdc1",   //ddr
		.min_uv          = 1200000,
		.max_uv         = 1200000,
		.suspend_vol  =   1200000,
	},
	{
		.name          = "vdd_core",    //logic
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		#ifdef CONFIG_ACT8846_SUPPORT_RESET
		.suspend_vol  =  1200000,
		#else
		.suspend_vol  =  900000,
		#endif
	},
	{
		.name          = "vdd_cpu",   //arm
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		#ifdef CONFIG_ACT8846_SUPPORT_RESET
		.suspend_vol  =  1200000,
		#else
		.suspend_vol  =  900000,
		#endif
	},
	{
		.name          = "act_dcdc4",   //vccio
		.min_uv         = 3300000,
		.max_uv         = 3300000,
		.suspend_vol    = 3300000,
	},
	
};
static  struct pmu_info  act8846_ldo_info[] = {
	{
		.name          = "act_ldo1",   //vdd10
		.min_uv          = 1000000,
		.max_uv         = 1000000,
	},
	{
		.name          = "act_ldo2",    //vdd12
		.min_uv          = 1200000,
		.max_uv         = 1200000,
	},
	{
		.name          = "act_ldo3",   //vcc18_cif
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "act_ldo4",   //vcca33
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "act_ldo5",   //vcctp
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "act_ldo6",   //vcc_jetta
		.min_uv         = 2800000,
		.max_uv         = 2800000,
	},
	{
		.name          = "act_ldo7",   //vcc18
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "act_ldo8",   //vcc28_cif
		.min_uv          = 2800000,
		.max_uv         = 2800000,
	},
 };

#include "../mach-rk30/board-pmu-act8846.c"

static struct i2c_board_info __initdata i2c1_info[] = {
	{
		.type    		= "act8846",
		.addr           = 0x5a, 
		.flags			= 0,
		.irq            = ACT8846_HOST_IRQ,
		.platform_data=&act8846_data,
	},
	{
		.type                   = "rtc_hym8563",
		.addr           = 0x51,
		.flags                  = 0,
		.irq            = RK30_PIN0_PB5,
	},
	{
        	.type           = "cw201x",
        	.addr           = 0x62,
        	.flags          = 0,
        	.platform_data  = &cw_bat_platdata,
	},

};


void __sramfunc board_pmu_suspend(void)
{      
       if(pmic_is_act8846())
       board_pmu_act8846_suspend(); 
}

void __sramfunc board_pmu_resume(void)
{      
       if(pmic_is_act8846())
       board_pmu_act8846_resume(); 
}

 int __sramdata gpio3d6_iomux,gpio3d6_do,gpio3d6_dir,gpio3d6_en;

#define grf_readl(offset)	readl_relaxed(RK30_GRF_BASE + offset)
#define grf_writel(v, offset)	do { writel_relaxed(v, RK30_GRF_BASE + offset); dsb(); } while (0)
 
void __sramfunc rk30_pwm_logic_suspend_voltage(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR

//	int gpio0d7_iomux,gpio0d7_do,gpio0d7_dir,gpio0d7_en;
	sram_udelay(10000);
	gpio3d6_iomux = grf_readl(GRF_GPIO3D_IOMUX);
	gpio3d6_do = grf_readl(GRF_GPIO3H_DO);
	gpio3d6_dir = grf_readl(GRF_GPIO3H_DIR);
	gpio3d6_en = grf_readl(GRF_GPIO3H_EN);

	grf_writel((1<<28), GRF_GPIO3D_IOMUX);
	grf_writel((1<<30)|(1<<14), GRF_GPIO3H_DIR);
	grf_writel((1<<30)|(1<<14), GRF_GPIO3H_DO);
	grf_writel((1<<30)|(1<<14), GRF_GPIO3H_EN);
#endif 
}
void __sramfunc rk30_pwm_logic_resume_voltage(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	grf_writel((1<<28)|gpio3d6_iomux, GRF_GPIO3D_IOMUX);
	grf_writel((1<<30)|gpio3d6_en, GRF_GPIO3H_EN);
	grf_writel((1<<30)|gpio3d6_dir, GRF_GPIO3H_DIR);
	grf_writel((1<<30)|gpio3d6_do, GRF_GPIO3H_DO);
	sram_udelay(10000);

#endif

}
extern void pwm_suspend_voltage(void);
extern void pwm_resume_voltage(void);
void  rk30_pwm_suspend_voltage_set(void)
{
}
void  rk30_pwm_resume_voltage_set(void)
{
}


static struct i2c_board_info __initdata i2c2_info[] = {

#if defined (CONFIG_CT36X_TS) || defined(CONFIG_TOUCHSCREEN_CT36X)
	{
		.type	       = CT36X_NAME,
		.addr          = 0x01,
		.flags         = 0,
		.platform_data = &ct36x_info,
	},
#endif

#if defined(CONFIG_TOUCHSCREEN_GT9XX)
	{
		.type		= GTP_I2C_NAME ,
        	.addr		= GTP_I2C_ADDR ,
        	.flags		= 0,
        	.irq		= RK30_PIN1_PB7,
        	//.platform_data = &ts_pdata,
	},
#endif

	{
		.type		= "cat66121_hdmi",
		.addr		= 0x4c,
		.flags		= 0,
		.irq		= RK30_PIN2_PD6,
		.platform_data 	= &rk_hdmi_pdata,
	},


        {    
                .type           = "us5151",
                .addr           = 0x10,
                .flags          = 0, 
        },   

};


static struct i2c_board_info __initdata i2c3_info[] = {
};

static struct i2c_board_info __initdata i2c4_info[] = {

#if defined (CONFIG_SND_SOC_RT5616)
        {
                .type                   = "rt5616",
                .addr                   = 0x1b,
                .flags                  = 0,
        },
#endif

#if defined (CONFIG_SND_SOC_RT5631)
        {
                .type                   = "rt5631",
                .addr                   = 0x1a,
                .flags                  = 0,
        },
#endif

};


#define I2C_SDA_PIN     INVALID_GPIO
#define I2C_SCL_PIN     INVALID_GPIO
static int rk30_i2c_io_init(void)
{
        //set iomux (gpio) here
        return 0;
}
struct i2c_gpio_platform_data default_i2c_gpio_data = {
       .sda_pin = I2C_SDA_PIN,
       .scl_pin = I2C_SCL_PIN,
       .udelay = 5, // clk = 500/udelay = 100Khz
       .timeout = 100,//msecs_to_jiffies(100),
       .bus_num    = 5,
       .io_init = rk30_i2c_io_init,
};
static struct i2c_board_info __initdata i2c_gpio_info[] = {
};


static void __init rk30_i2c_register_board_info(void)
{
	i2c_register_board_info(0, i2c0_info, ARRAY_SIZE(i2c0_info));
	i2c_register_board_info(1, i2c1_info, ARRAY_SIZE(i2c1_info));
	i2c_register_board_info(2, i2c2_info, ARRAY_SIZE(i2c2_info));
	i2c_register_board_info(3, i2c3_info, ARRAY_SIZE(i2c3_info));
	i2c_register_board_info(4, i2c4_info, ARRAY_SIZE(i2c4_info));
	i2c_register_board_info(5, i2c_gpio_info, ARRAY_SIZE(i2c_gpio_info));
}
//end of i2c

#define POWER_ON_PIN RK30_PIN0_PA0   //power_hold
static void rk30_pm_power_off(void)
{
	printk(KERN_ERR "rk30_pm_power_off start...\n");
       if (pmic_is_act8846()) {
               printk("enter dcdet===========\n");
               if(gpio_get_value (RK30_PIN0_PB2) == GPIO_LOW)
               {
                       printk("enter restart===========\n");
                       arm_pm_restart(0, "charge");
               }
		/** code here may cause tablet cannot boot when shutdown without charger pluged in
		  * and then plug in charger. -- Cody Xie*/
               else
		{
			act8846_device_shutdown();
		}
		  
       }

	gpio_direction_output(POWER_ON_PIN, GPIO_LOW);
	while (1);
}

static void __init machine_rk30_board_init(void)
{
	int ret;	

	avs_init(); //NAND io remap - possibly comment out? d33
	gpio_request(POWER_ON_PIN, "poweronpin");
	gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);
	
	pm_power_off = rk30_pm_power_off;
	
	gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);
/*
//gps lan
	if (gpio_request(RK30_PIN3_PC7, "RK30_GPS_LAN")) {
		gpio_free(RK30_PIN3_PC7);
		printk("func %s, line %d: request GPS_LAN gpio failed\n", __FUNCTION__, __LINE__);
	} else {
		gpio_direction_output(RK30_PIN3_PC7, GPIO_HIGH);
	}

*/
	rk30_i2c_register_board_info();
	spi_register_board_info(board_spi_devices, ARRAY_SIZE(board_spi_devices));
	platform_add_devices(devices, ARRAY_SIZE(devices));
	rk_platform_add_display_devices();
	//board_usb_detect_init(RK30_PIN0_PA7);

	rk29sdk_wifi_bt_gpio_control_init();

	//MT5931/MT6622
	clk_set_rate(clk_get_sys("rk_serial.0", "uart"), 24*1000000);
}

#define HD_SCREEN_SIZE 1920UL*1200UL*4*3
static void __init rk30_reserve(void)
{
	int size, ion_reserve_size;
#if defined(CONFIG_ARCH_RK3188)
	/*if lcd resolution great than or equal to 1920*1200,reserve the ump memory 
*/
	if(!(get_fb_size() < ALIGN(HD_SCREEN_SIZE,SZ_1M)))
	{
		int ump_mem_phy_size=320UL*1024UL*1024UL; 
		resource_mali[0].start = board_mem_reserve_add("ump buf", ump_mem_phy_size); 
		resource_mali[0].end = resource_mali[0].start + ump_mem_phy_size -1;
	}
#endif
#ifdef CONFIG_ION
	size = ddr_get_cap() >> 20;
	if(size >= 1024) { // DDR >= 1G, set ion to 120M
		rk30_ion_pdata.heaps[0].size = ION_RESERVE_SIZE_120M;
		ion_reserve_size = ION_RESERVE_SIZE_120M;
	}
	else {
		rk30_ion_pdata.heaps[0].size = ION_RESERVE_SIZE;
		ion_reserve_size = ION_RESERVE_SIZE;
	}
	printk("ddr size = %d M, set ion_reserve_size size to %d\n", size, ion_reserve_size);
	//rk30_ion_pdata.heaps[0].base = board_mem_reserve_add("ion", ION_RESERVE_SIZE);
	rk30_ion_pdata.heaps[0].base = board_mem_reserve_add("ion", ion_reserve_size);
#endif

#ifdef CONFIG_FB_ROCKCHIP
	resource_fb[0].start = board_mem_reserve_add("fb0 buf", get_fb_size());
	resource_fb[0].end = resource_fb[0].start + get_fb_size()- 1;
#if 0
	resource_fb[1].start = board_mem_reserve_add("ipp buf", RK30_FB0_MEM_SIZE);
	resource_fb[1].end = resource_fb[1].start + RK30_FB0_MEM_SIZE - 1;
#endif

#if defined(CONFIG_FB_ROTATE) || !defined(CONFIG_THREE_FB_BUFFER)
	resource_fb[2].start = board_mem_reserve_add("fb2 buf",get_fb_size());
	resource_fb[2].end = resource_fb[2].start + get_fb_size() - 1;
#endif
#endif


#ifdef CONFIG_VIDEO_RK29
	rk30_camera_request_reserve_mem();
#endif
	
	//it must be more than 8MB
	rk_gps_info.u32MemoryPhyAddr = board_mem_reserve_add("gps", SZ_8M);
	board_mem_reserved();
}
/******************************** arm dvfs frequency volt table **********************************/
/**
 * dvfs_cpu_logic_table: table for arm and logic dvfs 
 * @frequency	: arm frequency
 * @cpu_volt	: arm voltage depend on frequency
 */

// ds1006h 10'
static struct cpufreq_frequency_table dvfs_arm_table_volt_level2[] = {
        {.frequency = 312 * 1000,       .index = 900 * 1000},
        {.frequency = 504 * 1000,       .index = 925 * 1000},
        {.frequency = 816 * 1000,       .index = 1000 * 1000},
        {.frequency = 1008 * 1000,      .index = 1075 * 1000},
        {.frequency = 1200 * 1000,      .index = 1200 * 1000},
        {.frequency = 1416 * 1000,      .index = 1250 * 1000},
        {.frequency = 1608 * 1000,      .index = 1350 * 1000},
        {.frequency = CPUFREQ_TABLE_END},
};
#define dvfs_arm_table dvfs_arm_table_volt_level2

/******************************** gpu dvfs frequency volt table **********************************/
//ds1006h 10'
static struct cpufreq_frequency_table dvfs_gpu_table_volt_level1[] = {	
       {.frequency = 133 * 1000,       .index = 975 * 1000},
	{.frequency = 200 * 1000,       .index = 1000 * 1000},
	{.frequency = 266 * 1000,       .index = 1025 * 1000},
	{.frequency = 300 * 1000,       .index = 1050 * 1000},
	{.frequency = 400 * 1000,       .index = 1100 * 1000},
	{.frequency = 600 * 1000,       .index = 1250 * 1000},
        {.frequency = CPUFREQ_TABLE_END},
};

#define dvfs_gpu_table dvfs_gpu_table_volt_level1

/******************************** ddr dvfs frequency volt table **********************************/
static struct cpufreq_frequency_table dvfs_ddr_table_volt_level0[] = {
	{.frequency = 200 * 1000 + DDR_FREQ_SUSPEND,	.index = 950 * 1000},
	//{.frequency = 300 * 1000 + DDR_FREQ_VIDEO,	.index = 1000 * 1000},
	//{.frequency = 460 * 1000 + DDR_FREQ_DUALVIEW,	.index = 1150 * 1000},
	{.frequency = 528 * 1000 + DDR_FREQ_NORMAL,     .index = 1250 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

#define dvfs_ddr_table dvfs_ddr_table_volt_level0

/******************************** arm dvfs frequency volt table end **********************************/



//#define DVFS_CPU_TABLE_SIZE	(ARRAY_SIZE(dvfs_cpu_logic_table))
//static struct cpufreq_frequency_table cpu_dvfs_table[DVFS_CPU_TABLE_SIZE];
//static struct cpufreq_frequency_table dep_cpu2core_table[DVFS_CPU_TABLE_SIZE];
int get_max_freq(struct cpufreq_frequency_table *table)
{
	int i,temp=0;
	
	for(i=0;table[i].frequency!= CPUFREQ_TABLE_END;i++)
	{
		if(temp<table[i].frequency)
			temp=table[i].frequency;
	}	
	printk("get_max_freq=%d\n",temp);
	return temp;
}

void __init board_clock_init(void)
{
	u32 flags=RK30_CLOCKS_DEFAULT_FLAGS;
	
	rk30_clock_data_init(periph_pll_default, codec_pll_default, flags);
	dvfs_set_freq_volt_table(clk_get(NULL, "cpu"), dvfs_arm_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "gpu"), dvfs_gpu_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "ddr"), dvfs_ddr_table);

}

MACHINE_START(RK30, "RK30board")
	.boot_params	= PLAT_PHYS_OFFSET + 0x800,
	.fixup		= rk30_fixup,
	.reserve	= &rk30_reserve,
	.map_io		= rk30_map_io,
	.init_irq	= rk30_init_irq,
	.timer		= &rk30_timer,
	.init_machine	= machine_rk30_board_init,
MACHINE_END
