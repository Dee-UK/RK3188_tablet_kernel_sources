/*****************************************************************************
* Copyright(c) O2Micro, 2013. All rights reserved.
*	
* O2Micro OZ8806 battery gauge driver
* File: OZ8806_battery.c

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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/fs.h>
#include <mach/gpio.h>
#include <linux/power_supply.h>
#include <mach/board.h>

#include <linux/string.h>
#include <asm/irq.h>

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/syscalls.h>
#include <linux/string.h>


#include <linux/miscdevice.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#include <linux/version.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
//you must add this code here for O2MICRO
#include "parameter.h"


/****************************************************************************
* Struct section
*  add struct #define here if any
***************************************************************************/

#define PROC_MEMSHARE_DIR         "memshare"
#define PROC_MEMSHARE_PHYADDR     "phymem_addr"
#define PROC_MEMSHARE_SIZE        "phymem_size"

#if 0
/*alloc one page. 4096 bytes*/
#define PAGE_ORDER                  0
#endif

/*alloc 2 page. 8192 bytes*/
#define PAGE_ORDER                  1

/*this value can get from PAGE_ORDER*/
#define PAGES_NUMBER     			2

struct proc_dir_entry *proc_memshare_dir ;
unsigned long kernel_memaddr = 0;
unsigned long pa_memaddr= 0;
unsigned long kernel_memsize= 0;

#define VERSION		   "2012.9.25/3.00.01"	


#define DEVICE_NAME	"OZ8806_REG"
#define OZ8806_MAGIC 12
#define OZ8806_IOCTL_RESETLANG 		_IO(OZ8806_MAGIC,0) 
#define OZ8806_IOCTL_GETLANG  		_IOR(OZ8806_MAGIC,1,int) 
#define OZ8806_IOCTL_SETLANG  		_IOW(OZ8806_MAGIC,2,int)
#define OZ8806_READ_INFO  			_IOR(OZ8806_MAGIC,3,int)
#define OZ8806_READ_CONFIG  		_IOR(OZ8806_MAGIC,4,int) 
#define OZ8806_READ_ADDR  			_IOR(OZ8806_MAGIC,5,int)
	

struct OZ8806_data 
{
	struct power_supply bat;
	struct power_supply ac;
	struct delayed_work work;
	struct work_struct dcwakeup_work;
	unsigned int interval;
	unsigned int dc_det_pin;

	struct i2c_client	*myclient;
	struct mutex		update_lock;

	u32					valid;
	unsigned long		last_updated;
	u8					control;
	u16					aout16;
};


/*****************************************************************************
* Global variables section - Exported
* add declaration of global variables that will be exported here
* e.g.
*	int8_t foo;
****************************************************************************/
static struct OZ8806_data *the_OZ8806;

static enum power_supply_property oz8806_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
	//POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	//POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	//POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
};

static enum power_supply_property oz8806_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};


/*-------------------------------------------------------------------------*/
/*****************************************************************************
* Description:
*		below funciont is used to operate mmap
* Parameters:
*		description for each argument, new argument starts at new line
* Return:
*		what does this function returned?
*****************************************************************************/
/*-------------------------------------------------------------------------*/

static int proc_read_phymem_addr(char *page, char **start, off_t off, int count)
{
        return sprintf(page, "%08lx\n", __pa(kernel_memaddr));
}

static int proc_read_phymem_size(char *page, char **start, off_t off, int count)
{
        return sprintf(page, "%lu\n", kernel_memsize);
}

static int __init mm_init(void)
{
		int * addr;
		struct page *page;
		int i;

		/*build proc dir "memshare"and two proc files: phymem_addr, phymem_size in the dir*/
        proc_memshare_dir = proc_mkdir(PROC_MEMSHARE_DIR, NULL);
       // create_proc_info_entry(PROC_MEMSHARE_PHYADDR, 0, proc_memshare_dir, proc_read_phymem_addr);
        //create_proc_info_entry(PROC_MEMSHARE_SIZE, 0, proc_memshare_dir, proc_read_phymem_size);

        /*alloc one page*/
        kernel_memaddr =__get_free_pages(GFP_KERNEL, PAGE_ORDER);
        if(!kernel_memaddr)
        {
                printk("Allocate memory failure!\n");
        }
        else
        {
        		page = virt_to_page(kernel_memaddr );
				for(i = 0;i < (1 << PAGE_ORDER);i++)
				{
					SetPageReserved(page);
					page++;
				}
                kernel_memsize = PAGES_NUMBER * PAGE_SIZE;
                printk("Allocate memory success!. The phy mem addr=%08lx, size=%lu\n", __pa(kernel_memaddr), kernel_memsize);
        }
		pa_memaddr = __pa(kernel_memaddr);
		addr  = (int *)(kernel_memaddr);
		//*addr = 0x11223344;
		printk("Allocate memory  addr is %d!\n",*addr);
        return 0;
}

static void __exit mm_close(void)
{
        printk("The content written by user is: %s\n", (unsigned char *) kernel_memaddr);
        ClearPageReserved(virt_to_page(kernel_memaddr));
        free_pages(kernel_memaddr, PAGE_ORDER);
        remove_proc_entry(PROC_MEMSHARE_PHYADDR, proc_memshare_dir);
        remove_proc_entry(PROC_MEMSHARE_SIZE, proc_memshare_dir);
        remove_proc_entry(PROC_MEMSHARE_DIR, NULL);

        return;
}


/*-------------------------------------------------------------------------*/
/*****************************************************************************
* Description:
*		below function is used to operate io_ctrl
* Parameters:
*		description for each argument, new argument starts at new line
* Return:
*		what does this function returned?
*****************************************************************************/
/*-------------------------------------------------------------------------*/

static int oz8806_reg_open(struct inode *inode, struct file *file)
{
	int i;
	int err;
	//printk("OZ8806_IOCTL_GETLANG\n");
	return 0;
}     

static long oz8806_reg_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	uint8_t data;
	char buf[100];
	int ret;

	//printk("cmd is %d  \n",cmd);
	//printk("arg is %d  \n",arg);

	switch(cmd)
	{
		case OZ8806_IOCTL_GETLANG:
		
			//printk("OZ8806_IOCTL_GETLANG\n");
			//afe_register_read_byte(arg, &data);
		return data;

		case OZ8806_IOCTL_SETLANG:
			
			//printk("OZ8806_IOCTL_SETLANG\n");
			copy_from_user(buf, arg, 100);  
			printk("%s",buf);
			//afe_register_write_byte(uint8_t index, uint8_t dat);
		return 0;

		case OZ8806_READ_INFO:
			
			//printk("OZ8806_READ_INFO\n");
			ret = copy_to_user((int *)arg, batt_info, sizeof(bmu_data_t)); 
		return 0;

		case OZ8806_READ_CONFIG:
			
			//printk("OZ8806_READ_CONFIG\n");
			//ret = copy_to_user((int *)arg, &config_data, sizeof(config_data_t)); 
		return 0;

		case OZ8806_READ_ADDR:
		
		//printk("OZ8806_READ_ADDR\n");
		ret = copy_to_user((int *)arg, &pa_memaddr, 4); 
		//printk("kernel_memaddr is %d\n",*(char *)kernel_memaddr);
		return 0;

		default:
			return -EINVAL;
	}
}

static int oz8806_reg_close(struct inode *inode, struct file *file)
{
	return 0;
}


static struct file_operations dev_fops = {
	.owner	=	THIS_MODULE,
	.unlocked_ioctl	=	oz8806_reg_ioctl,
	.open = oz8806_reg_open,
	.release = oz8806_reg_close,
};

static struct miscdevice misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &dev_fops,
};

static int __init dev_init(void)
{
	int ret;
	ret = misc_register(&misc);
	return ret;
}

static void __exit dev_exit(void)
{
	misc_deregister(&misc);
}




/*-------------------------------------------------------------------------*/
/*****************************************************************************
* Description:
*		below function is linux power section
* Parameters:
*		description for each argument, new argument starts at new line
* Return:
*		what does this function returned?
*****************************************************************************/
/*-------------------------------------------------------------------------*/

static int OZ8806_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE_DATA
				     | I2C_FUNC_SMBUS_READ_BYTE))
		return -ENODEV;

	strlcpy(info->type, MYDRIVER, I2C_NAME_SIZE);

	return 0;
}

static int oz8806_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	
	struct OZ8806_data *data = container_of(psy, struct OZ8806_data, bat);
	switch (psp) {
	
	case POWER_SUPPLY_PROP_STATUS:
		if(gpio_get_value(data->dc_det_pin))
			val->intval = 3; //0;	/*discharging*/
		else
			val->intval = 1;	/*charging*/
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = batt_info->fVolt;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;//batt_info.fRSOC<0 ? 0:1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = batt_info->fCurr;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = batt_info->fRSOC;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;

	default:
		return -EINVAL;
	}

	return ret;
}

static int oz8806_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	struct OZ8806_data *data = container_of(psy, struct OZ8806_data, ac);
	
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS){
			if(gpio_get_value(data->dc_det_pin))
				val->intval = 0;	/*discharging*/
			else
				val->intval = 1;	/*charging*/
		}
		break;
		
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static void oz8806_powersupply_init(struct OZ8806_data *data)
{
	data->bat.name = "battery";
	data->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	data->bat.properties = oz8806_battery_props;
	data->bat.num_properties = ARRAY_SIZE(oz8806_battery_props);
	data->bat.get_property = oz8806_battery_get_property;
	
	data->ac.name = "ac";
	data->ac.type = POWER_SUPPLY_TYPE_MAINS;
	data->ac.properties = oz8806_ac_props;
	data->ac.num_properties = ARRAY_SIZE(oz8806_ac_props);
	data->ac.get_property = oz8806_ac_get_property;
}

static int charge_ok_status = 0;
extern uint8_t charger_finish;
static int charge_ok_count = 0;

static struct wake_lock batt_wake_lock;
static int charge_status_lock = 0;

static void oz8806_battery_work(struct work_struct *work)
{
	struct OZ8806_data *data = container_of(work, struct OZ8806_data, work.work); 
	
	//you must add this code here for O2MICRO
	mutex_lock(&data->update_lock);
	bmu_polling_loop();
	mutex_unlock(&data->update_lock);

//printk("===1111==%s  %d  ,charge_ok_status = %d,charger_finish = %d ,dc status = %d, chg status = %d\n",
//	__FUNCTION__,__LINE__,charge_ok_status,charger_finish,gpio_get_value(data->dc_det_pin),gpio_get_value(RK30_PIN0_PA6));

	if(gpio_get_value(RK30_PIN0_PA6) && (!charge_ok_status) && !gpio_get_value(data->dc_det_pin))
	{
		charge_ok_count += 1;
		//printk("======%s  %d  ,charge_ok_count = %d\n",__FUNCTION__,__LINE__,charge_ok_count);
		if(3 == charge_ok_count){
			charger_finish	= 1;
			charge_ok_status = 1;
			charge_ok_count = 0;
		}
	}
	if(!gpio_get_value(RK30_PIN0_PA6) || gpio_get_value(data->dc_det_pin))
	{
		charge_ok_status = 0;
		charger_finish	 = 0;
		charge_ok_count = 0;
	}

//add charge wake lock 
	if( 0 == gpio_get_value(data->dc_det_pin)){
			if(0 == charge_status_lock ){			
				wake_lock(&batt_wake_lock);  //lock
				charge_status_lock = 1; 
			}
	}else{
			if(1 == charge_status_lock ){			
				wake_unlock(&batt_wake_lock);  //unlock
				charge_status_lock = 0; 
			}
	}

//printk("===2222==%s  %d  ,charge_ok_status = %d,charger_finish = %d ,dc status = %d, chg status = %d\n",
//	__FUNCTION__,__LINE__,charge_ok_status,charger_finish,gpio_get_value(data->dc_det_pin),gpio_get_value(RK30_PIN0_PA6));

	if((gpio_get_value(data->dc_det_pin) == 0) && (batt_info->fRSOC <= 0))
		 batt_info->fRSOC = 1;

	if((gpio_get_value(data->dc_det_pin) == 0) && (batt_info->fRSOC >= 99))
		 batt_info->fRSOC = 100;

	power_supply_changed(&data->bat);
	power_supply_changed(&data->ac);
	/* reschedule for the next time */
	schedule_delayed_work(&data->work, data->interval);
}

static irqreturn_t OZ8806_dc_wakeup(int irq, void *dev_id)
{   
    schedule_work(&the_OZ8806->dcwakeup_work);
    return IRQ_HANDLED;
}

static void OZ8806_dcdet_delaywork(struct work_struct *work)
{
    int ret;

	struct OZ8806_data *data = container_of(work, struct OZ8806_data, dcwakeup_work); 
    
	int irq      = gpio_to_irq(data->dc_det_pin);
    int irq_flag = gpio_get_value (data->dc_det_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
    
    rk28_send_wakeup_key();
    
    free_irq(irq, NULL);
    ret = request_irq(irq, OZ8806_dc_wakeup, irq_flag, "oz8806_dc_det", NULL);
	if (ret) {
		free_irq(irq, NULL);
	}
	
	power_supply_changed(&data->bat);
}

static int OZ8806_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret, irq, irq_flag;
	struct OZ8806_data *data;

	if (!(data = kzalloc(sizeof(struct OZ8806_data), GFP_KERNEL)))
		return -ENOMEM;

	//Note that mainboard definition file, ex: arch/arm/mach-msm/board-xxx.c, must has declared
	// static struct i2c_board_info xxx_i2c_devs[] __initdata = {....}
	// and it must add including this "I2C_BOARD_INFO("OZ8806", 0x2F)," 
	// otherwise, probe will occur error
	// string is matching with definition in OZ8806_id id table

	// Init real i2c_client 
	i2c_set_clientdata(client, data);

	the_OZ8806 = data;
	data->myclient = client;
	data->interval = msecs_to_jiffies(4 * 1000);
	data->dc_det_pin = RK30_PIN0_PB2; //RK30_PIN6_PA5;

	
 	wake_lock_init(&batt_wake_lock, WAKE_LOCK_SUSPEND, "batt_lock");	

	mutex_init(&data->update_lock);

	INIT_DELAYED_WORK(&data->work, oz8806_battery_work);

	// Init OZ8806 chip
	//you must add this code here for O2MICRO
	bmu_init_parameter(client);
	bmu_init_chip(&parameter_customer);


	printk("AAAA OZ8806 POWER MANAGEMENT DRIVER VERSION is %s\n",VERSION);

	oz8806_powersupply_init(data);
	ret = power_supply_register(&client->dev, &the_OZ8806->bat);
	if (ret) {
		printk(KERN_ERR "failed to register battery\n");
		return ret;
	}
	ret = power_supply_register(&client->dev, &the_OZ8806->ac);
	if (ret) {
		printk(KERN_ERR "failed to register ac\n");
		return ret;
	}

	printk("%s %d",__FUNCTION__,__LINE__);
	schedule_delayed_work(&data->work, data->interval);
	if (data->dc_det_pin != INVALID_GPIO)
	{
		ret = gpio_request(data->dc_det_pin, "oz8806_dc_det");
		if (ret != 0) {
			gpio_free(data->dc_det_pin);
			printk("fail to request dc_det_pin\n");
			return -EIO;
		}

		INIT_WORK(&data->dcwakeup_work, OZ8806_dcdet_delaywork);
		irq = gpio_to_irq(data->dc_det_pin);
	        
		irq_flag = gpio_get_value (data->dc_det_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
	    	ret = request_irq(irq, OZ8806_dc_wakeup, irq_flag, "oz8806_dc_det", NULL);
	    	if (ret) {
	    		printk("failed to request dc det irq\n");
				return -EIO;
	    	}
	    	enable_irq_wake(irq);	
	}
	
	return 0;					//return Ok
}

static int OZ8806_remove(struct i2c_client *client)
{
	struct OZ8806_data *data = i2c_get_clientdata(client);
	printk("yyyy OZ8806 remove");
	mutex_lock(&data->update_lock);
	bmu_power_down_chip();
	mutex_unlock(&data->update_lock);
	power_supply_unregister(&data->bat);
	power_supply_unregister(&data->ac);
	cancel_delayed_work(&data->work);
	gpio_free(data->dc_det_pin);
	kfree(data);

	return 0;
}

static void OZ8806_shutdown(struct i2c_client *client)
{
	struct OZ8806_data *data = i2c_get_clientdata(client);
	printk(" OZ8806 shutdown");
	mutex_lock(&data->update_lock);
	bmu_power_down_chip();
	mutex_unlock(&data->update_lock);
	power_supply_unregister(&data->bat);
	power_supply_unregister(&data->ac);
	cancel_delayed_work(&data->work);
	gpio_free(data->dc_det_pin);
	kfree(data);
}

static int OZ8806_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct OZ8806_data *data = i2c_get_clientdata(client);
	printk("yyyy OZ8806 suspend");
	mutex_lock(&data->update_lock);
	mutex_unlock(&data->update_lock);
	return 0;
}

static int OZ8806_resume(struct i2c_client *client)
{
	struct OZ8806_data *data = i2c_get_clientdata(client);
	u16			ADCValue;
	int				tmpfRSOC;
	int ret = 0;
	printk("yyyy OZ8806 resume");
	//you must add this code here for O2MICRO
	mutex_lock(&data->update_lock);
	bmu_wake_up_chip();
	mutex_unlock(&data->update_lock);

	schedule_delayed_work(&data->work, data->interval);
	return 0;
}


/*-------------------------------------------------------------------------*/

static const struct i2c_device_id OZ8806_id[] = {
	{ MYDRIVER, 0 },							//string, id??
	{ }
};
MODULE_DEVICE_TABLE(i2c, OZ8806_id);

static struct i2c_driver OZ8806_driver = {
	.driver = {
		.name	= MYDRIVER,
	},
	.probe			= OZ8806_probe,
	.remove			= OZ8806_remove,
	.resume			= OZ8806_resume,
	.shutdown		= OZ8806_shutdown,
	.id_table		= OZ8806_id,

	//auto-detection function
	//.class			= I2C_CLASS_HWMON,			// Nearest choice
	//.detect			= OZ8806_detect,
	//.address_data	= &addr_data,
};

/*-------------------------------------------------------------------------*/

static int __init OZ8806_init(void)
{
	return i2c_add_driver(&OZ8806_driver);
}

static void __exit OZ8806_exit(void)
{
	printk("uuuu OZ8806 remove");
	i2c_del_driver(&OZ8806_driver);
}

/*-------------------------------------------------------------------------*/

#define	DRIVER_NAME	(OZ8806_driver.driver.name)

MODULE_DESCRIPTION("OZ8806 Battery Monitor IC Driver");
MODULE_LICENSE("GPL");

//subsys_initcall_sync(OZ8806_init);
module_init(OZ8806_init);
module_exit(OZ8806_exit);

module_init(mm_init);
module_exit(mm_close);

module_init(dev_init);
module_exit(dev_exit);


