#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/slab.h>

static struct proc_dir_entry *reg_proc_entry;

int reg_proc_write(struct file *file, const char __user *buff, size_t count, loff_t *offp)
{
	int ret = -1, i = 0;
	char *buf = kmalloc(count, GFP_KERNEL);
	char *data = buf;
	unsigned int regs_val = 0, read_val = 0;
	ret = copy_from_user((void*)buf, buff, count);
	
	while(1) {
		data = strstr(data, "0x");
		if(data == NULL)
			goto reg_proc_write_exit;
		sscanf(data, "0x%x", &regs_val);
		ssd_set_register(regs_val);
		read_val = ssd_read_register(regs_val >> 16);
		regs_val &= 0xffff;
		if(read_val != regs_val)
			printk("%s fail:0x%04x\n", __func__, read_val);	
		data += 3;
	}

reg_proc_write_exit:		
	kfree(buf);	
	msleep(10);
 	return count;
}

int reg_proc_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
#if 0	
	int ret = -1;
	const char buf[32] = {0};
	unsigned int regs_val = 0;
	ret = copy_from_user((void*)buf, buff, count);
	sscanf(buf, "0x%x", &regs_val);
	regs_val = ssd_read_register(regs_val);
	sprintf(buf, "0x%04x\n", regs_val);
	copy_to_user(buff, buf, 4);
	
	printk("%s:%04x\n", __func__, regs_val);
	msleep(10);
#endif	
	return count;
}

int reg_proc_open(struct inode *inode, struct file *file)
{
	//printk("%s\n", __func__);
	//msleep(10);
	return 0;
}

int reg_proc_close(struct inode *inode, struct file *file)
{
	//printk("%s\n", __func__);
	//msleep(10);
	return 0;   
}

struct file_operations reg_proc_fops = {
	.owner = THIS_MODULE,
	.open = reg_proc_open,
	.release = reg_proc_close,
	.write = reg_proc_write,
	.read = reg_proc_read,
};

static int reg_proc_init(char *name)
{
	int ret = 0;
  	reg_proc_entry = create_proc_entry(name, 0666, NULL);
	if(reg_proc_entry == NULL) {
		printk("Couldn't create proc entry : %s!\n", name);
		ret = -ENOMEM;
		return ret ;
	}
	else {
		printk("Create proc entry:%s success!\n", name);
		reg_proc_entry->proc_fops = &reg_proc_fops;
	}
	
	return 0;
}


