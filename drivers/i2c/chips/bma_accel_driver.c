/*  $Date: 2009/11/10 17:37:35 $
 *  $Revision: 1.0 $ 
 */

/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2010 Bosch Sensortec GmbH
 * All Rights Reserved
 */

/*! \file BMA023_driver.c
    \brief This file contains all function implementations for the BMA023 in linux
    
    Details.
*/


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <asm/uaccess.h>
#include <linux/unistd.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <mach/gpio.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

#include <linux/i2c/bma220.h>
#include <linux/i2c/bma023_dev.h>

//#define DEBUG 1	
#define BMA_DEBUG

#define ACC_DEV_MAJOR 241


/* i2c operation for bma API */
static char bma220_i2c_write(unsigned char sla, unsigned char reg_addr, unsigned char *data, unsigned char len);
static char bma220_i2c_read(unsigned char sla, unsigned char reg_addr, unsigned char *data, unsigned char len);
static void bma220_i2c_delay(unsigned int msec);

static char i2c_acc_bma023_read(u8 reg, u8 *val, unsigned int len );
static char i2c_acc_bma023_write( u8 reg, u8 *val );

enum BMA_SENSORS  
{
	BMA220 = 0,
	BMA023,
};

/* globe variant */
static struct i2c_client *bma_client = NULL;
static char			sensor_type = -1;		
struct class *acc_class;
static int 			calibration = 0 ;
struct bma_data {
	union{
		bma220_t			bma220;
		bma023_t			bma023;
	};
};

struct device *bma_dev_t;
EXPORT_SYMBOL(bma_dev_t);
/*************************************************************************/
/*						BMA220 I2C_API						   */
/*************************************************************************/
/*	i2c delay routine for eeprom	*/
static inline void bma220_i2c_delay(unsigned int msec)
{
	mdelay(msec);
}

/*	i2c write routine for bma	*/
static inline char bma220_i2c_write(unsigned char sla, unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	int dummy;
	int i=0;
	unsigned char addr;
	if( bma_client == NULL )	/*	No global client pointer?	*/
		return -1;
	addr = reg_addr<<1;		/*bma220 i2c addr left shift*/
	dummy = i2c_smbus_write_byte_data(bma_client, addr, data[0]);

	while(i<len)
	{
		addr = reg_addr<<1;		/*bma220 i2c addr left shift*/
		dummy = i2c_smbus_write_byte_data(bma_client, addr, data[0]);
		reg_addr++;
		data++;
		i++;		
		if(dummy<0)
			return -1;	
	}

	return 0;
}

/*	i2c read routine for bma220	*/
static inline char bma220_i2c_read(unsigned char sla, unsigned char reg_addr, unsigned char *data, unsigned char len) 
{
	int dummy=0;
	int i=0;
	unsigned char addr;
	if( bma_client == NULL )	/*	No global client pointer?	*/
		return -1;
	while(i<len)
	{        
		addr = reg_addr<<1;	/*bma220 i2c addr left shift*/
		dummy = i2c_smbus_read_word_data(bma_client, addr);
		data[i] = dummy & 0x00ff;
		i++;
		reg_addr++;
		dummy = len;
	}
	return 0;
}

/*************************************************************************/
/*						BMA023 I2C_API						   */
/*************************************************************************/
#define I2C_M_WR				0x00

static char i2c_acc_bma023_read(u8 reg, u8 *val, unsigned int len )
{
#if 0
	int 	 err;
	struct 	 i2c_msg msg[1];
	
	unsigned char data[1];
	trace_in() ;
	if( (bma_client == NULL) || (!bma_client->adapter) )
	{
		printk( "ERROR 1\n" ) ;
		return -ENODEV;
	}
	
	msg->addr 	= bma_client->addr;
	msg->flags 	= I2C_M_WR;
	msg->len 	= 1;
	msg->buf 	= data;
	*data       = reg;

	err = i2c_transfer(bma_client->adapter, msg, 1);
	debug( "MGS 2, err= %d\n", err ) ;

	if (err >= 0) 
	{
		msg->flags = I2C_M_RD;
		msg->len   = len;
		msg->buf   = val;
		err = i2c_transfer(bma_client->adapter, msg, 1);
    	debug( "MSG 3, err= %d\n", err ) ;
	}

	if (err >= 0) 
	{
    	debug( "ERROR 4, err= %d\n", err ) ;
		trace_out() ;
		return 0;
	}
	printk("%s %d i2c transfer error\n", __func__, __LINE__);/* add by inter.park */

	trace_out(); 
	return err;
#else
	int dummy=0;
	int i=0;
	if( bma_client == NULL )	/*	No global client pointer?	*/
		return -1;
	while(i<len)
	{		
		dummy = i2c_smbus_read_word_data(bma_client, reg);
		val[i] = dummy & 0x00ff;
		i++;
		reg++;
		dummy = len;
	}
	return 0;
#endif
}

static char i2c_acc_bma023_write( u8 reg, u8 *val )
{
#if 0
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];
	trace_in() ;


	if( (bma_client == NULL) || (!bma_client->adapter) ){
	    trace_out(); 
	    return -ENODEV;
	}

	data[0] = reg;
	data[1] = *val;

	msg->addr = bma_client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 2;
	msg->buf = data;

	err = i2c_transfer(bma_client->adapter, msg, 1);

	if (err >= 0) trace_out(); 
	return 0;

	printk("%s %d i2c transfer error\n", __func__, __LINE__);/* add by inter.park */
	trace_out(); 
	return err;
#else
	int dummy;
	int i=0;
	unsigned char len = 1;
	if( bma_client == NULL )	/*	No global client pointer?	*/
		return -1;	
	dummy = i2c_smbus_write_byte_data(bma_client, reg, val[0]);

	while(i<len)
	{
		dummy = i2c_smbus_write_byte_data(bma_client, reg, val[0]);
		reg++;
		val++;
		i++;		
		if(dummy<0)
			return -1;	
	}

	return 0;
#endif
}

/*************************************************************************/
/*						BMA023 Sysfs						   */
/*************************************************************************/
//TEST
static ssize_t bma_fs_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = 0;
	unsigned char data[6] = {0,0,0,0,0,0};	
	
	if(sensor_type == BMA220)
	{
		bma220_read_accel_xyz((bma220acc_t*)data);
		//printk("x: %d,y: %d,z: %d\n", ((bma220acc_t*)data)->x, ((bma220acc_t*)data)->y, ((bma220acc_t*)data)->z);
		count = sprintf(buf,"%d,%d,%d\n", ((bma220acc_t*)data)->x, ((bma220acc_t*)data)->y, ((bma220acc_t*)data)->z );
	}
	else if(sensor_type == BMA023)
	{
		bma023_read_accel_xyz((bma023acc_t*)data);	
		//printk("x: %d,y: %d,z: %d\n", ((bma023acc_t*)data)->x, ((bma023acc_t*)data)->y, ((bma023acc_t*)data)->z);
		count = sprintf(buf,"%d,%d,%d\n", ((bma023acc_t*)data)->x, ((bma023acc_t*)data)->y, ((bma023acc_t*)data)->z );
	}    

	return count;
}

static ssize_t bma_fs_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	//buf[size]=0;
	printk("input data --> %s\n", buf);

	return size;
}

static DEVICE_ATTR(acc_file, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, bma_fs_read, bma_fs_write);


/*	read command for BMA device file	*/
static ssize_t bma_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{	
	bma220acc_t acc;	
	int ret;
	if( bma_client == NULL )
	{
#ifdef BMA_DEBUG
		printk(KERN_INFO "I2C driver not install\n");
#endif
		return -1;
	}

	bma220_read_accel_xyz(&acc);
//#ifdef BMA_DEBUG
	printk(KERN_INFO "BMA Accel: X/Y/Z axis: %-8d %-8d %-8d\n" ,
		(int)acc.x, (int)acc.y, (int)acc.z);  
//#endif

	if( count != sizeof(acc) )
	{
		return -1;
	}
	ret = copy_to_user(buf,&acc, sizeof(acc));
	if( ret != 0 )
	{
#ifdef BMA_DEBUG
	printk(KERN_INFO "BMA Accel: copy_to_user result: %d\n", ret);
#endif
	}
	return sizeof(acc);
}

/*	write command for BMA Accel device file	*/
static ssize_t bma_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	if( bma_client == NULL )
		return -1;
#ifdef BMA_DEBUG
	printk(KERN_INFO "BMA Accel should be accessed with ioctl command\n");
#endif
	return 0;
}

/*	open command for BMA accel device file	*/
static int bma_open(struct inode *inode, struct file *file)
{
#ifdef BMA_DEBUG
		printk(KERN_INFO "%s\n",__FUNCTION__); 
#endif

	if( bma_client == NULL)
	{
#ifdef BMA_DEBUG
		printk(KERN_INFO "I2C driver not install\n"); 
#endif
		return -1;
	}

#ifdef BMA_DEBUG
	printk(KERN_INFO "BMA Accel has been opened\n");
#endif
	return 0;
}

/*	release command for BMA accel device file	*/
static int bma_close(struct inode *inode, struct file *file)
{
#ifdef BMA_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);	
#endif
	return 0;
}


/*	ioctl command for BMA accel device file	*/
static int bma_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned char data[6];

	/* check cmd */
	if(_IOC_TYPE(cmd) != BMA220_IOC_MAGIC)	
	{
#ifdef BMA_DEBUG
		printk("cmd magic type error\n");
#endif
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > BMA220_IOC_MAXNR)
	{
#ifdef BMA_DEBUG
		printk("cmd number error\n");
#endif
		return -ENOTTY;
	}

	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	
	if(err)
	{
#ifdef BMA_DEBUG
		printk("cmd access_ok error\n");
#endif
		return -EFAULT;
	}
	/* check bam120_client */
	if( bma_client == NULL)
	{
#ifdef BMA_DEBUG
		printk("I2C driver not install\n"); 
#endif
		return -EFAULT;
	}
	
	/* cmd mapping */

	switch(cmd)
	{
	case BMA220_SOFT_RESET:
		err = bma220_soft_reset();
		return err;

	case BMA220_SET_SUSPEND:
		err = bma220_set_suspend();
		return err;

	case BMA220_SET_OFFSET_TARGET_X:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_offset_target_x(*data);
		return err;

	case BMA220_SET_OFFSET_TARGET_Y:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_offset_target_y(*data);
		return err;

	case BMA220_SET_OFFSET_TARGET_Z:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_offset_target_z(*data);
		return err;

	case BMA220_SET_RANGE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_range(*data);
		return err;

	case BMA220_GET_RANGE:
		err = bma220_get_range(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA220_SET_MODE:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_mode(*data);
		return err;

	case BMA220_GET_MODE:
		err = bma220_get_mode(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA220_SET_BANDWIDTH:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_bandwidth(*data);
		return err;

	case BMA220_GET_BANDWIDTH:
		err = bma220_get_bandwidth(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA220_SET_LOW_TH:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_low_th(*data);
		return err;

	case BMA220_SET_LOW_DUR:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_low_dur(*data);
		return err;

	case BMA220_SET_HIGH_TH:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_high_th(*data);
		return err;

	case BMA220_SET_HIGH_DUR:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_high_dur(*data);
		return err;

	case BMA220_RESET_INTERRUPT:
		err = bma220_reset_int();
		return err;

	case BMA220_READ_ACCEL_X:
		err = bma220_read_accel_x((signed char*)data);
		if(copy_to_user((signed char*)arg,(signed char*)data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA220_READ_ACCEL_Y:
		err = bma220_read_accel_y((signed char*)data);
		if(copy_to_user((signed char*)arg,(signed char*)data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA220_READ_ACCEL_Z:
		err = bma220_read_accel_z((signed char*)data);
		if(copy_to_user((signed char*)arg,(signed char*)data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA220_SET_EN_LOW:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_en_low(*data);
		return err;

	case BMA220_SET_EN_HIGH_XYZ:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_en_high_xyz(*data);
		return err;

	case BMA220_SET_LATCH_INT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_latch_int(*data);
		return err;

	case BMA220_SET_LOW_HY:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_low_hy(*data);
		return err;

	case BMA220_SET_HIGH_HY:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_high_hy(*data);
		return err;

	case BMA220_READ_ACCEL_XYZ:
		if(sensor_type == BMA220)
		{
			err = bma220_read_accel_xyz((bma220acc_t*)data);
			if(copy_to_user((bma220acc_t*)arg,(bma220acc_t*)data,3)!=0)
			{
#ifdef BMA_DEBUG
				printk("copy_to error\n");
#endif
				return -EFAULT;
			}
		}else if(sensor_type == BMA023)
		{
			err = bma023_read_accel_xyz((bma023acc_t*)data);
			//printk("[%s] read accel x = %d, y = %d, z = %d\n", __func__, ((bma023acc_t*)data)->x, ((bma023acc_t*)data)->y, ((bma023acc_t*)data)->z);
			if(copy_to_user((bma023acc_t*)arg,(bma023acc_t*)data,3*sizeof(short))!=0)
			{
#ifdef BMA_DEBUG
				printk("copy_to error\n");
#endif
				return -EFAULT;
			}
		}
		
		return err;

    case BMA220_GET_OFFSET_XYZ:
		err = bma220_get_offset_xyz((bma220acc_t*)data);
		if(copy_to_user((bma220acc_t*)arg,(bma220acc_t*)data,3)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_SET_OFFSET_XYZ:
		if(copy_from_user((bma220acc_t*)data,(bma220acc_t*)arg,3)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
        err = bma220_set_offset_xyz(*(bma220acc_t *)data);
		return err;

	
	case BMA220_SET_SLEEP_EN:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_sleep_en(*data);
		return err;

	case BMA220_SET_SC_FILT_CONFIG:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_sc_filt_config(*data);
		return err;

	case BMA220_SET_SERIAL_HIGH_BW:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_serial_high_bw(*data);
		return err;

	case BMA220_SET_EN_ORIENT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_en_orient(*data);
		return err;

	case BMA220_SET_ORIENT_EX:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_orient_ex(*data);
		return err;

	case BMA220_SET_ORIENT_BLOCKING:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_orient_blocking(*data);
		return err;

	case BMA220_SET_EN_TT_XYZ:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_en_tt_xyz(*data);
		return err;

	case BMA220_SET_TT_TH:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_tt_th(*data);
		return err;

	case BMA220_SET_TT_DUR:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_tt_dur(*data);
		return err;

	case BMA220_SET_TT_FILT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_tt_filt(*data);
		return err;

	case BMA220_SET_EN_SLOPE_XYZ:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_en_slope_xyz(*data);
		return err;

	case BMA220_SET_EN_DATA:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_en_data(*data);
		return err;

	case BMA220_SET_SLOPE_TH:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_slope_th(*data);
		return err;

	case BMA220_SET_SLOPE_DUR:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_slope_dur(*data);
		return err;

	case BMA220_SET_SLOPE_FILT:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_slope_filt(*data);
		return err;

	case BMA220_SET_CAL_TRIGGER:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_cal_trigger(*data);
		return err;

	case BMA220_SET_HP_XYZ_EN:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_hp_xyz_en(*data);
		return err;

	case BMA220_SET_SLEEP_DUR:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_sleep_dur(*data);
		return err;

	case BMA220_SET_OFFSET_RESET:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_offset_reset(*data);
		return err;

	case BMA220_SET_CUT_OFF_SPEED:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_cut_off_speed(*data);
		return err;

	case BMA220_SET_CAL_MANUAL:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
    	err = bma220_set_cal_manual(*data);
		return err;

	case BMA220_SET_SBIST:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_sbist(*data);
		return err;

	case BMA220_SET_INTERRUPT_REGISTER:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_interrupt_register(*data);
		return err;

	case BMA220_SET_DIRECTION_INTERRUPT_REGISTER:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_direction_interrupt_register(*data);
		return err;
	
	case BMA220_GET_DIRECTION_STATUS_REGISTER:
		err = bma220_get_direction_status_register(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_INTERRUPT_STATUS_REGISTER:
		err = bma220_get_interrupt_status_register(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_ORIENTATION:
		err = bma220_get_orientation(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_ORIENT_INT:
		err = bma220_get_orient_int(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_CHIP_ID:
		err = bma220_get_chip_id(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_SC_FILT_CONFIG:
		err = bma220_get_sc_filt_config(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_SLEEP_EN:
		err = bma220_get_sleep_en(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_SERIAL_HIGH_BW:
		err = bma220_get_serial_high_bw(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_LATCH_INT:
		err = bma220_get_latch_int(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_EN_DATA:
		err = bma220_get_en_data(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_EN_HIGH_XYZ:
		err = bma220_get_en_high_xyz(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_HIGH_TH:
		err = bma220_get_high_th(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_HIGH_HY:
		err = bma220_get_high_hy(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_HIGH_DUR:
		err = bma220_get_high_g_dur(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_EN_LOW:
		err = bma220_get_en_low(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_LOW_TH:
		err = bma220_get_low_th(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_LOW_HY:
		err = bma220_get_low_hy(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_LOW_DUR:
		err = bma220_get_low_g_dur(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_EN_ORIENT:
		err = bma220_get_en_orient(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_ORIENT_EX:
		err = bma220_get_orient_ex(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_ORIENT_BLOCKING:
		err = bma220_get_orient_blocking(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_EN_TT_XYZ:
		err = bma220_get_en_tt_xyz(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_TT_TH:
		err = bma220_get_tt_th(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_TT_DUR:
		err = bma220_get_tt_dur(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_TT_FILT:
		err = bma220_get_tt_filt(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_SET_TT_SAMP:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_tt_samp(*data);
		return err;

    case BMA220_SET_TIP_EN:
		if(copy_from_user(data,(unsigned char*)arg,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_from_user error\n");
#endif
			return -EFAULT;
		}
		err = bma220_set_tip_en(*data);
		return err;

    case BMA220_GET_TT_SAMP:
		err = bma220_get_tt_samp(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_TIP_EN:
		err = bma220_get_tip_en(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_EN_SLOPE_XYZ:
		err = bma220_get_en_slope_xyz(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_SLOPE_TH:
		err = bma220_get_slope_th(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_SLOPE_DUR:
		err = bma220_get_slope_dur(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_SLOPE_FILT:
		err = bma220_get_slope_filt(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_HP_XYZ_EN:
		err = bma220_get_hp_xyz_en(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_OFFSET_TARGET_X:
		err = bma220_get_offset_target_x(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_OFFSET_TARGET_Y:
		err = bma220_get_offset_target_y(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_OFFSET_TARGET_Z:
		err = bma220_get_offset_target_z(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_CUT_OFF_SPEED:
		err = bma220_get_cut_off_speed(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

    case BMA220_GET_CAL_MANUAL:
		err = bma220_get_cal_manual(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_CAL_RDY:
		err = bma220_get_cal_rdy(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_SLEEP_DUR:
		err = bma220_get_sleep_dur(data);
		if(copy_to_user((unsigned char*)arg,data,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	case BMA220_GET_SENSOR_TYPE:		
		printk("[%s] Get Sensor Type = %d\n", __func__, sensor_type);
		if(copy_to_user((char*)arg,&sensor_type,1)!=0)
		{
#ifdef BMA_DEBUG
			printk("copy_to_user error\n");
#endif
			return -EFAULT;
		}
		return err;

	default:
		return 0;
	}
}

static const struct file_operations bma_fops = {
	.owner = THIS_MODULE,
	.read = bma_read,
	.write = bma_write,
	.open = bma_open,
	.release = bma_close,
	.ioctl = bma_ioctl,
};

static struct miscdevice bma_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bma_accel",
	.fops = &bma_fops,
};

static int bma_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
#ifdef BMA_DEBUG
	printk(KERN_INFO "%s\n", __FUNCTION__);
#endif
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	strlcpy(info->type, "bma", I2C_NAME_SIZE);

	return 0;
}

static int bma_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err = 0;
	int tempvalue;
	struct bma_data *data;
#ifdef BMA_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_INFO "i2c_check_functionality error\n");
		goto exit;
	}
	data = kzalloc(sizeof(struct bma_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}		

	printk("[%s] slave addr = %x\n", __func__, client->addr);
	
	/* read chip id */
	tempvalue = 0;
#if 1//BMA220_SMBUS
	tempvalue = i2c_smbus_read_word_data(client, 0x00);
#else
	i2c_master_send(client, (char*)&tempvalue, 1);
	i2c_master_recv(client, (char*)&tempvalue, 1);
#endif
	if((tempvalue&0x0007) == 0x0002)
	{
		printk(KERN_INFO "Bosch Sensortec Device detected!\nBMA023 registered I2C driver!\n");
		bma_client = client;
		sensor_type = BMA023;
	}
	else
	{
		client->addr = BMA220_I2C_ADDR;
		tempvalue = 0;

		tempvalue = i2c_smbus_read_word_data(client, 0x00);
 		
		if((tempvalue&0x00FF) == 0x00dd)
		{
			printk(KERN_INFO "Bosch Sensortec Device detected!\nBMA220 registered I2C driver!\n");			
			bma_client = client;
			sensor_type= BMA220;
		}
		else
		{		
			printk(KERN_ERR "Bosch Sensortec Device not found, i2c error %d \n", tempvalue);
	//		i2c_detach_client(client);				
			bma_client = NULL;
			err = -1;
			goto kfree_exit;
		}
	}
	
	i2c_set_clientdata(bma_client, data);
	err = misc_register(&bma_device);
	if (err) {
		printk(KERN_ERR "bma accel device register failed\n");
		goto kfree_exit;
	}
	printk(KERN_INFO "bma accel device create ok\n");

	if(sensor_type == BMA220)
	{
		/* bma220 sensor initial */
		data->bma220.bus_write = bma220_i2c_write;
		data->bma220.bus_read = bma220_i2c_read;
		data->bma220.delay_msec = bma220_i2c_delay;	

		bma220_init(&(data->bma220));
		bma220_set_bandwidth(2); //bandwidth 250Hz
		bma220_set_range(0);	//range +/- 2G

		/* register interrupt */
		bma220_set_en_tt_xyz(0);
		bma220_set_en_slope_xyz(0);
		bma220_set_en_high_xyz(0);
		bma220_set_en_low(0);
		bma220_set_en_orient(0);
		bma220_reset_int();
	}
	else if(sensor_type == BMA023)
	{
		data->bma023.bma023_bus_write = i2c_acc_bma023_write;
		data->bma023.bma023_bus_read  = i2c_acc_bma023_read;

		/*call init function to set read write functions, read registers */
		bma023_init( &(data->bma023) );

		/* from this point everything is prepared for sensor communication */


		/* set range to 2G mode, other constants: 
		 * 	   			4G: bma023_RANGE_4G, 
		 * 	    		8G: bma023_RANGE_8G */

		bma023_set_range(bma023_RANGE_2G); 

		/* set bandwidth to 25 HZ */
		bma023_set_bandwidth(bma023_BW_25HZ);
	}

	return 0;

//	misc_deregister(&bma_device);
kfree_exit:
	kfree(data);
exit:
	return err;
}


static int bma_remove(struct i2c_client *client)
{
	struct bma_data *data = i2c_get_clientdata(client);
#ifdef BMA_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif	
	misc_deregister(&bma_device);

	//i2c_detach_client(client);
	kfree(data);
	bma_client = NULL;
	return 0;
}

#ifdef CONFIG_PM
static int bma_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret = 0;;

	if(sensor_type == BMA220)
	{
		if((ret = bma220_set_mode(2)) != 0)	// 2: suspend mode
			printk(KERN_ERR "[%s] Change to Suspend Mode is failed\n",__FUNCTION__);
		
	}
	else if(sensor_type == BMA023)
	{
		bma023_set_operation_mode( ACCEL, STANDBY, calibration ) ;
	}

#ifdef BMA_DEBUG
	printk(KERN_INFO "[%s] bma220 !!suspend mode!!\n",__FUNCTION__);
#endif

	return 0;
}

static int bma_resume(struct i2c_client *client)
{
	int ret = 0;

	if(sensor_type == BMA220)
	{
		if((ret = bma220_set_mode(0)) != 0)	// Normal mode
			printk(KERN_ERR "[%s] Change to Normal Mode is failed\n",__FUNCTION__);
	}
	else if(sensor_type == BMA023)
	{
		bma023_set_operation_mode( ACCEL, ONLYACCEL, calibration ) ;
	}

#ifdef BMA_DEBUG
	printk(KERN_INFO "[%s] bma220 !!resume mode!!\n",__FUNCTION__);
#endif

	return 0;
}
#else
#define bma_suspend NULL
#define bma_resume NULL
#endif

static unsigned short normal_i2c[] = { I2C_CLIENT_END};
I2C_CLIENT_INSMOD_1(bma_accel);

static const struct i2c_device_id bma_id[] = {
	{ "bma_accel", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma_id);

static struct i2c_driver bma_accel_driver = {	
	.driver = {
		.owner	= THIS_MODULE,	
		.name	= "bma_accel",
	},
	.class		= I2C_CLASS_HWMON,
	.id_table		= bma_id,
	.address_data	= &addr_data,
	.probe		= bma_probe,
	.remove		= bma_remove,
	.detect		= bma_detect,
	.suspend		= bma_suspend,
	.resume		= bma_resume,
};

static int __init BMA_init(void)
{

#ifdef BMA_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif

	acc_class = class_create(THIS_MODULE, "accelerometer");

	if (IS_ERR(acc_class)) 
		return PTR_ERR( acc_class );

	bma_dev_t = device_create( acc_class, NULL, MKDEV(ACC_DEV_MAJOR, 0), "%s", "accelerometer");
	
	if (IS_ERR(bma_dev_t)) 
	{
		return PTR_ERR(bma_dev_t);
	}

	if (device_create_file(bma_dev_t, &dev_attr_acc_file) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_acc_file.attr.name);

	return i2c_add_driver(&bma_accel_driver);
}

static void __exit BMA_exit(void)
{
	i2c_del_driver(&bma_accel_driver);
	printk(KERN_ERR "BMA_ACCEL exit\n");

	class_destroy( acc_class );
}



module_init(BMA_init);
module_exit(BMA_exit);

