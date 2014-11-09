/***************************************************************************
* Copyright (C) 2007 by trem (Philippe Reynes) *
* tremyfr@yahoo.fr *
* *
* This program is free software; you can redistribute it and/or modify *
* it under the terms of the GNU General Public License as published by *
* the Free Software Foundation; either version 2 of the License, or *
* (at your option) any later version. *
* *
* This program is distributed in the hope that it will be useful, *
* but WITHOUT ANY WARRANTY; without even the implied warranty of *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the *
* GNU General Public License for more details. *
* *
* You should have received a copy of the GNU General Public License *
* along with this program; if not, write to the *
* Free Software Foundation, Inc., *
* 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA. *
***************************************************************************/
/**
* This kernel driver demonstrates how an RTDM device can be called from
* a RT task and how to use a semaphore to create a blocking device operation.
*
* It is a simple device, only 4 operation are provided:
* - open: start device usage
* - close: ends device usage
* - write: store transfered data in an internal buffer (realtime context)
* - read: return previously stored data and erase buffer (realtime context)
*
*/
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <rtdm/rtdm_driver.h>

#define BUFF_SIZE_MAX 1024
#define DEVICE_NAME "rt_spi"
#define DRIVER_NAME "rt_spi"
#define DEVICE_ID 0
#define SOME_SUB_CLASS 4711

//define for SPI 
#define USER_BUFF_SIZE 128
#define SPI_BUS 1
#define SPI_BUS_CS 1		//for CS1
#define SPI_MODE SPI_MODE_0
#define SPI_BUS_SPEED 1000000

/* Function prototype declarations */
//static void rt_spi_cleanup_ctx(struct rt_spi_dev *ctx);
static int SPI_rtdm_open(struct rtdm_dev_context *context, rtdm_user_info_t * user_info, int oflags);
static int SPI_rtdm_close(struct rtdm_dev_context *context, rtdm_user_info_t * user_info);
static ssize_t SPI_rtdm_read_rt(struct rtdm_dev_context *context, rtdm_user_info_t * user_info, void *buf, size_t nbyte);
static int rt_spi_queue_read(void);
static void rt_spi_read_completion_handler(void *arg);
static ssize_t SPI_rtdm_write_rt(struct rtdm_dev_context *context, rtdm_user_info_t * user_info, const void *buf, size_t nbyte);
static int rt_spi_queue_write(void);
static void rt_spi_write_completion_handler(void *arg);
static int rt_spi_probe(struct spi_device *spi_device);
static int rt_spi_remove(struct spi_device *spi_device);
static int add_rt_spi_device_to_bus(void);


/**
* The structure of the buffer
*
*/
typedef struct buffer_s {
	int size;
	u8 data[BUFF_SIZE_MAX];
} buffer_t;
/**
* The global buffer
*
*/
buffer_t buffer;
/**
* The global semaphore
*
*/
rtdm_sem_t sem;

/**
* This structure describe the SPI data control
*
*/
struct rt_spi_control {
	struct spi_message msg;
	struct spi_transfer transfer;
	u32 busy;
	u32 spi_callbacks;
	u32 busy_counter;
	u8 *tx_buff; 
	u8 *rx_buff;
	int buff_size;
};
static struct rt_spi_control rt_spi_control;

/**
* This structure describe the SPI device
*
*/
struct rt_spi_dev {
	//struct semaphore spi_sem;
	//struct semaphore fop_sem;
	spinlock_t spi_lock;
	dev_t devt;
	//struct cdev cdev;
	//struct class *class;
	struct spi_device *spi_device;
	//struct hrtimer timer;
	//u32 timer_period_sec;
	//u32 timer_period_ns;
	//u32 running;
	char *user_buff;
	
	rtdm_irq_t irq_handle;		/* device IRQ handle */
	rtdm_lock_t lock;		/* lock to protect context struct */
	unsigned long base_addr;	/* hardware IO base address */
	rtdm_event_t event;		/* raised to unblock reader or writter*/
	rtdm_mutex_t out_lock;		/* single-writer mutex */
};
static struct rt_spi_dev rt_spi_dev;

/**
* This structure describe the simple RTDM device
*
*/
static struct rtdm_device rtdm_spi_dev = {
	.struct_version = RTDM_DEVICE_STRUCT_VER,
	.device_flags = RTDM_NAMED_DEVICE,
	.context_size = sizeof(struct rt_spi_dev),
	.device_name = DEVICE_NAME,
	.device_id = DEVICE_ID,
	.open_nrt = SPI_rtdm_open, //the driver allows only RT application if open_rt and open_nrt is the same
	.open_rt = SPI_rtdm_open,
	.ops = {
		.close_nrt = SPI_rtdm_close,
		.close_rt = SPI_rtdm_close,
		.read_rt = SPI_rtdm_read_rt,
		.write_rt = SPI_rtdm_write_rt,
	},
	.device_class = RTDM_CLASS_EXPERIMENTAL,
	.device_sub_class = SOME_SUB_CLASS,
	.profile_version = 1,
	.driver_name = DRIVER_NAME,
	.driver_version = RTDM_DRIVER_VER(0, 1, 2),
	.peripheral_name = DRIVER_NAME,
	.proc_name = DEVICE_NAME,
	.provider_name = "RT_MaG_Team",
};

/**
* This structure describe the SPI driver
*
*/
static struct spi_driver rt_spi_driver = {
        .driver = {
                .name = DRIVER_NAME,
                .owner = THIS_MODULE,
        },
        .probe = rt_spi_probe,
        .remove = rt_spi_remove,
};

static u8 *buf_global;
static rtdm_user_info_t *user_info_global;


/*static void rt_spi_cleanup_ctx(struct rt_spi_dev *ctx)
{
	rtdm_event_destroy(&ctx->event);
}*/

/**
* Open the device
*
* This function is called when the device shall be opened.
*
*/
static int SPI_rtdm_open(struct rtdm_dev_context *context, rtdm_user_info_t * user_info, int oflags)
{
	struct rt_spi_dev *spi_ctx;
	int dev_id = context->device->device_id;
	//int err;
	//rtdm_lockctx_t lock_ctx;

        rtdm_printk("Begin of SPI_rtdm_open() for device %i\n",dev_id);
	
	spi_ctx = (struct rt_spi_dev *)context->dev_private;

	/* IPC initialisation - cannot fail with used parameters */
	rtdm_lock_init(&spi_ctx->lock);
	rtdm_event_init(&spi_ctx->event, 0);
	
	/* Part copied from Guillaume i2c driver */ // DON'T KNOW WHAT WE NEED!
	/*ctx->base_addr = (unsigned long)mapped_io[dev_id]; // base_addr needed??

	ctx->status = 0; // status needed??

	rt_i2c_set_config(ctx, &default_config); // SPI config already done in __init?? 
	
	err = rtdm_irq_request(&ctx->irq_handle, irq[dev_id], rt_i2c_interrupt, irqtype[dev_id], context->device->proc_name, ctx);  // TO PUT OR NOT TO PUT??
	if (err)
	{
		rtdm_printk("rtdm_irq_request err\n");
		rt_i2c_cleanup_ctx(ctx);

		return err;
	}*/
	
	/* enable interrupts */  // NEEDED??
	/*rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

	writew(0x7fff,ctx->base_addr + I2C_STAT);
	//writew(BF|NACK|ARDY|XRDY,ctx->base_addr + I2C_IE);
        //writew(BF|ARDY|RRDY,ctx->base_addr + I2C_IE);
        writew(0x7fff,ctx->base_addr + I2C_IE);
        writew(BF|ARDY|RRDY|AERR,ctx->base_addr + I2C_IE);
        writew(0,ctx->base_addr + I2C_CNT);

	rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);*/

	rtdm_printk("End of SPI_rtdm_open()\n");
	
	return 0;
}
/**
* Close the device
*
* This function is called when the device shall be closed.
*
*/
static int SPI_rtdm_close(struct rtdm_dev_context *context, rtdm_user_info_t * user_info)
{
	struct rt_spi_dev *spi_ctx;
	int dev_id = context->device->device_id;
	int err;
	rtdm_lockctx_t lock_ctx;

        rtdm_printk("Begin of SPI_rtdm_close() for device %i\n",dev_id);
	
	spi_ctx = (struct rt_spi_dev *)context->dev_private;
	
	err = rt_spi_remove(spi_ctx->spi_device); // not sure if correct...
	
	/* Part copied from Guillaume i2c driver */ // DON'T KNOW WHAT WE NEED!
	/*unsigned long base = ctx->base_addr;

	rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

	// mask all I2C interrupts and clear pending ones. 
	writew(0,base + I2C_IE);
	writew(0x7fff,base + I2C_STAT);

	rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

	rtdm_irq_free(&ctx->irq_handle);*/

	//rt_spi_cleanup_ctx(spi_ctx);

	rtdm_printk("End of SPI_rtdm_close()\n");
	
	return 0;
}

/**
* Read from the device
*
* This function is called when the device is read in realtime context.
*
*/
static ssize_t SPI_rtdm_read_rt(struct rtdm_dev_context *context, rtdm_user_info_t * user_info, void *buf, size_t nbyte)
{
	int ret, status;
	
	/* take the semaphore */
	rtdm_sem_down(&sem);
	
	if (rt_spi_control.busy == 0) {
		
		/* write the user buffer in the kernel buffer */
		//buffer.size = (nbyte > BUFF_SIZE_MAX) ? BUFF_SIZE_MAX : nbyte; 
		if (nbyte > BUFF_SIZE_MAX) {
			buffer.size = BUFF_SIZE_MAX;
			rtdm_printk("Number of bytes requested higher than BUFF_SIZE_MAX: buffer.size = BUFF_SIZE_MAX!\n");
		}
		else {
			buffer.size = nbyte;
		}
	
		/* rx_buff point to buffer.data */	
		rt_spi_control.rx_buff = &buffer.data;
		rt_spi_control.buff_size = buffer.size;

		/* user_info_global,buf_global point to user_info,buf */	
		user_info_global = user_info;
		buf_global = buf;
		
		/* call spi_message_add_tail() and spi_async() */
		status = rt_spi_queue_read();
	
	}
	else {
		rtdm_printk("SPI read/write not possible: previous data still in queue!\n");
	}
	
	/* release the semaphore */
	rtdm_sem_up(&sem);
	
	return status;
}

/**
*
* This function is called to read data to buffer (call spi_message_add_tail() and spi_async())
*
*/
static int rt_spi_queue_read(void)
{
	int status;
	unsigned long flags;
 
	spi_message_init(&rt_spi_control.msg);
 
	/* this gets called when the spi_message completes */
	rt_spi_control.msg.complete = rt_spi_read_completion_handler;
	rt_spi_control.msg.context = NULL;
 
	rt_spi_control.transfer.tx_buf = NULL;
	rt_spi_control.transfer.rx_buf = rt_spi_control.rx_buff;
	rt_spi_control.transfer.len = rt_spi_control.buff_size;
 
	spi_message_add_tail(&rt_spi_control.transfer, &rt_spi_control.msg);
 
	spin_lock_irqsave(&rt_spi_dev.spi_lock, flags); // CHANGE TO rtdm_irqsave??
	//rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
 
	if (rt_spi_dev.spi_device)
		status = spi_async(rt_spi_dev.spi_device, &rt_spi_control.msg);
	else
		status = -ENODEV;
 
	spin_unlock_irqrestore(&rt_spi_dev.spi_lock, flags); // CHANGE TO rtdm_irqrestore??
	//rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
	
	if (status == 0)
		rt_spi_control.busy = 1;
	
	return status;	
}

/**
*
* Callback function when read is completed.
*
*/
static void rt_spi_read_completion_handler(void *arg)
{	
	int ret;

	rtdm_printk("SPI read completed\n");
	
	ret = rtdm_safe_copy_to_user(user_info_global, buf_global, buffer.data, buffer.size);
	
	/* if an error has occured, send it to user */
	if (ret)
		rtdm_printk("Error during copy_to_user!\n");
		
	/* clean the kernel buffer */
	buffer.size = 0;

	rt_spi_control.spi_callbacks++;
	rt_spi_control.busy = 0;
}

/**
* Write in the device
*
* This function is called when the device is written in realtime context.
*
*/
static ssize_t SPI_rtdm_write_rt(struct rtdm_dev_context *context, rtdm_user_info_t * user_info, const void *buf, size_t nbyte)
{
	int ret, status;
	
	/* take the semaphore */
	rtdm_sem_down(&sem);
	
	if (rt_spi_control.busy == 0) {
	
		/* write the user buffer in the kernel buffer */
		//buffer.size = (nbyte > BUFF_SIZE_MAX) ? BUFF_SIZE_MAX : nbyte; 
		if (nbyte > BUFF_SIZE_MAX) {
			buffer.size = BUFF_SIZE_MAX;
			rtdm_printk("Number of bytes requested higher than BUFF_SIZE_MAX: buffer.size = BUFF_SIZE_MAX!\n");
		}
		else {
			buffer.size = nbyte;
		}
	
		ret = rtdm_safe_copy_from_user(user_info, buffer.data, buf, buffer.size);
		
		/* if an error has occured, send it to user */
		if (ret)
			return ret;
		
		/* make tx_buff point to buffer.data */	
		rt_spi_control.tx_buff = &buffer.data;
		rt_spi_control.buff_size = buffer.size;
		
		/* call spi_message_add_tail() and spi_async() */
		status = rt_spi_queue_write();
	}
	else {
		rtdm_printk("SPI read/write not possible: previous data still in queue!\n");
	}
	
	/* release the semaphore */
	rtdm_sem_up(&sem);
	
	return status;
}

/**
*
* This function is called to write data to buffer (call spi_message_add_tail() and spi_async())
*
*/
static int rt_spi_queue_write(void)
{
	int status;
	unsigned long flags;
 
	spi_message_init(&rt_spi_control.msg);
 
	/* this gets called when the spi_message completes */
	rt_spi_control.msg.complete = rt_spi_write_completion_handler;
	rt_spi_control.msg.context = NULL;
 
	rt_spi_control.transfer.tx_buf = rt_spi_control.tx_buff;
	rt_spi_control.transfer.rx_buf = NULL;
	rt_spi_control.transfer.len = rt_spi_control.buff_size;
 
	spi_message_add_tail(&rt_spi_control.transfer, &rt_spi_control.msg);
 
	spin_lock_irqsave(&rt_spi_dev.spi_lock, flags); // CHANGE TO rtdm_irqsave??
	//rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
 
	if (rt_spi_dev.spi_device)
		status = spi_async(rt_spi_dev.spi_device, &rt_spi_control.msg);
	else
		status = -ENODEV;
 
	spin_unlock_irqrestore(&rt_spi_dev.spi_lock, flags); // CHANGE TO rtdm_irqrestore??
	//rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
	
	if (status == 0)
		rt_spi_control.busy = 1;
	
	return status;	
}

/**
*
* Callback function when write is completed.
*
*/
static void rt_spi_write_completion_handler(void *arg)
{	
	rtdm_printk("SPI write completed\n");
	// DO SOMETHING ELSE??

	rt_spi_control.spi_callbacks++;
	rt_spi_control.busy = 0;
}

/**
*
* This function is called to probe the device (activate the module)
*
*/
static int rt_spi_probe(struct spi_device *spi_device)
{
	if (rtdm_sem_down(&sem)) //if (down_interruptible(&rt_spi_dev.spi_sem))
		return -EBUSY;

	rt_spi_dev.spi_device = spi_device;

	rtdm_sem_up(&sem); //up(&rt_spi_dev.spi_sem);

	return 0;
}

/**
*
* This function is called to remove the device
*
*/
static int rt_spi_remove(struct spi_device *spi_device)
{
	if (rtdm_sem_down(&sem)) //if (down_interruptible(&rt_spi_dev.spi_sem))
		return -EBUSY;
	
	rt_spi_dev.spi_device = NULL;

	rtdm_sem_up(&sem); //up(&rt_spi_dev.spi_sem);

	return 0;
}

/**
*
* This function is called to initialize the SPI bus
*
*/
static int add_rt_spi_device_to_bus(void)
{
	struct spi_master *spi_master;			//définition des structures spi_master dans SPI.h
	struct spi_device *spi_device;			//idem
	struct device *pdev;					//Doit-on remplacer ça par struct rtdm_device?
	char buff[64];	//buffer for string debug 
	int status = 0;
	 
	spi_master = spi_busnum_to_master(SPI_BUS);			// Define déclaré ici
	if (!spi_master) {
		rtdm_printk(KERN_ALERT "spi_busnum_to_master(%d) returned NULL\n",SPI_BUS);
		rtdm_printk(KERN_ALERT "Missing modprobe omap2_mcspi?\n");		//=> retour dans linux puisque utilisation de omap2_mcspi
		return -1;
	}
	 
	spi_device = spi_alloc_device(spi_master);
	if (!spi_device) {
		put_device(&spi_master->dev);
		rtdm_printk(KERN_ALERT "spi_alloc_device() failed\n");
		return -1;
	}
	 
	/* specify a chip select line */
	spi_device->chip_select = SPI_BUS_CS;
	 
	/* Check whether this SPI bus.cs is already claimed */
	snprintf(buff, sizeof(buff), "%s.%u",dev_name(&spi_device->master->dev),spi_device->chip_select);
	 
	pdev = bus_find_device_by_name(spi_device->dev.bus, NULL, buff);
	if (pdev) {
	/* We are not going to use this spi_device, so free it */
		spi_dev_put(spi_device);
	/*
	* There is already a device configured for this bus.cs combination.
	* It's okay if it's us. This happens if we previously loaded then
	* unloaded our driver.
	* If it is not us, we complain and fail.
	*/
		if (pdev->driver && pdev->driver->name && strcmp(rt_spi_driver.driver.name, pdev->driver->name)) {
			rtdm_printk(KERN_ALERT "Driver [%s] already registered for %s\n",pdev->driver->name, buff);
			status = -1;
		}
	} 
	else {
		spi_device->max_speed_hz = SPI_BUS_SPEED;
		spi_device->mode = SPI_MODE;
		spi_device->bits_per_word = 16;
		spi_device->irq = -1;
		spi_device->controller_state = NULL;
		spi_device->controller_data = NULL;
		strlcpy(spi_device->modalias, rt_spi_driver.driver.name, SPI_NAME_SIZE);
		status = spi_add_device(spi_device);
		if (status < 0) {
			spi_dev_put(spi_device);
			rtdm_printk(KERN_ALERT "spi_add_device() failed: %d\n",status);
		}
	}
	 
	put_device(&spi_master->dev);
	
	return status;
}

/**
* This function is called when the module is loaded
*
* It simply registers the RTDM device.
*
*/
int __init SPI_rtdm_init(void)
{
	int status, error;
	
	buffer.size = 0;	/* clear the data buffer */	// rtdm function						
	rtdm_sem_init(&sem, 0);	/* init the global semaphore */	 // rtdm function			

	error = spi_register_driver(&rt_spi_driver);
	if (error < 0) {
		rtdm_printk(KERN_ALERT "spi_register_driver() failed %d\n", error);
		return error;
	}

	error = add_rt_spi_device_to_bus();
	if (error < 0) {
		rtdm_printk(KERN_ALERT "add_rt_spi_to_bus() failed\n");
		spi_unregister_driver(&rt_spi_driver);
		return error;
	}
	
	status = rtdm_dev_register(&rtdm_spi_dev); // rtdm function // PUT IT AT BEGINNING??
	 
	return status;
}
module_init(SPI_rtdm_init);

/**
* This function is called when the module is unloaded
*
* It unregister the RTDM device, polling at 1000 ms for pending users.
*
*/
void __exit SPI_rtdm_exit(void)
{
	spi_unregister_device(rt_spi_dev.spi_device);
	spi_unregister_driver(&rt_spi_driver);

	//device_destroy(rt_spi_dev.class, rt_spi_dev.devt);
	//class_destroy(rt_spi_dev.class);

	//cdev_del(&rt_spi_dev.cdev);
	//unregister_chrdev_region(rt_spi_dev.devt, 1);

	if (rt_spi_dev.user_buff)
		kfree(rt_spi_dev.user_buff);

	rtdm_dev_unregister(&rtdm_spi_dev, 1000); // rtdm function
}
module_exit(SPI_rtdm_exit);



MODULE_DESCRIPTION("RTDM SPI module");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("RT-MaG Team");
MODULE_VERSION("0.1");
