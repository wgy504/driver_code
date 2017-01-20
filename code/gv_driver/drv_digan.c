/***********************************************************
 * 版权信息: 金溢科技版权所有，保留一切权利
 * 文件名称: drv_digan.c
 * 文件作者: huangrw, yuml
 * 完成日期: 2015-09-22
 * 当前版本: 1.0.0
 * 主要功能: 地感信号，RSU状态指示，串口通路现在驱动程序
 * 版本历史: 
 ***********************************************************/
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/irq.h>
#include <linux/swab.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <plat/gpmc.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <asm/delay.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/sched.h>

#define G_VER           "v1.01"
#define DEV_NAME        "gv_digan"
#define DEV_FILE_NAME   "DGmodule"
//
#define DEV_MAJOR       241

//平台设备私有数据结构，GPIO片内资源属于平台设备
struct drv_digan_io 
{
	int io_rsu1_status;
	int io_rsu2_status;

	int io_digan_in1;
	int io_digan_in2;

	int io_uart_select1;
	int io_uart_select2;
	int io_uart_select3;
};

//地感设备类别
static struct class *dg_class;

//驱动私有数据
struct gv_dev 
{
    char *name;
    int id;

    struct platform_device *pdev;
    struct cdev cdev;
    dev_t devno; //device node num
	int digan_irq1;
	int digan_irq2;

    wait_queue_head_t recv_wait; //等待列队，阻塞
    u32 recv_ready;//中断输出就绪变量

    spinlock_t recv_lock;

    struct mutex open_lock; //仅允许一次打开

	struct drv_digan_io *dio;
	u8 dg_val[2]; //1-car in, 0-car out
};

/*****************************************************************
 函数名称：gv_io_out
 函数描述：GPIO输出电平
 输入参数：ionum：管脚标号
		   val电平0,1
 输出参数：无
 返回说明：无
 其它说明：
 *****************************************************************/
static inline void gv_io_out(int ionum, int val)
{   
    gpio_set_value(ionum, val);
}

/*****************************************************************
 函数名称：gv_io_in
 函数描述：读GPIO的电平
 输入参数：无
 输出参数：无
 返回说明：电平或错误状态
 其它说明：
 *****************************************************************/
static inline int gv_io_in(int ionum)
{   
    return gpio_get_value(ionum);
}

/*****************************************************************
 函数名称：psam_ioctl
 函数描述：功能性IO操作接口
 输入参数：filp，文件信息结构
		   cmd，命令号
		   arg，选择性输入参数
 输出参数：arg，选择性输出结果
 返回说明：成功标识
 其它说明：
 *****************************************************************/
static long gv_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct gv_dev *dev = filp->private_data;
    int ret = 0;
	struct drv_digan_io *dio = dev->dio;

	switch (cmd)
	{
		case 2:
			if (arg == 1)
			{
				gv_io_out(dio->io_uart_select1, 1);
				gv_io_out(dio->io_uart_select2, 0);
				gv_io_out(dio->io_uart_select3, 0);
			}
			else if (arg == 2)
			{
				gv_io_out(dio->io_uart_select1, 0);
				gv_io_out(dio->io_uart_select2, 1);
				gv_io_out(dio->io_uart_select3, 0);
			}
			else if (arg == 3)
			{
				gv_io_out(dio->io_uart_select1, 0);
				gv_io_out(dio->io_uart_select2, 0);
				gv_io_out(dio->io_uart_select3, 1);
			}
			else
			{
				gv_io_out(dio->io_uart_select1, 0);
				gv_io_out(dio->io_uart_select2, 0);
				gv_io_out(dio->io_uart_select3, 0);				
			}			
			break;

		case 3:
			ret = copy_to_user((u8 *)arg, dev->dg_val, sizeof(dev->dg_val));
			break;

		case 4: //RSU1 STATUS
			if (arg)
			{
				gv_io_out(dio->io_rsu1_status, 1);
			}
			else
			{
				gv_io_out(dio->io_rsu1_status, 0);
			}

			break;

		case 5: //RSU1 STATUS		
			if (arg)
			{
				gv_io_out(dio->io_rsu2_status, 1);
			}
			else
			{
				gv_io_out(dio->io_rsu2_status, 0);
			}
			break;

		case 6:
			break;

		case 7:
			dev->recv_ready = 0;
			break;

		default:
			printk("drv_digan ioctl cmd %d not support\n", cmd);
			break;
	}


    return ret;
}

/*****************************************************************
 函数名称：gv_open
 函数描述：打开地感设备接口
 输入参数：inode,设备节点
		   filp，文件参数
 输出参数：无
 返回说明：成功标识
 其它说明：
 *****************************************************************/
static int gv_open(struct inode *inode, struct file *filp)
{
    struct gv_dev *dev = container_of(inode->i_cdev, struct gv_dev, cdev);

    unsigned long flags = 0;

    //mutex open    
    if (!mutex_trylock(&dev->open_lock))
    {
        printk("digan dev already open, return.\n");
        return -1;
    }

    spin_lock_irqsave(&dev->recv_lock, flags);
    filp->private_data = dev;   
    dev->recv_ready = 0;
    spin_unlock_irqrestore(&dev->recv_lock, flags); 

    return 0;
}

/*****************************************************************
 函数名称：gv_release
 函数描述：释放地感设备接口
 输入参数：inode,设备节点
		   filp，文件参数
 输出参数：无
 返回说明：成功标识
 其它说明：
 *****************************************************************/
static int gv_release(struct inode *inode, struct file *filp)
{
    struct gv_dev *dev = container_of(inode->i_cdev, struct gv_dev, cdev);
    
    filp->private_data = dev;
    mutex_unlock(&dev->open_lock);

    return 0;
}

/*****************************************************************
 函数名称：gv_poll
 函数描述：等待列队查询资源是否就绪, 系统调用select内部被使用
 输入参数：filp，文件参数
		   table,等待列队
 输出参数：无
 返回说明：> 0,资源可用
 其它说明：
 *****************************************************************/
static unsigned int gv_poll(struct file *filp, poll_table *wait)
{
    unsigned int mask;
    unsigned long flags;
    struct gv_dev *dev = filp->private_data;
    
    poll_wait(filp, &dev->recv_wait, wait);

    spin_lock_irqsave(&dev->recv_lock, flags);
    if (dev->recv_ready)
    {
        mask =   (POLLIN | POLLRDNORM);
    }
    else           
    {
        mask = 0;
    }
    spin_unlock_irqrestore(&dev->recv_lock, flags);

    return mask;
}

/*****************************************************************
 函数名称：gv_write
 函数描述：设备写接口
 输入参数：file，文件信息结构
		   buf，写数据缓存
		   count，写数据大小
		   f_pos，写入位置
 输出参数：f_pos，更新写入后位置
 返回说明：<0，写入失败，其他为写入成功数据大小
 其它说明：
 *****************************************************************/
static ssize_t gv_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *f_pos)
{
	return 0;
}

/*****************************************************************
 函数名称：gv_read
 函数描述：设备读接口
 输入参数：file，文件信息结构
		   count，读数据大小
		   f_pos，读位置
 输出参数：buf，读数据缓存
		   f_pos，更新读后的位置
 返回说明：<0，读失败，其他为读取成功数据大小
 其它说明：
 *****************************************************************/
static ssize_t gv_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    int ret = 0;
    struct gv_dev *dev = filp->private_data;

	while (dev->recv_ready == 0)
	{
		ret = wait_event_interruptible(dev->recv_wait, dev->recv_ready);
		if (ret < 0)
		{
			break; //interrupt by signal
		}
	}
	
	if (dev->recv_ready > 0)
    {	
		ret = copy_to_user(buf, dev->dg_val, sizeof(dev->dg_val));
		dev->recv_ready = 0;
		return sizeof(dev->dg_val);
	}

    return -1;	
}


//设备文件操作
static struct file_operations gv_fops = {
    .owner =    THIS_MODULE,
    .write =    gv_write,
    .read =     gv_read,
    .unlocked_ioctl = gv_ioctl,
    .open =     gv_open,
    .release =  gv_release,
    .llseek =   no_llseek,
    .poll   = gv_poll,
};

/*****************************************************************
 函数名称：cdev_setup
 函数描述：初始化字符设备并创建设备文件
 输入参数：pdata，设备私有数据
		major，字符设备主设备号
		minor，字符设备从设备号
		fops，字符设备文件操作
 输出参数：无
 返回说明：无
 其它说明：
 *****************************************************************/
static void cdev_setup(struct gv_dev *pdata, int major, int minor, struct file_operations *fops)
{
    struct cdev *cdev = &pdata->cdev;   
    struct device *dev;
    int err;
    dev_t devno = MKDEV(major, minor);

    cdev_init(cdev, fops);
    cdev->owner = THIS_MODULE;
    cdev->ops = fops;
    
    err = cdev_add(cdev, devno, 1);
    if (err)
    {
        printk("gpmc add cdev error\n");
    }   
    
    dev = device_create(dg_class, &pdata->pdev->dev, devno, pdata, "%s", pdata->name);
    if (!IS_ERR(dev))
    {
        pdata->devno = devno;       
    }
}

/*****************************************************************
 函数名称：cdev_release
 函数描述：注销字符设备并删除设备文件
 输入参数：dev，设备私有数据
 输出参数：无
 返回说明：无
 其它说明：
 *****************************************************************/
static void cdev_release(struct gv_dev *dev)
{
    cdev_del(&dev->cdev);

    if (dev->devno)
    {
        device_destroy(dg_class, dev->devno);
    }
}

/*****************************************************************
 函数名称：digan_irq_handler
 函数描述：地感信号中断服务
 输入参数：irq，中断编号
		   dev_id，中断私有数据
 输出参数：无
 返回说明：中断处理成功标识
 其它说明：
 *****************************************************************/
static irqreturn_t digan_irq_handler(int irq, void *dev_id)
{
    struct gv_dev  *dev = dev_id;
	//printk("get irq, %d, %d\n", gv_io_in(dev->dio->io_digan_in1), gv_io_in(dev->dio->io_digan_in2));
	if (irq == dev->digan_irq1)
	{
		if (gv_io_in(dev->dio->io_digan_in1) == 0)
		{
			dev->dg_val[0] = 1;
		}
		else
		{
			dev->dg_val[0] = 0;
		}
	}
	else
	{
		if (gv_io_in(dev->dio->io_digan_in2) == 0)
		{
			dev->dg_val[1] = 1;
		}
		else
		{
			dev->dg_val[1] = 0;
		}
	}

	dev->recv_ready = 1;
	wake_up_interruptible(&dev->recv_wait);

    return IRQ_HANDLED;
}

/*****************************************************************
 函数名称：gv_drv_probe
 函数描述：设备探测函数，设备与驱动配对后探测能否正常驱动
 输入参数：pdev，设备参数
 输出参数：无
 返回说明：成功标识
 其它说明：
 *****************************************************************/
static int __devinit gv_drv_probe(struct platform_device *pdev)
{
    int retval = -1;
    struct gv_dev  *pdata;

    int irq_flags = IORESOURCE_IRQ_LOWEDGE; //IORESOURCE_IRQ_HIGHEDGE; 

    printk("digan_drv_probe, pdev-id = %d...\n", pdev->id);


    pdata = kzalloc(sizeof(struct gv_dev), GFP_KERNEL);    
    dev_set_drvdata(&pdev->dev, pdata);
	pdata->dio = pdev->dev.platform_data;


    pdata->id = pdev->id;
    pdata->pdev = pdev;

    spin_lock_init(&pdata->recv_lock);
    init_waitqueue_head(&pdata->recv_wait);
    pdata->recv_ready = 0;

    mutex_init(&pdata->open_lock);


	if (pdata->dio->io_digan_in1 > 0)
	{
		pdata->digan_irq1 = gpio_to_irq(pdata->dio->io_digan_in1);
		retval = request_irq(pdata->digan_irq1, digan_irq_handler, irq_flags, "digan_irq1", pdata);
        if (retval) 
        {
            printk( "Unable to claim requested irq: %d\n", pdata->digan_irq1);
        }
		
		printk("io[%d] = %d, irq_no=%d\n", pdata->dio->io_digan_in1, gv_io_in(pdata->dio->io_digan_in1), pdata->digan_irq1);		
	}

	if (pdata->dio->io_digan_in2 > 0)
	{
		pdata->digan_irq2 = gpio_to_irq(pdata->dio->io_digan_in2);
		retval = request_irq(pdata->digan_irq2, digan_irq_handler, irq_flags, "digan_irq2", pdata);
        if (retval) 
        {
            printk( "Unable to claim requested irq: %d\n", pdata->digan_irq2);
        }

		printk("io[%d] = %d, irq_no=%d\n", pdata->dio->io_digan_in2, gv_io_in(pdata->dio->io_digan_in2), pdata->digan_irq2);		
	}


    pdata->name = DEV_FILE_NAME;
    cdev_setup(pdata, DEV_MAJOR, pdata->id, &gv_fops);

    return 0;
}

/*****************************************************************
 函数名称：gv_drv_remove
 函数描述：注销设备的资源
 输入参数：pdev，设备参数
 输出参数：无
 返回说明：无
 其它说明：
 *****************************************************************/
static int gv_drv_remove(struct platform_device *pdev)
{

    struct gv_dev  *pdata = dev_get_drvdata(&pdev->dev);
    printk("g90 fpga remove, id = %d\n", pdata->id);

	
	if (pdata->digan_irq1 > 0)
	{
		free_irq(pdata->digan_irq1, pdata);
	}

	if (pdata->digan_irq2 > 0)
	{
		free_irq(pdata->digan_irq2, pdata);
	}

    kfree(pdata);
    cdev_release(pdata);    

    return 0;
}

//GPIO片内资源，平台设备驱动
static struct platform_driver gv_driver = {
    .probe = gv_drv_probe,
    .remove = __devexit_p(gv_drv_remove),
    .driver = {
        .name   = DEV_NAME,
        .owner  = THIS_MODULE,
    },
};

/*****************************************************************
 函数名称：gv_init
 函数描述：模块加载时初始化
 输入参数：无
 输出参数：无
 返回说明：成功标识
 其它说明：
 *****************************************************************/
static int __init gv_init(void)
{
    printk("G90 gpmc init, driver version = %s.\n", G_VER);

    dg_class = class_create(THIS_MODULE, "dg_class");
    return platform_driver_register(&gv_driver);
}

/*****************************************************************
 函数名称：gv_exit
 函数描述：模块注销时释放资源
 输入参数：无
 输出参数：无
 返回说明：无
 其它说明：
 *****************************************************************/
static void __exit gv_exit(void)
{
    printk("G90 gpmc deinit.\n");	
    platform_driver_unregister(&gv_driver);
    class_destroy(dg_class);    
}



module_init(gv_init);
module_exit(gv_exit);

MODULE_LICENSE("GPL"); 
MODULE_AUTHOR("R.wen");
   
