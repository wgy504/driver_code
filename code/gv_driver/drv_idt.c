/***********************************************************
 * 版权信息: 金溢科技版权所有，保留一切权利
 * 文件名称: drv_idt.c
 * 文件作者: huangrw, yuml
 * 完成日期: 2015-09-22
 * 当前版本: 1.0.0
 * 主要功能: 双口RAM驱动，用于710单片机与am335x共享数据
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


#define G_VER           "v1.00"
#define DEV_NAME        "gv_idt"
#define DEV_FILE_NAME   "IDTmodule"

#define DEV_MAJOR       243

//平台设备私有数据，GPIO编号用于IO操作
struct gv_idt_io
{
	int io_busy_in;
	int io_rw_out;
	int io_oe;
	int io_rsu_select;
};

//设备类别
static struct class *idt_class;

//驱动私有数据
struct gv_dev 
{
    char *name;
    int id;

    void __iomem *ioaddr;

    struct platform_device *pdev;
    struct cdev cdev;
    dev_t devno; //device node num
	int irq;

    wait_queue_head_t recv_wait; //
    u32 recv_ready; //中断服务中修改，就绪条件变量

    spinlock_t recv_lock;

    struct mutex open_lock; //设备仅能打开一次

	struct gv_idt_io *dio;

	u8 recv_buf[1024];
	int rlen;
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
 函数名称：idt_read
 函数描述：从双口RAM读出数据
 输入参数：dev，设备私有数据
 输出参数：dev->recv_buf
 返回说明：数据大小
 其它说明：
 *****************************************************************/
static int idt_read(struct gv_dev *dev)
{	
	int len = 0;
    volatile unsigned char *offset  = 0;

	gv_io_out(dev->dio->io_rw_out, 1);  //read enable

	offset = ((volatile unsigned char *)dev->ioaddr) + 0x3ff;	
	len = offset[0];

	if (len > 0)	
	{
		int i;
		offset = ((volatile unsigned char *)dev->ioaddr) + 257;	

		for (i=0; i<len; i++)
		{
			dev->recv_buf[i] = offset[i];
		}
	}
	
	return len; 
}

/*****************************************************************
 函数名称：idt_write
 函数描述：向双口RAM写入数据
 输入参数：dev，设备私有数据
		   buf，写数据缓存
		   len，写数据大小
 返回说明：<0，写入失败，其他为写入成功数据大小
 其它说明：
 *****************************************************************/
static int idt_write(struct gv_dev *dev, u8 *buf, int len)
{
	int i=0;
    volatile unsigned char *offset  = 0;
	
	if (len > 256)
	{
		printk("idt send buf too long, len = %d\n", len);
		return -1;
	}
	
	gv_io_out(dev->dio->io_rw_out, 0); //write enable

	offset = ((volatile unsigned char *)dev->ioaddr) + 1;
	for (i=0; i<len; i++)
	{
		offset[i] = buf[i];
	}


	offset = ((volatile unsigned char *)dev->ioaddr) + 0x3fe;
	offset[0] = len;

	udelay(100);
	gv_io_out(dev->dio->io_rw_out, 1);  //read enable

	return len;
}

/*****************************************************************
 函数名称：gv_ioctl
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
	struct gv_idt_io *dio = dev->dio;

	switch (cmd)
	{
		case 0:
			if (arg == 2)
			{
				gv_io_out(dio->io_rsu_select, 0);
			}
			else
			{
				gv_io_out(dio->io_rsu_select, 1);		
			}			
			break;

		default:
			printk("drv_idt ioctl cmd %d not support\n", cmd);
			return -1;
	}

    return 0;
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
        printk("idt dev already open, return.\n");
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
 函数名称：cal_bcc
 函数描述：计算数据的BCC校验
 输入参数：buf，数据
		   len，数据大小
 输出参数：无
 返回说明：BCC值
 其它说明：
 *****************************************************************/
static u8 cal_bcc(u8 *buf, int len)
{
	u8 bcc = 0;	
	int i = 0;

	for (i=0; i<len; i++)
	{
		bcc ^= buf[i];
	}
	
	return bcc;
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
	int ret = 0;
	u8 sbuf[256];
    struct gv_dev *dev = filp->private_data;
	unsigned long flags = 0;
	
	if (count > 126)
	{
		printk("idt send buf too long, len = %d\n", count);
		return -1;
	}

	spin_lock_irqsave(&dev->recv_lock, flags);
	dev->recv_ready = 0;
	ret = copy_from_user(sbuf, buf, count);
	sbuf[count] = cal_bcc(sbuf, count);

	ret = idt_write(dev, sbuf, count+1);

    spin_unlock_irqrestore(&dev->recv_lock, flags);

	return ret;
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
	unsigned long flags = 0;

	while (dev->recv_ready == 0)
	{
		ret = wait_event_interruptible(dev->recv_wait, dev->recv_ready);
		if (ret < 0)
		{
			break; //interrupt by signal
		}
	}
	
	//printk("idt read...\n");

	spin_lock_irqsave(&dev->recv_lock, flags);
	
	if ((dev->recv_ready > 0) && (dev->rlen > 0))
    {	
		ret = copy_to_user(buf, dev->recv_buf, dev->rlen);
		dev->recv_ready = 0;

	    spin_unlock_irqrestore(&dev->recv_lock, flags);

		//printk("idt read len=%d\n", dev->rlen);
		return dev->rlen;
	}

    spin_unlock_irqrestore(&dev->recv_lock, flags);
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
    
    dev = device_create(idt_class, &pdata->pdev->dev, devno, pdata, "%s", pdata->name);
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
        device_destroy(idt_class, dev->devno);
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
static irqreturn_t idt_irq_handler(int irq, void *dev_id)
{
    struct gv_dev  *dev = dev_id;
	unsigned long flags = 0;

	spin_lock_irqsave(&dev->recv_lock, flags);
	dev->rlen = idt_read(dev);

	dev->recv_ready = 1;
	wake_up_interruptible(&dev->recv_wait);

	//printk("idt irq rlen = %d\n", dev->rlen);

    spin_unlock_irqrestore(&dev->recv_lock, flags);

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
    int res_size;
    struct resource *res, *irq_res;

    int irq_flags = IORESOURCE_IRQ_HIGHEDGE; //IORESOURCE_IRQ_LOWEDGE;

    pdata = kzalloc(sizeof(struct gv_dev), GFP_KERNEL);    
    dev_set_drvdata(&pdev->dev, pdata);
	pdata->dio = pdev->dev.platform_data;

    pdata->id = pdev->id;
    pdata->pdev = pdev;

    printk("idt_drv_probe, pdev-id = %d, rw_io=%d...\n", pdev->id, pdata->dio->io_rw_out);

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        printk("platform_get_resource idx = %d error\n", 0);
        return -1;
    }
    res_size = resource_size(res);

    pdata->ioaddr = ioremap_nocache(res->start, res_size);
    if (pdata->ioaddr == NULL) {
        printk("ioremap error\n");
        return -2;
    }

    irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (!irq_res) {
        printk("drv_idt Could not allocate irq resource.\n");
        return -1;
    }

    pdata->irq = irq_res->start;
    irq_flags = irq_res->flags & IRQF_TRIGGER_MASK;

    retval = request_irq(pdata->irq, idt_irq_handler, irq_flags, "idt_irq", pdata);			
    
    if (retval) 
    {
        printk( "Unable to claim requested irq: %d\n", pdata->irq);
    }

    printk("id = %d, start = %08x, size = %d, ioaddr = %08x, irq = %d\n", pdev->id, res->start, res_size, (unsigned int)pdata->ioaddr, pdata->irq);

	gv_io_out(pdata->dio->io_oe, 0); 

    spin_lock_init(&pdata->recv_lock);
    init_waitqueue_head(&pdata->recv_wait);
    pdata->recv_ready = 0;

    mutex_init(&pdata->open_lock);

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

    free_irq(pdata->irq, pdata);
    iounmap(pdata->ioaddr);

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

    idt_class = class_create(THIS_MODULE, "idt_class");
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
    class_destroy(idt_class);    
}


module_init(gv_init);
module_exit(gv_exit);

MODULE_LICENSE("GPL"); 
MODULE_AUTHOR("R.wen");
   
