#include <linux/module.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#define DRV_NAME "RAW-DEV"

struct raw_private {
	unsigned long mem_phy;
        void __iomem *mem_vir;
        unsigned int mem_size;
	unsigned int irq;
        struct cdev mycdev;
};

int raw_open(struct inode *inode, struct file *filp)
{}

int raw_release(struct inode *inode, struct file *filp)
{}

ssize_t raw_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{}

static struct file_operations raw_fops = {
	.owner = THIS_MODULE,
	.open = raw_open,
	.release = raw_release,
	.read = raw_read,
};


int __devinit raw_probe(struct platform_device *pdev)
{
	struct raw_private *priv;
	struct timeval tval;
	struct resource *res;

	do_gettimeofday(&tval);
	printk("RAW-DEV probe in %ld:%ld us.\n", tval.tv_sec, tval.tv_usec);
	printk("dev->name = %s; dev->id = %d\n", pdev->name, pdev->id);
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	printk("RAW mem 0x%x---0x%x\n", res->start, res->end);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	printk("RAW irq 0x%x---0x%x\n", res->start, res->end);

	priv = (struct raw_private *)kzalloc(sizeof(*priv), GFP_KERNEL);
	if (NULL == priv)
		return -ENOMEM;
	printk("Allocate priv in 0x%p\n", priv);

	platform_set_drvdata(pdev, priv);

	return 0;
}

int __devexit raw_remove(struct platform_device *pdev)
{
	struct raw_private *priv;
	struct timeval tval;

	priv = (struct raw_private *)platform_get_drvdata(pdev);
	do_gettimeofday(&tval);
	printk("RAW remove in %ld:%ld us.\n", tval.tv_sec, tval.tv_usec);
	printk("Remove priv 0x%p\n", priv);
	kfree(priv);

	return 0;
}


struct platform_driver raw_drv = {
	.probe = raw_probe,
	.remove = raw_remove,
	.driver = {
		.name = DRV_NAME,
	},
};

int __init my_init(void)
{
	printk("Register raw-dev driver.\n");
	return platform_driver_register(&raw_drv);
}

void __exit my_exit(void)
{
	printk("Unregister raw-dev driver.\n");
	platform_driver_unregister(&raw_drv);
}

module_init(my_init);
module_exit(my_exit);

MODULE_LICENSE("GPL");

