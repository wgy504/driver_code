#include <linux/module.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#define DRV_NAME "RAW-DEV"
#define RAW_MAJOR 50

struct raw_private {
	unsigned long mem_phy;
        void __iomem *mem_vir;
        unsigned long mem_size;
	unsigned int irq;
        struct cdev raw_cdev;
};

struct class *raw_class;

int raw_open(struct inode *inode, struct file *filp)
{
	struct raw_private *dev = container_of(inode->i_cdev, struct raw_private, raw_cdev);
	filp->private_data = dev;

	return 0;
}

int raw_release(struct inode *inode, struct file *filp)
{
	return 0;
}

ssize_t raw_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
//	int i;
	char *kbuf;
	struct raw_private *dev = filp->private_data;

	count = min(count, (size_t)(dev->mem_size - *f_pos));
	if (0 == count)
		return 0;
	printk("Read: count=%d, *f_pos=%lld\n", count, *f_pos);

	kbuf = (char *)kzalloc(count, GFP_KERNEL);
	if (NULL == kbuf)
		return -ENOMEM;

	/* copy from io */
	memcpy_fromio(kbuf, (dev->mem_vir + *f_pos), count);

//	for (i=0; i<count; i++)
//		kbuf[i] = ioread8(dev->mem_vir + *f_pos + i);
	
	if (copy_to_user(buf, kbuf, count)) {
		kfree(kbuf);
		return -EFAULT;
	}

	*f_pos += count;
	return count;
}


static struct file_operations raw_fops = {
	.owner = THIS_MODULE,
	.open = raw_open,
	.release = raw_release,
	.read = raw_read,
};

int __devinit raw_probe(struct platform_device *pdev)
{
	struct raw_private *priv;
	struct resource *res;
	dev_t dev_id;

	printk("RAW probe: dev->name = %s; dev->id = %d\n", pdev->name, pdev->id);
	
	/* allocate raw_private */
	priv = (struct raw_private *)kzalloc(sizeof(*priv), GFP_KERNEL);
	if (NULL == priv)
		return -ENOMEM;

	/* get iomem address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	printk("RAW phy mem 0x%x---0x%x\n", res->start, res->end);
	priv->mem_phy = res->start;
	priv->mem_size = (unsigned long)(res->end - res->start + 1);
	/* ioremap */
	priv->mem_vir = ioremap(priv->mem_phy, priv->mem_size);
	if (NULL == priv->mem_vir) {
		printk("Cannot ioremap 0x%lx.\n", priv->mem_phy);
		kfree(priv);
		return -1;
	}
	printk("RAW vir mem: 0x%p\n", priv->mem_vir);

	/* get irq number */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	priv->irq = res->start;
	printk("RAW irq %d---%d\n", res->start, res->end);

	/* register cdev */
	dev_id = MKDEV(RAW_MAJOR, pdev->id);
	cdev_init(&priv->raw_cdev, &raw_fops);
	priv->raw_cdev.owner = THIS_MODULE;
	cdev_add(&priv->raw_cdev, dev_id, 1);
	
	/* create class file */
	device_create(raw_class, /* class * */
			NULL,	/* parent */
			dev_id, /* dev num */
			NULL,	/* drvdata */
			"raw_dev%d", pdev->id); /* name */

	platform_set_drvdata(pdev, priv);
	return 0;
}

int __devexit raw_remove(struct platform_device *pdev)
{
	dev_t dev_id = MKDEV(RAW_MAJOR, pdev->id);
	struct raw_private *priv;
	priv = (struct raw_private *)platform_get_drvdata(pdev);

	cdev_del(&priv->raw_cdev);
	device_destroy(raw_class, dev_id);
	iounmap(priv->mem_vir);
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
	int ret;
	dev_t dev_id = MKDEV(RAW_MAJOR, 0);

	/* class create */
	raw_class = class_create(THIS_MODULE, "raw-class");
	if (NULL == raw_class) {
		printk("Cannot create class.\n");
		return -1;
	}

#if 1	
	printk("Register raw-dev driver.\n");
	/* request chrdev region */
	ret = register_chrdev_region(dev_id, 100, "RAW-dev");
	if (ret) {
		class_destroy(raw_class);
		printk("Cannot request chrdev region.\n");
		return -1;
	}
#endif
	ret = platform_driver_register(&raw_drv);
	if (ret) {
		class_destroy(raw_class);
		unregister_chrdev_region(dev_id, 100);
	};
	return ret;
}

void __exit my_exit(void)
{
	dev_t dev_id = MKDEV(RAW_MAJOR, 0);
	printk("Unregister raw-dev driver.\n");
	platform_driver_unregister(&raw_drv);
	unregister_chrdev_region(dev_id, 100);
	class_destroy(raw_class);
}


module_init(my_init);
module_exit(my_exit);

MODULE_LICENSE("GPL");

