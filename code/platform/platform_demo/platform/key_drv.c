#include <linux/module.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>

#define KEY_NAME "key-i8042"
#define KEY_BUF	 4000
#define KEY_MAJOR 50
#define CHAR_COUNT 100

#define PIPE 'z'
#define PIPE_RESET _IO(PIPE, 1)
#define PIPE_RESIZE _IOW(PIPE, 2, unsigned long)

static int major = KEY_MAJOR;
module_param(major, int, 0444);


struct key_private {
	char *start, *end;
	char *wp, *rp;
	unsigned long buf_len;
	char tmp[10];
	unsigned int kpos;

	unsigned int kio;
	void __iomem *kvir;
	unsigned int kirq;
	dev_t knum;

	wait_queue_head_t kreadq;
	struct work_struct kwork;
	struct semaphore ksem;
	spinlock_t kspin;
	struct cdev kcdev;
};


size_t spacefree(struct key_private *dev)
{
	if (dev->rp == dev->wp)
		return (size_t)(dev->end - dev->start - 1);
	if (dev->rp < dev->wp)
		return (size_t)(dev->buf_len + dev->rp - dev->wp - 1);
	else // (dev->rp > dev->wp)
		return (size_t)(dev->rp - dev->wp - 1);
}


/* key service bottom half */
void key_workq(struct work_struct *data)
{
	struct key_private *kpriv = container_of(data, struct key_private, kwork);
	char *bottom_buf;
	size_t count = 0;
	unsigned long flags; 

	/* copy key value from kpriv->tmp */
	spin_lock_irqsave(&kpriv->kspin, flags);
	count = kpriv->kpos;
	bottom_buf = (char *)kzalloc(count, GFP_ATOMIC);
	if (NULL == bottom_buf) {
	   	spin_unlock_irqrestore(&kpriv->kspin, flags);
		return;
	}
	memcpy(bottom_buf, kpriv->tmp, count);
	kpriv->kpos = 0;
	spin_unlock_irqrestore(&kpriv->kspin, flags);

	/* copy bottom_buf to kpriv->wp */
	down(&kpriv->ksem);

	/* if buffer is full, drop the key data */
	if (spacefree(kpriv) == 0) {
		up(&kpriv->ksem);
		kfree(bottom_half);
		printk("Key buffer full, time is %ld\n", jiffies);
		return;
	}

	/* copy data to kpriv->wp */
	count = min(count, spacefree(kpriv));
	if (kpriv->rp > kpriv->wp)
		count = min(count, (size_t)(kpriv->rp - kpriv->wp -1));
	if (kpriv->rp <= kpriv->wp)
		count = min(count, (size_t)(kpriv->end - kpriv->wp));
	memcpy(kpriv->wp, bottom_buf, count);
	
	/* update kpriv->wp */
	kpriv->wp += count;
	if (kpriv->wp == kpriv->end)
		kpriv->wp = kpriv->start;

	up(&kpriv->ksem);
	wake_up(&kpriv->kreadq);
}


/* key service upper half */
irqreturn_t key_service(int irq, void *data)
{
	struct key_private *kpriv = data;
	
	/* read key value, save to bottom buffer */
	spin_lock(&kpriv->kspin);
	kpriv->tmp[kpriv->kpos++] = ioread8(kpriv->kvir);
	spin_unlock(&kpriv->kspin);

	/* raise bottom half */
	schedule_work(&kpriv->kwork);

	return IRQ_HANDLED;
}


int key_open(struct inode *inode, struct file *filp)
{
	struct key_private *kpriv = container_of(inode->i_cdev, struct key_private, 
kcdev);
	filp->private_data = kpriv;

	return 0;
}

int key_release(struct inode *inode, struct file *filp)
{
	return 0;
}

ssize_t key_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct key_private *kpriv = filp->private_data;

	if (down_interruptible(&kpriv->ksem))
		return -ERESTARTSYS;

	/* wait until kpriv->wp != kpriv->rp, and get mutex */
	while (kpriv->wp == kpriv->rp) {
		up(&kpriv->ksem);
		if (wait_event_interruptible(kpriv->kreadq, (kpriv->wp != kpriv->rp)))
			return -ERESTARTSYS;
		if (down_interruptible(&kpriv->ksem))
			return -ERESTARTSYS;
	}

	/* calculate read count */
	if (kpriv->wp > kpriv->rp)
		count = min(count, (size_t)(kpriv->wp - kpriv->rp));
	if (kpriv->wp < kpriv->rp)
		count = min(count, (size_t)(kpriv->end - kpriv->rp));

	if (copy_to_user(buf, kpriv->rp, count)) {
		up(&kpriv->ksem);
		return -EFAULT;
	}
	
	kpriv->rp += count;
	if (kpriv->rp == kpriv->end)
		kpriv->rp = kpriv->start;

	up(&kpriv->ksem);

	return count;
}


int key_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct key_private *dev = filp->private_data;
	int ret = 0;
	char *new_buf;

	printk("Cmd = 0x%x\n", cmd);
	if (_IOC_TYPE(cmd) != PIPE) {
		ret = -ENOTTY;
		goto out;	
	}

	if (cmd == PIPE_RESET) {
		if (down_interruptible(&dev->ksem)) {
			ret = -ERESTARTSYS;
			goto out;
		}
		dev->rp = dev->wp = dev->start;
	} else if (cmd == PIPE_RESIZE) {
		if (down_interruptible(&dev->ksem)) {
			ret = -ERESTARTSYS;
			goto out;
		}
		new_buf = (char *)kzalloc(arg, GFP_KERNEL);
		if (NULL == new_buf) {
			ret = -ENOMEM;
			goto out1;
		}
		kfree(dev->start);
		dev->start = new_buf;
		dev->rp = dev->wp = dev->start;
		dev->end = dev->start + arg;
	} else {
		printk("Cannot recognize this command\n");
		ret = -EINVAL;
		goto out1;
	}
out1:
	up(&dev->ksem);
out:
	return ret;
}



struct file_operations key_fops = {
	.owner 	= THIS_MODULE, 
	.read  	= key_read,
	.ioctl	= key_ioctl,
	.open	= key_open,
	.release = key_release,
};


int __devinit key_probe(struct platform_device *pdev)
{
	struct key_private *kpriv;
	struct resource *res;
	int ret;

	/* allocate key_private */
	kpriv = (struct key_private *)kzalloc(sizeof(*kpriv), GFP_KERNEL);
	if (NULL == kpriv)
		return -ENOMEM;

	/* allocate key buffer */
	kpriv->buf_len = KEY_BUF;
	kpriv->start = (char *)kzalloc(kpriv->buf_len, GFP_KERNEL);
	if (NULL == kpriv->start) {
		ret = -ENOMEM;
		goto out1;
	}
	kpriv->end = kpriv->start + kpriv->buf_len;
	kpriv->rp = kpriv->wp = kpriv->start;

	/* get ioport from pdev */
	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	kpriv->kio = res->start;
	kpriv->kvir = ioport_map(kpriv->kio, (res->end - res->start + 1));
	if (NULL == kpriv->kvir) {
		ret = -1;
		goto out2;
	}

	/* get irq */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	kpriv->kirq = res->start;
	ret = request_irq(kpriv->kirq, /* irq number */
			key_service, 	/* handler */
			IRQF_SHARED,	/* flags */
			"plat-key",	/* name */
			kpriv);		/* data */
	if (ret) {
		printk("Cannot request key irq %d\n", kpriv->kirq);
		goto out3;
	}

	/* initialize many things in kpriv */
	INIT_WORK(&kpriv->kwork, key_workq);
	init_waitqueue_head(&kpriv->kreadq);
	init_MUTEX(&kpriv->ksem);
	spin_lock_init(&kpriv->kspin);

	/* set dev num */
	if (pdev->id == -1)
		kpriv->knum = MKDEV(major, 0);
	else 
		kpriv->knum = MKDEV(major, pdev->id);
	cdev_init(&kpriv->kcdev, &key_fops);
	kpriv->kcdev.owner = THIS_MODULE;
	cdev_add(&kpriv->kcdev, kpriv->knum, 1);

	/* save kpriv to platform_device */
	platform_set_drvdata(pdev, kpriv);
	
	return 0;

out3:
	ioport_unmap(kpriv->kvir);
out2:
	kfree(kpriv->start);
out1:
	kfree(kpriv);

	return ret;
}

int __devexit key_remove(struct platform_device *pdev)
{
	struct key_private *kpriv = platform_get_drvdata(pdev);

	free_irq(kpriv->kirq, kpriv);
	flush_scheduled_work();

	cdev_del(&kpriv->kcdev);
	ioport_unmap(kpriv->kvir);
	kfree(kpriv->start);
	kfree(kpriv);

	return 0;
}


struct platform_driver key_drv = {
	.probe 	= key_probe,
	.remove	= key_remove,
	.driver	= {
		.owner 	= THIS_MODULE,
		.name	= KEY_NAME,
	},
};

int __init my_init(void)
{
	int ret;
	dev_t dev_id;

	//request chrdev region
	if (major) {
		dev_id = MKDEV(major, 0);
		ret = register_chrdev_region(dev_id, CHAR_COUNT, "MEM schar");
	} else {
		ret = alloc_chrdev_region(&dev_id, 0, CHAR_COUNT, "MEM dchar");
		major = MAJOR(dev_id);
	}

	if (ret) {
		printk("Cannot request chrdev region.\n");
		return -1;
	}

	ret = platform_driver_register(&key_drv);

	if (ret == 0)
		printk("Register key_driver.\n");
	else {
		unregister_chrdev_region(dev_id, CHAR_COUNT);
		printk("Register key_driver error.\n");
	}

	return ret;
}


void __exit my_exit(void)
{
	dev_t dev_id = MKDEV(major, 0);

	unregister_chrdev_region(dev_id, CHAR_COUNT);
	platform_driver_unregister(&key_drv);

	printk("Unregister key_driver\n");
}

module_init(my_init);
module_exit(my_exit);

MODULE_LICENSE("GPL");
MODULE_VERSION("1.5");

