#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>

#include <linux/cdev.h>
#include <linux/errno.h>

#include <linux/wait.h>
#include <linux/sched.h>

#include <linux/poll.h>

#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/preempt.h>

#define DEV_SIZE 100

struct _test_t{
	char kbuf[DEV_SIZE];	
	unsigned int major;
	unsigned int minor;
	unsigned int cur_size;
	dev_t devno;
	struct cdev test_cdev;

};



int test_open(struct inode *node, struct file *filp)
{
	struct _test_t *dev;
	printk("<kernel>[%s]\n", __FUNCTION__);	
	dev = container_of(node->i_cdev, struct _test_t, test_cdev);
	filp->private_data = dev;

	return 0;
}

ssize_t test_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
{
	int ret;	
	unsigned long flag = 0;

	for(; ;)
	{
		preempt_disable();
		local_irq_save(flag);
		printk("<kernel>[%s]task pid[%d], context [%s]\n", __FUNCTION__, current->pid, current->comm);
		local_irq_restore(flag);
		preempt_enable();
		mdelay(2000);
	}
	
	return count - ret;
}

struct file_operations test_fops = {
	.open = test_open,	
	.read = test_read,
};

struct _test_t my_dev;

int test_init(void)
{
	int ret;	
	my_dev.cur_size = 0;
	my_dev.major = 0;
	my_dev.minor = 0;
//	register_chrdev(major, "test_driver", &test_fops);		
if(my_dev.major){
	my_dev.devno = MKDEV(my_dev.major, my_dev.minor);
	ret = register_chrdev_region(my_dev.devno, 1, "test new driver");	
	if(ret)
	{
		printk("register_chrdev_region failed\n");
		ret = -EBUSY;
		goto err0;
	}
}else{
	ret = alloc_chrdev_region(&my_dev.devno, my_dev.minor, 1, "test alloc driver");	
	if(ret)
	{
		printk("register_chrdev_region failed\n");
		ret = -EBUSY;
		goto err0;
	}

	my_dev.major = MAJOR(my_dev.devno);
	my_dev.minor = MINOR(my_dev.devno);
	printk("alloc major[%d], minor[%d]\n", my_dev.major, my_dev.minor);
}
	cdev_init(&my_dev.test_cdev, &test_fops);


	ret = cdev_add(&my_dev.test_cdev, my_dev.devno, 1);
	if(ret)
	{
		printk("cdev_add\n");
		ret = -ENODEV;
		goto err1;
	}

	printk("hello kernel\n");

	return 0;

err1:
	unregister_chrdev_region(my_dev.devno, 1);
err0:
	return ret;
}

void test_exit(void)
{
//	unregister_chrdev(major, "test_driver");
	cdev_del(&my_dev.test_cdev);
	unregister_chrdev_region(my_dev.devno, 1);
	printk("bye\n");
}


module_init(test_init);
module_exit(test_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Richard");
MODULE_VERSION("v0.1");
