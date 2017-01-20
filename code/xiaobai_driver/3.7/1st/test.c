#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>

#include <linux/wait.h>
#include <linux/sched.h>

#include <linux/poll.h>

#include <asm/uaccess.h>
#include <linux/errno.h>

#define DEBUG_SWITCH    1
#if DEBUG_SWITCH
	#define P_DEBUG(fmt, args...)   printk("<1>" "<kernel>[%s]"fmt, __FUNCTION__, ##args)
#else
	#define P_DEBUG(fmt, args...)   printk("<7>" "<kernel>[%s]"fmt, __FUNCTION__, ##args)
#endif

#define DEV_SIZE 100

struct _test_t{
	char kbuf[DEV_SIZE];
	unsigned int major;
	unsigned int minor;
	unsigned int cur_size;
	dev_t devno;
	struct cdev test_cdev;
	wait_queue_head_t test_queue;
	wait_queue_head_t read_queue;	
	wait_queue_head_t write_queue;
	struct fasync_struct *async_queue;
};

int test_open(struct inode *node, struct file *filp)
{
	struct _test_t *dev;
	dev = container_of(node->i_cdev, struct _test_t, test_cdev);
	filp->private_data = dev;
	return 0;
}

ssize_t test_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
{
	int ret;
	struct _test_t *dev = filp->private_data;

	if(filp->f_flags & O_NONBLOCK) 
		return - EAGAIN;

	P_DEBUG("read data.....\n");
	if(wait_event_interruptible(dev->test_queue, dev->cur_size > 0))
		return - ERESTARTSYS;

	if (copy_to_user(buf, dev->kbuf, count)){
		ret = - EFAULT;
	}else{
		ret = count;
		dev->cur_size -= count;
		P_DEBUG("read %d bytes, cur_size:[%d]\n", count, dev->cur_size);
		wake_up_interruptible(&dev->write_queue);
	}

	return ret;		//返回实际写入的字节数或错误号
}

ssize_t test_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset)
{
	int ret;
	struct _test_t *dev = filp->private_data;

	if(copy_from_user(dev->kbuf, buf, count)){
		ret = - EFAULT;
	}else{
		ret = count;
		dev->cur_size += count;
		P_DEBUG("write %d bytes, cur_size:[%d]\n", count, dev->cur_size);
		P_DEBUG("kbuf is [%s]\n", dev->kbuf);
		wake_up_interruptible(&dev->test_queue);
		wake_up_interruptible(&dev->read_queue);

		if (dev->async_queue){
			kill_fasync(&dev->async_queue, SIGIO, POLL_IN);
		}
	}

	return ret;		//返回实际写入的字节数或错误号
}

unsigned int test_poll (struct file *filp, struct poll_table_struct *table)
{
	struct _test_t *dev = filp->private_data;
	unsigned int mask = 0;

	poll_wait(filp, &dev->read_queue, table);
	poll_wait(filp, &dev->write_queue, table);

	if(dev->cur_size > 0)	//设备可读
		mask |= POLLIN;	
	if(dev->cur_size < DEV_SIZE) //设备可写
		mask |= POLLOUT;

	P_DEBUG("***mask[%d]***\n", mask);
	return mask;
}

int test_fasync (int fd, struct file *filp, int mode)
{
	struct _test_t *dev = filp->private_data;

	return fasync_helper(fd, filp, mode, &dev->async_queue);
}

int test_close(struct inode *node, struct file *filp)
{
	test_fasync(-1, filp, 0);
	return 0;
}

struct file_operations test_fops = {
	.open = test_open,
	.release = test_close,
	.write = test_write,
	.read = test_read,
	.poll = test_poll,
	.fasync = test_fasync,
};

struct _test_t my_dev;

static int __init test_init(void)	//模块初始化函数
{
	int result = 0;
	my_dev.cur_size = 0;
	my_dev.major = 0;
	my_dev.minor = 0;

	if(my_dev.major){						
		my_dev.devno = MKDEV(my_dev.major, my_dev.minor);
		result = register_chrdev_region(my_dev.devno, 1, "test new driver");
	}else{
		result = alloc_chrdev_region(&my_dev.devno, my_dev.minor, 1, "test alloc diver");
		my_dev.major = MAJOR(my_dev.devno);
		my_dev.minor = MINOR(my_dev.devno);
	}

	if(result < 0){
		P_DEBUG("register devno errno!\n");
		goto err0;
	}

	printk("major[%d] minor[%d]\n", my_dev.major, my_dev.minor);

	cdev_init(&my_dev.test_cdev, &test_fops);
	my_dev.test_cdev.owner = THIS_MODULE;
	/*初始化等待队列头，注意函数调用的位置*/
	init_waitqueue_head(&my_dev.test_queue);
	init_waitqueue_head(&my_dev.read_queue);
	init_waitqueue_head(&my_dev.write_queue);

	result = cdev_add(&my_dev.test_cdev, my_dev.devno, 1);
	if(result < 0){
		P_DEBUG("cdev_add errno!\n");
		goto err1;
	}

	printk("hello kernel\n");
	return 0;

err1:
	unregister_chrdev_region(my_dev.devno, 1);
err0:
	return result;
}

static void __exit test_exit(void)		//模块卸载函数
{
	cdev_del(&my_dev.test_cdev);
	unregister_chrdev_region(my_dev.devno, 1);

	printk("good bye kernel\n");
}

module_init(test_init);
module_exit(test_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xoao bai");
MODULE_VERSION("v0.1");
