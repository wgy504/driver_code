#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>

#include <asm/uaccess.h>
#include <linux/errno.h>

#include "test_cmd.h"

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
};

int test_open(struct inode *node, struct file *filp)
{
	struct _test_t *dev;
	dev = container_of(node->i_cdev, struct _test_t, test_cdev);
	filp->private_data = dev;
	return 0;
}

int test_close(struct inode *node, struct file *filp)
{
	return 0;
}

ssize_t test_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
{
	int ret;
	struct _test_t *dev = filp->private_data;

	/*如果偏移量已经超过了数组的容量*/
	if(*offset >= DEV_SIZE || dev->cur_size == 0){
		return count ? - ENXIO : 0;	//count为0则返回0，表示读取0个数据成功
	}								//count不为0则分会错误号，地址越界
	if(*offset + count > DEV_SIZE){	//如果读取字节数超过了最大偏移量
		count = DEV_SIZE - *offset;	//则减少读取字节数。
	}
	/*copy_to_user的参数也要改一下*/
	if (copy_to_user(buf, dev->kbuf + *offset, count)){
		ret = - EFAULT;
	}else{
		ret = count;
		dev->cur_size -= count;	//读取后数组的字节数减少
		*offset += count;		//偏移量增加
		P_DEBUG("read %d bytes, cur_size:[%d]\n", count, dev->cur_size);
	}

	return ret;		//返回实际写入的字节数或错误号
}

ssize_t test_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset)
{
	int ret;
	struct _test_t *dev = filp->private_data;

	/*如果偏移量已经超过了数组的容量*/
	if(*offset >= DEV_SIZE || dev->cur_size == DEV_SIZE){
		return count ? - ENXIO : 0;	//count为0则返回0，表示读取0个数据成功
	}								//count不为0则分会错误号，地址越界
	if(*offset + count > DEV_SIZE){	//如果读取字节数超过了最大偏移量
		count = DEV_SIZE - *offset;	//则减少读取字节数。
	}
	/*copy_from_user的参数也要改一下*/
	if(copy_from_user(dev->kbuf + *offset, buf, count)){
		ret = - EFAULT;
	}else{
		ret = count;
		dev->cur_size += count;	//写入后数组的字节数增加
		*offset += count;		//偏移量增加
		P_DEBUG("write %d bytes, cur_size:[%d]\n", count, dev->cur_size);
		P_DEBUG("kbuf is [%s]\n", dev->kbuf);
	}

	return ret;		//返回实际写入的字节数或错误号
}

loff_t test_llseek (struct file *filp, loff_t offset, int whence)
{
	loff_t new_pos;					//新偏移量
	loff_t old_pos = filp->f_pos;	//旧偏移量

	switch(whence){
		case SEEK_SET:
			new_pos = offset;
			break;
		case SEEK_CUR:
			new_pos = old_pos + offset;
			break;
		case SEEK_END:
			new_pos = DEV_SIZE + offset;
			break;
		default:
			P_DEBUG("unknow whence\n");
			return - EINVAL;
	}

	if(new_pos < 0 || new_pos > DEV_SIZE){	//如果偏移量越界，返回错误号
		P_DEBUG("f_pos failed\n");
		return - EINVAL;
	}

	filp->f_pos = new_pos;
	return new_pos;							//正确返回新的偏移量
}

int test_ioctl (struct inode *node, struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct _test_t *dev = filp->private_data;

	switch(cmd){
		case TEST_CLEAR:
			memset(dev->kbuf, 0, DEV_SIZE);
			dev->cur_size = 0;
			filp->f_pos = 0;
			ret = 0;
			break;
		default:	/*命令错误时的处理*/
			P_DEBUG("error cmd!\n");
			ret = - EINVAL;
			break;
	}

	return ret;
}

struct file_operations test_fops = {
	.open = test_open,
	.release = test_close,
	.write = test_write,
	.read = test_read,
	.llseek = test_llseek,
	.ioctl = test_ioctl,
};

struct _test_t my_dev;

static int __init test_init(void)	//模块初始化函数
{
	int result = 0;
	my_dev.cur_size = 0;
	my_dev.major = 0;
	my_dev.minor = 0;

	/*1.申请设备号*/
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

	/*2.注册设备*/
	cdev_init(&my_dev.test_cdev, &test_fops);
	my_dev.test_cdev.owner = THIS_MODULE;
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
	/*1.从内核中删除cdev*/
	cdev_del(&my_dev.test_cdev);
	/*2.注销设备号*/
	unregister_chrdev_region(my_dev.devno, 1);

	printk("good bye kernel\n");
}

module_init(test_init);
module_exit(test_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xoao bai");
MODULE_VERSION("v0.1");
