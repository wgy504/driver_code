/*		
 *	name:	waitqueuqe_test.c
 *	auth:	wuw
 *	date:	2016年1月30日 18:47:31
 *	
 *
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/device.h>  
#include <linux/wait.h> 
#include <linux/kernel.h>  
 
#include <linux/string.h>  
#include <linux/sysfs.h>  
#include <linux/stat.h> 
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/fs.h>

#include <linux/mm.h>
#include <linux/sched.h>

#include <asm/io.h>
#include <asm/system.h>

#include <linux/delay.h>



#define DEBUG_SWITCH    1
#if DEBUG_SWITCH
	#define P_DEBUG(fmt, args...)   printk("<1>" "<kernel>[%s]"fmt, __FUNCTION__, ##args)
#else
	#define P_DEBUG(fmt, args...)   printk("<7>" "<kernel>[%s]"fmt, __FUNCTION__, ##args)
#endif


struct my_dev_t
{
	
	dev_t devno;
	struct cdev cdev;
	struct class *cls;
	struct device *dev;
	struct fasync_struct *async_queue;

};


struct my_dev_t *my_dev;
static int dev_major = 0;


int my_open(struct inode *inode, struct file *filp)
{
	struct my_dev_t *dev;
	dev = container_of(inode->i_cdev, struct my_dev_t, cdev);
	filp->private_data = dev;
	return 0;

}







ssize_t my_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
{
	int ret = 0;
	struct my_dev_t *dev = filp->private_data;

	

	return ret;	
}

ssize_t my_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset)
{
	int ret = 0;
	struct my_dev_t *dev = filp->private_data;
	if (dev->async_queue){
			kill_fasync(&dev->async_queue, SIGIO, POLL_IN);
		}
	

	return ret;		
}

loff_t my_llseek (struct file *filp, loff_t offset, int whence)
{
	loff_t new_pos = 0;					//新偏移量
	loff_t old_pos = filp->f_pos;	//旧偏移量

	
	filp->f_pos = new_pos;
	return new_pos;							//正确返回新的偏移量
}

int my_fasync (int fd, struct file *filp, int mode)
{
	struct my_dev_t *dev = filp->private_data;

	return fasync_helper(fd, filp, mode, &dev->async_queue);
}

int my_close(struct inode *node, struct file *filp)
{
	my_fasync(-1, filp, 0);
	return 0;
}

static const struct file_operations fops = 
{
	.owner = THIS_MODULE,
	.open = my_open,
	.read = my_read,
	.write = my_write,
	.llseek = my_llseek,
	.release = my_close,
	.fasync = my_fasync,
};



static int __init test_init(void)
{
	int ret = 0;
	
	my_dev = kmalloc(sizeof(struct my_dev_t), GFP_KERNEL);
	if(my_dev == NULL){
		printk("Err, init kmalloc failed!\n");
		return -1;
	}
	memset(my_dev, 0, sizeof(struct my_dev_t));
	/*用主设备号和次设备号得到设备号, 注册设备号*/
	my_dev->devno = MKDEV(dev_major, 0);
	if(dev_major)
		ret = register_chrdev_region(my_dev->devno, 1, "my_dev");
	else{
		ret = alloc_chrdev_region(&my_dev->devno, 0, 1, "my_dev");
		dev_major = MAJOR(my_dev->devno);
	}
	if(ret < 0)
		goto register_chrdev_error;
	/*创建class*/
	my_dev->cls = class_create(THIS_MODULE, "my_class");
	/*用IS_ERR判断返回的指针是否合法， @@@driver@@@*/
	if(IS_ERR(my_dev->cls)){
		printk("Err, init class_create failed!\n");
		ret = -1;
		goto class_create_error;

	}
	/*添加cdev*/
	cdev_init(&my_dev->cdev, &fops);
	ret = cdev_add(&my_dev->cdev, my_dev->devno, 1);
	if(ret < 0){
		printk("Err, init cdev_add failed!\n");
		goto cdev_add_error;
	}

	/*创建设备*/
	my_dev->dev = device_create(my_dev->cls, NULL, my_dev->devno, NULL,"my_dev");
	if(IS_ERR(my_dev->dev)){
		printk("Err, init device_create failed!\n ");
		ret = -1;
		goto device_create_error;
	}

	return 0;

device_create_error:
	class_destroy(my_dev->cls);
	
cdev_add_error:
	cdev_del(&my_dev->cdev);
	
class_create_error:
	unregister_chrdev_region(my_dev->devno,1);
	
register_chrdev_error:
	kfree(my_dev);


	return ret;
}



static void __exit test_exit(void)
{
	device_destroy(my_dev->cls, my_dev->devno);
	class_destroy(my_dev->cls);
	cdev_del(&my_dev->cdev);
	unregister_chrdev_region(my_dev->devno, 1);



}

module_init(test_init);  
module_exit(test_exit);  


MODULE_AUTHOR("yshisx");  
MODULE_LICENSE("Dual BSD/GPL"); 





