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

#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/device.h>  
 
#include <linux/kernel.h>  
 
#include <linux/string.h>  
#include <linux/sysfs.h>  
#include <linux/stat.h> 
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/fs.h>

#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <asm/system.h>

#include <linux/delay.h>



#define DEBUG_SWITCH    1
#if DEBUG_SWITCH
	#define P_DEBUG(fmt, args...)   printk("<1>" "<kernel>[%s]"fmt, __FUNCTION__, ##args)
#else
	#define P_DEBUG(fmt, args...)   printk("<7>" "<kernel>[%s]"fmt, __FUNCTION__, ##args)
#endif
#define DEV_SIZE 100

struct my_dev_t
{
	char kbuf[DEV_SIZE];
	dev_t devno;
	struct cdev cdev;
	struct class *cls;
	struct device *dev;
	unsigned int cur_size;

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





int my_close(struct inode *node, struct file *filp)
{
	return 0;
}

ssize_t my_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
{
	int ret;
	struct my_dev_t *dev = filp->private_data;

	if(*offset >= DEV_SIZE){//如果偏移量已经超过了数组的容量
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

ssize_t my_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset)
{
	int ret;
	struct my_dev_t *dev = filp->private_data;

	if(*offset >= DEV_SIZE){//如果偏移量已经超过了数组的容量
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

loff_t my_llseek (struct file *filp, loff_t offset, int whence)
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



static const struct file_operations fops = 
{
	.owner = THIS_MODULE,
	.open = my_open,
	.read = my_read,
	.write = my_write,
	.llseek = my_llseek,
	.release = my_close,
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

	/*添加cdev*/
	cdev_init(&my_dev->cdev, &fops);
	ret = cdev_add(&my_dev->cdev, my_dev->devno, 1);
	if(ret < 0){
		printk("Err, init cdev_add failed!\n");
		goto cdev_add_error;
	}

	/*创建class*/
	my_dev->cls = class_create(THIS_MODULE, "my_class");
	/*用IS_ERR判断返回的指针是否合法， @@@driver@@@*/
	if(IS_ERR(my_dev->cls)){
		printk("Err, init class_create failed!\n");
		ret = -1;
		goto class_create_error;

	}

	/*创建设备*/
	my_dev->dev = device_create(my_dev->cls, NULL, my_dev->devno, NULL,"test");
	if(IS_ERR(my_dev->dev)){
		printk("Err, init device_create failed!\n ");
		ret = -1;
		goto device_create_error;
	}

	return 0;

device_create_error:
	class_destroy(my_dev->cls);
class_create_error:

	cdev_del(&my_dev->cdev);
cdev_add_error:

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





