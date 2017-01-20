/*		
 *	name:	bus_device.c
 *	auth:	wuw
 *	date:	2016年3月7日 
 *	
 *
 */



#include <linux/device.h>  
#include <linux/module.h>  
#include <linux/kernel.h>  
#include <linux/init.h>  
#include <linux/string.h>  
#include <linux/sysfs.h>  
#include <linux/stat.h> 
#include <linux/slab.h> 


extern struct device my_bus_dev;
extern struct bus_type my_bus_type;


//static char *dev_name = "bus_dev_attr";
static void my_release(struct device *dev)  
{   
	pr_info("%s release device\n",__FUNCTION__);
}  

struct device my_dev = {
	.bus = &my_bus_type,
	.parent = &my_bus_dev,
	.release = my_release,


};

static ssize_t mydev_show(struct device *dev, struct device_attribute *attr, char *buf)  
{  
    return sprintf(buf, "%s\n", "This is my device!");  
}  
static DEVICE_ATTR(my_name, S_IRUGO, mydev_show, NULL);  


static int __init platform_device_test_init(void)
{
	int ret = 0;
	pr_info("%s !\n", __FUNCTION__);
	
	dev_set_name(&my_dev, "my_dev");
	ret = device_register(&my_dev);
	ret = device_create_file(&my_dev, &dev_attr_my_name);
	/*再增加一个属性文件是怎样的*/
	return ret;
	
	

}

static void __exit platform_device_test_exit(void)
{
	pr_info("%s !\n", __FUNCTION__);
	device_unregister(&my_dev);  


}


module_init(platform_device_test_init);  
module_exit(platform_device_test_exit);  


MODULE_AUTHOR("yshisx");  
MODULE_LICENSE("Dual BSD/GPL");

