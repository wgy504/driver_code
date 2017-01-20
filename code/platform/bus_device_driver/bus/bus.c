/*		
 *	name:	platform_bus.c
 *	auth:	wuw
 *	date:	2016Äê3ÔÂ7ÈÕ 
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



static char *version = "my_bus_v0.1";

static ssize_t show_bus_version(struct bus_type *bus, char *buf)  
{  
    return snprintf(buf, PAGE_SIZE, "%s\n", version);  
}  

static BUS_ATTR(version, S_IRUGO, show_bus_version, NULL); 



static void my_bus_release(struct device *dev)  
{
	 printk(KERN_INFO "my bus release\n");  
}

static int my_match(struct device *dev, struct device_driver *driver)  
{  
    return !strncmp(dev_name(dev), driver->name, strlen(driver->name));  
}  

struct bus_type my_bus_type = {
	.name = "my_bus_type",
	.match = my_match,
};

struct device my_bus_dev = {

	.init_name = "my_bus",
	.release = my_bus_release,
};


EXPORT_SYMBOL(my_bus_type);
EXPORT_SYMBOL(my_bus_dev);

static int __init platform_bus_test_init(void)
{
	int ret = 0;
	ret = bus_register(&my_bus_type);
	if(ret)
		return ret;
	if(bus_create_file(&my_bus_type, &bus_attr_version))
		printk(KERN_INFO "Fail to create version attribute!\n");
	ret = device_register(&my_bus_dev);
	if(ret){
		printk(KERN_INFO "Fail to register device:my_bus!\n"); 
	}
		
		return ret;
}




static void __exit platform_bus_test_exit(void)
{

	device_unregister(&my_bus_dev);  
    bus_unregister(&my_bus_type);  
}

module_init(platform_bus_test_init);  
module_exit(platform_bus_test_exit);  


MODULE_AUTHOR("yshisx");  
MODULE_LICENSE("Dual BSD/GPL");


