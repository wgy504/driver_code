/*		
 *	name:	bus_driver.c
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



static int my_probe(struct device *dev)  
{  

	


    printk("Driver found device which my driver can handle!\n");  
    return 0;  
}  
static int my_remove(struct device *dev)  
{  
    printk("Driver found device unpluged!\n");  
    return 0;  
}  



struct device_driver my_driver = {  
	.name = "my_dev",  
	.bus = &my_bus_type,  
	.probe = my_probe,  
	.remove = my_remove,  
};  

static ssize_t mydriver_show(struct device_driver *driver, char *buf)  
{  
    return sprintf(buf, "%s\n", "This is my driver!");  
}  
static DRIVER_ATTR(drv, S_IRUGO, mydriver_show, NULL);  

static int __init platform_driver_test_init(void)
{
	int ret = 0;
	pr_info("%s !\n", __FUNCTION__);
	  
    /*注册驱动*/  
    ret = driver_register(&my_driver);  
    /*创建属性文件*/  
    ret = driver_create_file(&my_driver, &driver_attr_drv);  
    
	
	return ret;
	
	

}

static void __exit platform_driver_test_exit(void)
{
	pr_info("%s !\n", __FUNCTION__);
	 
	driver_unregister(&my_driver);  


}


module_init(platform_driver_test_init);  
module_exit(platform_driver_test_exit);  


MODULE_AUTHOR("yshisx");  
MODULE_LICENSE("Dual BSD/GPL");

