#include <linux/module.h>
#include <linux/init.h>

#include <linux/device.h>

extern struct bus_type usb_bus;

struct device_driver usb_driver = {
	.name = "usb_driver",	//在/sys/中的驱动目录名字
	.bus = &usb_bus,		//必须指定驱动函数所属总线，不然不能注册。
};

static int __init usb_driver_init(void)
{
	int ret;
	/*驱动注册，注册成功后在/sys/bus/usb/driver目录下创建目录usb_driver*/
	ret = driver_register(&usb_driver);
	if(ret){
		printk("driver register failed!\n");
		return ret;	
	}
	printk("usb driver init\n");
	return 0;
}

static void __exit usb_driver_exit(void)
{
	driver_unregister(&usb_driver);
	printk("usb driver bye!\n");
}

module_init(usb_driver_init);
module_exit(usb_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xiao bai");
