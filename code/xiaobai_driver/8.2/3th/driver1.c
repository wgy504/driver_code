#include <linux/module.h>
#include <linux/init.h>

#include <linux/device.h>

extern struct bus_type usb_bus;

void init_mouse1(void)
{
	printk("init usb mouse\n");
}

int usb_driver1_probe(struct device *dev)
{//查询特定设备是否存在，以及是否能够才操作该设备，然后再进行设备操作。
	//check_mouse();	//自己假设一下检查设备
	init_mouse1();		//usb鼠标驱动的真正入口
	return 0;
}

int usb_driver1_remove(struct device *dev)
{
	printk("remove mouse driver\n");
	return 0;
}

struct device_driver usb_driver1 = {
	.name = "usb_mouse_1",	//在/sys/中的驱动目录名字
	.bus = &usb_bus,		//必须指定驱动函数所属总线，不然不能注册。
	.probe = usb_driver1_probe,
	.remove = usb_driver1_remove,
};

static int __init usb_driver1_init(void)
{
	int ret;
	/*驱动注册，注册成功后在/sys/bus/usb/driver目录下创建目录usb_mouse*/
	ret = driver_register(&usb_driver1);
	if(ret){
		printk("driver1 register failed!\n");
		return ret;	
	}
	printk("usb driver1 init\n");
	return 0;
}

static void __exit usb_driver1_exit(void)
{
	driver_unregister(&usb_driver1);
	printk("usb driver1 bye!\n");
}

module_init(usb_driver1_init);
module_exit(usb_driver1_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xiao bai");
