#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>

extern struct bus_type usb_bus;

void usb_dev_release_1(struct device *dev)	//卸载函数没有干具体的事情
{
		printk("<kernel> release\n");
}

struct device usb_device1 = {
	.bus_id = "usb_mouse_1",
	.bus = &usb_bus,			//指定该设备的总线,在/sys/bus/usb
	.release = usb_dev_release_1,	//必须要都有release函数，不然卸载时会出错
};

static int __init usb_device1_init(void)
{
	int ret;

	ret = device_register(&usb_device1);
	if(ret){
		printk("device1 register failed!\n");
		return ret;	
	}

	printk("usb device1 init\n");
	return 0;
}

static void __exit usb_device1_exit(void)
{
	device_unregister(&usb_device1);
	printk("usb device1 bye!\n");
}

module_init(usb_device1_init);
module_exit(usb_device1_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xiao bai");
