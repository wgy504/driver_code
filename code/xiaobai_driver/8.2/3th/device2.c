#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>

extern struct bus_type usb_bus;

void usb_dev_release_2(struct device *dev)	//卸载函数没有干具体的事情
{
		printk("<kernel> release\n");
}

struct device usb_device2 = {
	.bus_id = "2_usb_mouse",
	.bus = &usb_bus,			//指定该设备的总线,在/sys/bus/usb
	.release = usb_dev_release_2,	//必须要都有release函数，不然卸载时会出错
};

static int __init usb_device2_init(void)
{
	int ret;

	ret = device_register(&usb_device2);
	if(ret){
		printk("device2 register failed!\n");
		return ret;	
	}

	printk("usb device2 init\n");
	return 0;
}

static void __exit usb_device2_exit(void)
{
	device_unregister(&usb_device2);
	printk("usb device2 bye!\n");
}

module_init(usb_device2_init);
module_exit(usb_device2_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xiao bai");
