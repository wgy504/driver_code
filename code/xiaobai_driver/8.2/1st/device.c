#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>

extern struct bus_type usb_bus;

void usb_dev_release(struct device *dev)	//卸载函数没有干具体的事情
{
		printk("<kernel> release\n");
}

struct device usb_device = {
	.bus_id = "usb_device",
	.bus = &usb_bus,			//指定该设备的总线,在/sys/bus/usb
	.release = usb_dev_release,	//必须要都有release函数，不然卸载时会出错
};

static int __init usb_device_init(void)
{
	int ret;
/*设备注册，注册成功后在/sys/device目录下创建目录usb_device并在指定总线
 * usb_bus的目录/sys/bus/usb/device创建/sys/device/usb_device的软连接*/
	ret = device_register(&usb_device);
	if(ret){
		printk("device register failed!\n");
		return ret;	
	}

	printk("usb device init\n");
	return 0;
}

static void __exit usb_device_exit(void)
{
	device_unregister(&usb_device);
	printk("usb device bye!\n");
}

module_init(usb_device_init);
module_exit(usb_device_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xiao bai");
