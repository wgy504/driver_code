#include <linux/module.h>
#include <linux/time.h>
#include <linux/platform_device.h>

#define RTL8169_MEM 	0xe1110000
#define RTL8169_SIZE 	0x1000
#define RTL8169_IRQ	26

void rtl8169_release(struct device *dev)
{
	struct timeval tval;
	do_gettimeofday(&tval);
	printk("Call 8169 release in %ld:%ld us.\n", tval.tv_sec, tval.tv_usec);
}


struct resource rtl8169_res[] = {
	[0] = {
		.start 	= RTL8169_MEM,
		.end	= RTL8169_MEM + RTL8169_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start 	= RTL8169_IRQ,
		.end	= RTL8169_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device rtl8169_dev = {
	.name 	= "RAW-DEV",
	.id 	= 10,
	.num_resources = ARRAY_SIZE(rtl8169_res),
	.resource = rtl8169_res,
	.dev = {
		.release = rtl8169_release,
	},
};
	

int __init my_init(void)
{
	printk("Register raw 10--rtl8169 dev.\n");
	return platform_device_register(&rtl8169_dev);
}

void __exit my_exit(void)
{
	printk("Unregister raw-10--8139 dev.\n");
	platform_device_unregister(&rtl8169_dev);
}

module_init(my_init);
module_exit(my_exit);

MODULE_LICENSE("GPL");

