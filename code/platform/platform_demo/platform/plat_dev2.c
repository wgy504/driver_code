#include <linux/module.h>
#include <linux/time.h>
#include <linux/platform_device.h>

#define EHCI_MEM 	0xe1284000
#define EHCI_SIZE 	0x400
#define EHCI_IRQ	23

void ehci_release(struct device *dev)
{
	struct timeval tval;
	do_gettimeofday(&tval);
	printk("Call ehci release in %ld:%ld us.\n", tval.tv_sec, tval.tv_usec);
}


struct resource ehci_res[] = {
	[0] = {
		.start 	= EHCI_MEM,
		.end	= EHCI_MEM + EHCI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start 	= EHCI_IRQ,
		.end	= EHCI_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device ehci_dev = {
	.name 	= "RAW-DEV",
	.id 	= 20,
	.num_resources = ARRAY_SIZE(ehci_res),
	.resource = ehci_res,
	.dev = {
		.release = ehci_release,
	},
};
	

int __init my_init(void)
{
	printk("Register raw 20--ehci dev.\n");
	return platform_device_register(&ehci_dev);
}

void __exit my_exit(void)
{
	printk("Unregister raw-20--ehci dev.\n");
	platform_device_unregister(&ehci_dev);
}

module_init(my_init);
module_exit(my_exit);

MODULE_LICENSE("GPL");

