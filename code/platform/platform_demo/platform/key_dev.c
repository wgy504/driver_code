#include <linux/module.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/time.h>

#define KEY_IO 	0x60
#define KEY_IRQ 1
#define KEY_NAME "key-i8042"


void key_release(struct device *dev)
{
	struct timeval tval;
	do_gettimeofday(&tval);
	printk("Key release: %ld:%ld.\n", tval.tv_sec, tval.tv_usec);
}


struct resource key_res[] = {
	[0] = {
		.start 	= KEY_IO,
		.end	= KEY_IO,
		.flags	= IORESOURCE_IO,
	},
	[1] = {
		.start 	= KEY_IRQ,
		.end	= KEY_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};


struct platform_device key_dev = {
	.name 	= KEY_NAME, 
	.id	= -1,
	.num_resources 	= ARRAY_SIZE(key_res),
	.resource	= key_res,
	.dev	= {
		.release = key_release,
	}
};


int __init my_init(void)
{
	int ret;
	struct timeval tval;

	ret = platform_device_register(&key_dev);
	do_gettimeofday(&tval);

	if (ret == 0) {
		printk("Register key dev: %ld:%ld.\n", tval.tv_sec, tval.tv_usec);
	} else {
		printk("Register key dev error.\n");
	}

	return ret;
}

void __exit my_exit(void)
{
	struct timeval tval;

	platform_device_unregister(&key_dev);
	do_gettimeofday(&tval);
	printk("Unregister key dev: %ld:%ld.\n", tval.tv_sec, tval.tv_usec);
}

module_init(my_init);
module_exit(my_exit);

MODULE_LICENSE("GPL");
MODULE_VERSION("1.5");

