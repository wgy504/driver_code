#include <linux/module.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/waitqueue.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>

#define KEY_NAME "key-i8042"
#define BUF_LEN	 4000

struct key_private {
	char *start, *end;
	char *wp, *rp;
	unsigned long buf_len;
	char tmp[10];
	int kpos;

	void __iomem *kvir;
	int kirq;
	wait_queue_head_t kreadq;
	work_struct kwork;
	struct semaphore ksem;
	spinlock_t kspin;
	struct cdev kcdev;
};
	




int __devinit key_probe(struct platform_device *pdev)
{
	struct timeval tval;
	do_gettimeofday(&tval);
	printk("Enter probe, pdev->id = %d: %ld:%ld.\n", pdev->id, tval.tv_sec, tval.tv_usec);

	return 0;
}

int __devexit key_remove(struct platform_device *pdev)
{
	struct timeval tval;
	do_gettimeofday(&tval);
	printk("Enter romove, pdev->id = %d: %ld:%ld.\n", pdev->id, tval.tv_sec, tval.tv_usec);
	return 0;
}


struct platform_driver key_drv = {
	.probe 	= key_probe,
	.remove	= key_remove,
	.driver	= {
		.owner 	= THIS_MODULE,
		.name	= KEY_NAME,
	},
};

int __init my_init(void)
{
	int ret;
	struct timeval tval;

	ret = platform_driver_register(&key_drv);
	do_gettimeofday(&tval);

	if (ret == 0)
		printk("Register key_driver: %ld:%ld\n", tval.tv_sec, tval.tv_usec);
	else 
		printk("Register key_driver error.\n");

	return ret;
	
}

void __exit my_exit(void)
{	
	struct timeval tval;

	platform_driver_unregister(&key_drv);
	do_gettimeofday(&tval);
	printk("Unregister key_driver: %ld:%ld\n", tval.tv_sec, tval.tv_usec);
}

module_init(my_init);
module_exit(my_exit);

MODULE_LICENSE("GPL");
MODULE_VERSION("1.5");

