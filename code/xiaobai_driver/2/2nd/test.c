#include <linux/module.h>
#include <linux/init.h>

static int __init test_init(void)
{
    printk("hello world!\n");
    printk("<0>" "hello world! 0\n");
    printk("<1>" "hello world! 1\n");
    printk("<2>" "hello world! 2\n");
    printk("<3>" "hello world! 3\n");
    printk("<4>" "hello world! 4\n");
    printk("<5>" "hello world! 5\n");
    printk("<6>" "hello world! 6\n");
    printk("<7>" "hello world! 7\n");
	return 0;
}

static void __exit test_exit(void)
{
    printk("good bye!\n");
}

module_init(test_init);
module_exit(test_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xiao bai");
MODULE_VERSION("1.0");
