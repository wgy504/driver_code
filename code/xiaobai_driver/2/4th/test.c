#include <linux/module.h>
#include <linux/init.h>

int num = 123;
char *name = "xiao bai";

static int __init test_init(void)
{
    printk("hello world!\n");
	printk("num = %d, name:[%s]\n", num, name);
	return 0;
}

static void __exit test_exit(void)
{
    printk("good bye!\n");
}

module_init(test_init);
module_exit(test_exit);
module_param(num,  int, 0644);
module_param(name, charp, 0644);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xiao bai");
MODULE_VERSION("1.0");
