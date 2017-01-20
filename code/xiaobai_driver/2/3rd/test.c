#include <linux/module.h>
#include <linux/init.h>

#define DEBUG_SWITCH	0
#if DEBUG_SWITCH
	#define P_DEBUG(fmt, args...)	printk("<1>" "<kernel>[%s]"fmt, __FUNCTION__, ##args)
#else
	#define P_DEBUG(fmt, args...)	printk("<7>" "<kernel>[%s]"fmt, __FUNCTION__, ##args)
#endif
	

static int __init test_init(void)
{
    printk("hello world!\n");
    P_DEBUG("debug!\n");
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
