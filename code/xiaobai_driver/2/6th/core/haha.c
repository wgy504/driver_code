#include <linux/module.h>
#include <linux/init.h>
#include "../include/haha.h"

int haha(void)
{
	printk("haha!\n");
	return 0;
}

EXPORT_SYMBOL(haha);
MODULE_LICENSE("GPL");
