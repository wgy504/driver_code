#include <linux/init.h>
#include <linux/module.h> 

static int hello_init(void)
{
	
	printk(KERN_ALERT "Hello, char base £¡\n");

#ifdef DEBUG

	printk(KERN_ALERT "Hello, char base1111111111111111111 £¡\n");
	
#endif	
	
	return 0;
}

static void hello_exit(void)
{
	printk(KERN_ALERT "Goodbye, char base ! \n");
}

module_init(hello_init);
module_exit(hello_exit);

MODULE_LICENSE("Dual BSD/GPL");