#include <linux/module.h>
#include <linux/init.h>

#include <linux/interrupt.h>

irqreturn_t irq_handler(int irqno, void *dev_id)
{
	printk("key down\n");
	return IRQ_HANDLED;	
}

int a = 99;

int test_init(void)
{
	int ret;
	/* 设置IRQF_SHARED后, dev_id必须设置为唯一值 */
	ret = request_irq(IRQ_EINT1, irq_handler, IRQF_SHARED | IRQF_TRIGGER_FALLING, "test key driver", &a);		
	if(ret){
		printk("<kernel> request irq failed\n");	
		return -1;
	}
	printk("hello irq\n");
	return 0;
}
void test_exit(void)
{
	free_irq(IRQ_EINT1, &a);
	printk("bye\n");
}


module_init(test_init);
module_exit(test_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Richard");
MODULE_VERSION("v0.1");
