#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>	/* printk() to debug*/
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>		/* everything... */
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/fcntl.h>	/* O_ACCMODE */
#include <linux/cdev.h>		/* char driver */
#include <asm/system.h>		/* cli(), *_flags */
#include <asm/uaccess.h>	/* copy_*_user */
#include "irq_stack_test.h"		/* local definitions */
#include <linux/sched.h>
#include <plat/irqs.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include "mux.h"


#define TASKLET


#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))
char *name = "gpio_irq_stack";

#ifdef TASKLET

	static void do_mytasklet(unsigned long n);

	struct tasklet_struct my_tasklet;
	int tasklet_data = 0;
	//DECLARE_TASKLET(my_tasklet, do_mytasklet, 0);
	static void do_mytasklet(unsigned long n)
	{
		//pr_info("%s : n = %ld\n", __func__, n);
		pr_info("%s : tasklet_data = %d\n", __func__, tasklet_data++);
	} 

#endif
static irqreturn_t rst_rec_interrupt(int irq, void *dev_id)
{
	int test_point;
	pr_info("-------------\n");
	printk("\n%s-PID:%d-kernel_stack@0x%p-to-%lx\ntest_point@0x%lx\n", 
			current->comm, current->pid, current->stack, ((unsigned long)current->stack)+(THREAD_START_SP),
			(unsigned long)(&test_point));
#ifdef TASKLET			
	tasklet_schedule(&my_tasklet);
#endif
	return IRQ_HANDLED;
}
EXPORT_SYMBOL_GPL(rst_rec_interrupt);

/*
 * Finally, the module stuff
 */

/*
 * The cleanup function is used to handle initialization failures as well.
 * Thefore, it must be careful to work correctly even if some of the items
 * have not been initialized
 */
void irq_stack_cleanup(void)
{
		pr_info("free irq %d, gpio %d.\n",gpio_to_irq(GPIO_TO_PIN(1, 16)), GPIO_TO_PIN(1, 16));
		free_irq(gpio_to_irq(GPIO_TO_PIN(1, 16)) , (void *)name);
		gpio_free(GPIO_TO_PIN(1, 16));
		tasklet_kill(&my_tasklet);
}
struct test
{
	 char num:3;
	 char po:5;
}gs_tt;

int __init irq_stack_init(void)
{
	int result, ret;
	int num = 0;
	
	int pin;
	int irq_num;
	char *gpio_name = "gpio_irq_0";
#ifdef TASKLET	

	tasklet_data = 888;
	tasklet_init(&my_tasklet, do_mytasklet, tasklet_data);
#endif	
	pr_info("%s start!", __FUNCTION__);
	gs_tt.num = 0x01;
	gs_tt.po = 0x11;
	num = *((char*)&gs_tt);
	pr_info("num = %d\n", num);
	//pr_info("addr = %lx, test = %d\n", tt, *tt);
	//pr_info("addr = %lx, test = %d\n", ((u8*)tt+1), *((u8*)tt+1));
	pin = GPIO_TO_PIN(1, 16);
	omap_mux_init_signal("gpmc_a0.gpio1_16", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP);
	ret = gpio_request_one(pin, GPIOF_IN, gpio_name);
	if(ret != 0)
	{
        printk("gpio request %s error, pin = %d\n", "gpmc_a0.gpio1_16",GPIO_TO_PIN(1, 16));
				return 1;
	}

	gpio_export(pin, 1);
	irq_num = gpio_to_irq(pin);
	result = request_irq(irq_num , rst_rec_interrupt, IRQF_SHARED | IRQF_TRIGGER_FALLING, gpio_name, (void *)name);
	if (result) {
		printk(KERN_WARNING "irq_stack: can't get n irq!\n");
		goto fail;
	}
	pr_info("request irq succeed, irq number = %d\n", irq_num );
	

	
	pr_info("**********%s*********\n", __func__);
	return 0; /* succeed */

  fail:
	irq_stack_cleanup();
	return result;
}

module_init(irq_stack_init);
module_exit(irq_stack_cleanup);

MODULE_DESCRIPTION("irq stack test module");
MODULE_VERSION("v1.0");
MODULE_AUTHOR("wuw");
MODULE_LICENSE("Dual BSD/GPL");
