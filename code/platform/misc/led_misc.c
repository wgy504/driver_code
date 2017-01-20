#include <linux/module.h>  
#include <linux/init.h>  
#include <linux/gpio.h>
#include <linux/platform_device.h>  
#include <asm-generic/sizes.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/syscore_ops.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <mach/irqs.h>

#include <asm/mach/irq.h>

#include <linux/delay.h>

#include "/mnt/hgfs/system/linux-3.2-g90b_mini/arch/arm/mach-omap2/mux.h"

#include <linux/miscdevice.h>  
  
#define MISC_NAME "my_misc"  


struct pinmux_config {
	const char *string_name; /* signal name format */
	int val; /* Options for the mux register value */
	int gpio_num; //number
    int init_level;
    const char *my_name;
};
#define OMAP_MUX_MODE7      7
#define AM33XX_PULL_UP			(1 << 4)
#define	AM33XX_PIN_OUTPUT_PULLUP	(AM33XX_PULL_UP)
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

static struct pinmux_config led_io_resource[] = {
	{"lcd_data0.gpio2_6", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP, GPIO_TO_PIN(2, 6), GPIOF_OUT_INIT_LOW, "my_led_0"},
	{NULL, 0},
	
	
	
};


//自定义platform_data结构
struct my_led_data
{
	const char *name;
	struct pinmux_config *pinmux_conf;
};


//自定义platform_data 数据
static struct my_led_data led_data[1] = {
	[0] = {
		.name = "led",
		.pinmux_conf = led_io_resource,	
	},

	
	
};


#define LED_0_ADDR  0x44E07000

static struct resource my_led_resource[] = {
	[0] = {
		.start = LED_0_ADDR,
		.end = LED_0_ADDR,
		.flags = IORESOURCE_MEM,
	},

};




static void setup_pin_mux(struct pinmux_config *pin_mux)
{
	int i, ret;

	for (i = 0; pin_mux->string_name != NULL; pin_mux++)
    {
		omap_mux_init_signal(pin_mux->string_name, pin_mux->val);
        if(pin_mux->gpio_num < 0)
        {
            continue;
        }
        ret = gpio_request_one(pin_mux->gpio_num, pin_mux->init_level, pin_mux->my_name);
        if(ret != 0)
        {
            pr_err("Error requesting gpio %s (gpio%d_%d): %d\n", pin_mux->my_name,
                    pin_mux->gpio_num / 32, pin_mux->gpio_num % 32, ret);
        }
        else
        {
            gpio_export(pin_mux->gpio_num, 1);
        }
    }
}


	
static int my_led_drv_remove(struct platform_device *pdev)
{
	pr_info("%s\n", __func__);
	
	return 0;
}


void __iomem *g_gpio_oe = NULL;
void __iomem *g_gpio_dataout = NULL;

static int my_misc_open(struct inode *nd, struct file *filp)
{
	g_gpio_oe = ( unsigned long *)ioremap(0x44E07000+0x134, 4);
	g_gpio_dataout = ( unsigned long *)ioremap(0x44E07000+0x13C, 16);
	pr_info("g_gpio_oe addr is 0x%p\ng_gpio_dataout addr is 0x%p\n", g_gpio_oe, g_gpio_dataout);
	return 0;
}


static ssize_t my_misc_read(struct file *filp, char __user *buf, size_t len, loff_t *offset)
{
	int val = 0;

	val = readl(g_gpio_oe);
	val &= ~ (1<<22);
	writel(val, g_gpio_oe);
	
	
	val = readl(g_gpio_dataout);
	pr_info("io val = %x\n", (val >> 22) & 0x01);
	
	put_user(val, (char __user*)buf);
	
	return 0;
}

static ssize_t my_misc_write(struct file *filp, const char __user *buf, size_t len, loff_t *offset)
{
	int sw = 0;
	int val = 0;
	get_user(sw, (char __user *)buf);
	val = readl(g_gpio_oe);
	val &= ~ (1<<22);
	writel(val, g_gpio_oe);
	
	val = readl(g_gpio_dataout);
	if(sw == 1)
		val |=  (1 << 22);
	else
		val &= ~ (1<<22);
	writel(val, g_gpio_dataout);
	
	pr_info("write val is %d\n", sw);
	return 0;
}


struct file_operations misc_fops = {
	.owner = THIS_MODULE,
	.open = my_misc_open,
	.read = my_misc_read,
	.write = my_misc_write,
	
	
	
};
struct miscdevice my_misc = {
	.name = MISC_NAME,
	.minor = MISC_DYNAMIC_MINOR,
	.fops = &misc_fops,
	
	
	
};

static int my_led_drv_probe(struct platform_device *pdev)
{
	int ret = 0;
	ret = misc_register(&my_misc);  
    printk(MISC_NAME "\t register %s!\n", (0 == ret)?"successed":"failed");  
	
	
	
	
/*	
	struct resource *res = NULL;
	struct my_led_data *led_data = NULL;
	int i = 0;
	unsigned int val = 0;
	//volatile unsigned long *GPIO_OE = NULL;
	//volatile unsigned long *GPIO_DATAOUT = NULL;
	int cnt = 2;
	
	void __iomem *GPIO_OE = NULL;
	void __iomem *GPIO_DATAOUT = NULL;
	pr_info("%s start!\n", __func__);
	//GPIO0_22 led_1
	GPIO_OE = (volatile unsigned long *)ioremap(0x44E07000+0x134, 4);
	GPIO_DATAOUT = (volatile unsigned long *)ioremap(0x44E07000+0x13C, 16);
	pr_info("GPIO_OE addr is 0x%p\nGPIO_DATAOUT addr is 0x%p\n", GPIO_OE, GPIO_DATAOUT);

	//val = __raw_readl(GPIO_OE);
	val = readl(GPIO_OE);
	val &= ~ (1<<22);
	//__raw_writel(val, GPIO_OE);
	writel(val, GPIO_OE);
	
	//val = __raw_readl(GPIO_DATAOUT);
	val = readl(GPIO_DATAOUT);
	val &= ~ (1<<22);
	//__raw_writel(val, GPIO_DATAOUT);
	writel(val, GPIO_DATAOUT);
	ssleep(5);
	//val = __raw_readl(GPIO_DATAOUT);
	val = readl(GPIO_DATAOUT);
	val |=  (1<<22);
	//__raw_writel(val, GPIO_DATAOUT);
	writel(val, GPIO_DATAOUT);
	
	iounmap(GPIO_OE);
	iounmap(GPIO_DATAOUT);
*/
	
	/*
	//0x44E07000 GPIO_0
	//0x481AC000  GPIO_2
 	GPIO_OE = (volatile unsigned long *)ioremap(0x481AC000+0x134, 4);
	GPIO_DATAOUT = (volatile unsigned long *)ioremap(0x481AC000+0x13C, 16);
	//GPIO_DATAOUT = GPIO_OE+2;
	*GPIO_OE &= ~ (1<<6);
	while(cnt-- > 0)
	{
		*GPIO_DATAOUT &= ~(1<<6);
		ssleep(1);
		*GPIO_DATAOUT |= (1<<6);
		ssleep(1);
	}
	
	iounmap(GPIO_OE);
	iounmap(GPIO_DATAOUT);
	#endif
	
	*/
	
	
	/*
	
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!res){
		pr_info("platform_get_resource get IORESOURCE_MEM 0 failed!\n");
		return -ENOENT;
		
	}
	pr_info("resource 0 start is 0x%X\n", res->start);
	
	led_data = (struct my_led_data*)pdev->dev.platform_data;
	setup_pin_mux(led_data->pinmux_conf);
	pr_info("get platform_data!\n");
	while(i++ < 10)
	{
		gpio_set_value(led_data->pinmux_conf->gpio_num, i%2);
		ssleep(1);
	}
	
	*/
	
	
	return 0;	
	
	
}


static struct platform_driver my_led_drv = {
	.probe = my_led_drv_probe,
	.remove = my_led_drv_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "my_leds",
	},
};


static void my_led_release(struct device *dev)
{
	pr_info("%s!!", __func__);
}


static struct platform_device my_led_dev = {
	
	.name = "my_leds",
	.id = 0,
	.dev = {
		.platform_data = led_data,
		.release = my_led_release,
	
	},
	.num_resources = 1,
	.resource = my_led_resource,

};



static int __init myled__init(void)
{
	
	int ret = 0;
	ret = platform_device_register(&my_led_dev);
	if(ret){
		pr_info("platform_device_register failed!");
		return ret;
	}
	pr_info("platform_device_register ok!\n");	
	
	ret = platform_driver_register(&my_led_drv);
	if(ret){
		pr_info("platform_driver_register failed!");
		return ret;
		
	}
	pr_info("platform_driver_register ok!\n");
	
	
	
	
	

	return 0;
	
	
	

}

static void __exit myled__exit(void)
{
	platform_device_unregister(&my_led_dev);
	pr_info("platform_device_unregister ok!\n");
	
	
	
	platform_driver_unregister(&my_led_drv);
	pr_info("platform_driver_unregister ok!\n");
	
	misc_deregister(&my_misc);
	
	iounmap(g_gpio_dataout);
	iounmap(g_gpio_oe);

}


module_init(myled__init);  
module_exit(myled__exit);  
  
MODULE_LICENSE("GPL");  
MODULE_AUTHOR("yshisx");  




