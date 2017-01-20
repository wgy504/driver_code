#include <linux/init.h>
#include <linux/module.h> 


#define DEVICE_NAME "qt210_leds"
#define DEVICE_COUNT 1
#define QT210_LEDS_MAJOR 0
#define QT210_LEDS_MINOR 234
#define PARAM_SIZE 3

static unsigned char mem[4];
static int major = QT210_LEDS_MAJOR;
static int minor = QT210_LEDS_MINOR;
static dev_t dev_number;
static int leds_state = 1;
static char *params[]={"string1", "string2", "string3"};
static int param_size = PARAM_SIZE;

static struct class *leds_class = NULL;

static long qt210_leds_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch(cmd){
		case 0 :
		case 1 :
					if(arg > 3){
						return -EINVAL;
					}
					if(cmd == 1){
						gpio_set_value(S5PV210_GPH0(arg), 1);	
									
					}
					else{
						gpio_set_value(S5PV210_GPH0(arg), 0);	
						
					}
					return 0;
		default :
					return -EINVAL;		
	}	
}

static ssize_t qt210_leds_write(struct file *flie, const char _user *buf, size_t count, loff_t *ppos)
{
	unsigned tmp = count;
	unsigned long i = 0;
	memset(mem, 0, 4);	
	if(count > 4){
		tmp = 4;		
	}
	
	if(copy_from_user(mem, buf, tmp)){
		return -EFAULT;
	}
	else{
		for(i = 0; i < 4; i++){
			if(mem[i] = '1'){
				gpio_set_value(S5PV210_GPH0(i), 1);
							
			}
			else{
				gpio_set_value(S5PV210_GPH0(i), 0);		
				
			}
		}
		return conut;
		
		
		}
}



static struct file_operations dev_fops = {
	.ower = THIS_MODULE,
	.unlocked_ioctl = qt210_leds_ioctl,
	.write = qt210_leds_write,
	.read = led_read,
	
};

static struct cdev leds_cdev;

static int leds_create_device()
{
	int ret = 0;
	int err = 0;
	
	
	cdev_init(&leds_dev, &dev_fops);
	leds_cdev.owner = THIS_MODULE;
	if(major > 0){
		dev_number = MKDEV(major, minor);
		err = register_chrdev_region(dev_number, DEVICE_COUNT, DEVICE_NAME);
		if(err < 0){
			printk(KERN_WARING "register_char_region failed\n");
			return err;
		}

	}
	else{
		err = alloc_chrdev_region(&leds_cdev.dev, 10, DEVICE_COUNT, DEVICE_NAME);
		if(	err < 0){
			printk(KERN_WARING "alloc_chrdev_region failed! \n");
			return err;
		}
		major = MAJOR(leds_cdev.dev);	
		minor = MINOR(leds_cdev.dev);
		dev_number = ldes_cdev.dev;
	}
		ret = cdev_add(&leds_cdev, dev_number, DEVICE_COUNT);
		leds_class = class_create(THIS_MODULE, DEVICE_NAME);
		device_create(leds_class, NULL, dev_number, NULL, DEVICE_NAME);
		return ret;
}

static int leds_init_gpm(int leds_default)
{
	int err;
	int i;
	err = gpio_request(S5PV210_GPH(0), "LED0");
	if(err){
		printk(KERN_ER "failed to request GPH0(0) for LED0\n");
		return err;
	}	
	err = gpio_request(S5PV210_GPH(1), "LED1");
	if(err){
		printk(KERN_ER "failed to request GPH0(1) for LED1\n");
		return err;
	}
	for(i = 0; i < 2; i++){
		gpio_direction_output(S5PV210_GPH(i), 1);	
		gpio_set_value(S5PV210_GPH(i), leds_default);
		
	}
	return 0;
	
	
}


static int led_init(void)
{
	printk(KERN_ALERT "Hello, led driver £¡\n");
	int ret;
	ret = leds_create_device();
	leds_init_gpm(~leds_state);
	printk(DEVICE_NAME"\tinitialized\n");
	
	printk("param0\t%s\n", params[0]);
	printk("param1\t%s\n", params[1]);
	printk("param2\t%s\n", params[2]);
	
	return ret;
}

static void leds_destroy_device(void)
{
	device_destroy(leds_class, dev_number);
	if(leds_class)
		class_destroy(led_class);
	unregister_chrdev_region(dev_number, DEVICT_COUNT);
	return;
}



static void led_exit(void)
{
	printk(KERN_ALERT "Goodbye, led driver ! \n");
	int i;
	leds_destroy_device();
	for(i = 0; i < 4; i++){
		gpio_free(S5PV210_GPH0(i));			
	}
	
	
}

module_init(led_init);
module_exit(led_exit);

MODULE_LICENSE("Dual BSD/GPL");