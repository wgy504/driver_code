//#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <asm/uaccess.h>


#define DRIVER_VERSION "v2.0"
#define DRIVER_AUTHOR "yshisx"
#define DRIVER_DESC "my tty driver"

/* Module information */
MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL");

#define TTY_LAN_MINORS_NUM    5
#define TTY_LAN_MAJOR        212

static struct tty_driver *my_tty_driver;
//static struct tty_struct *my_tty_struct;
//static struct tty_struct *my_tty_struct;
static int open_count = 0;




static void my_tty_close(struct tty_struct *tty, struct file *filp)
{
    printk("ClOSE OK!\n");
    //kfree(my_tty_struct);
    return;
}


static int my_tty_open(struct tty_struct *tty, struct file *filp)
{
    if(open_count != 0){
        printk("Open failed!\n");
		return -EBUSY;
    }
    //my_tty_struct = kmalloc(sizeof(struct tty_struct), GFP_KERNEL);
    tty->low_latency = 1;
    //my_tty_struct = tty;

    return 0;
}

static int my_tty_put_char(struct tty_struct *tty, unsigned char ch)
{
    
    printk("put_char :%c\n", ch);
    return 0;
}

static int my_tty_write_room(struct tty_struct *tty)
{
    int room;
    room = 255;
    return room;
}

static int my_tty_write(struct tty_struct *tty, const unsigned char *buffer, int count)
{
 
    int retval = count;
    //tty = my_tty_struct;
	pr_info("tty->low_latency = %d\n", tty->low_latency);
    printk(KERN_DEBUG "%s - \n", __FUNCTION__);
    printk("count :%d\n", count);
    printk("user write: %s ", buffer);
    printk("\n");
    tty_insert_flip_string(tty, buffer, count);
    tty_flip_buffer_push(tty);
    
    return retval;
}

static void my_tty_set_termios(struct tty_struct *tty, struct ktermios *old)
{
    //tty = my_tty_struct;
    if(tty->termios->c_cflag == old->c_cflag){
        printk("Nothing to change!\n");
        return ;
    }
    printk("There is something to Change............\n");
    return ;
}

static struct tty_operations my_tty_ops = {
    .open = my_tty_open,
    .close = my_tty_close,
    .write = my_tty_write,
    .put_char = my_tty_put_char,
    .write_room = my_tty_write_room,
    .set_termios = my_tty_set_termios,
};

static int __init my_tty_init(void)
{

	int i;
    int retval;

    my_tty_driver = alloc_tty_driver(TTY_LAN_MINORS_NUM);
    if(!my_tty_driver){
		pr_info("alloc_tty_driver error!\n");
		return -ENOMEM;
	}
    pr_info("alloc_tty_driver ok!\n");    
    
    my_tty_driver->owner = THIS_MODULE;
    my_tty_driver->driver_name = "my_drv";
    my_tty_driver->name = "my_drv";
    my_tty_driver->major = TTY_LAN_MAJOR,
    my_tty_driver->minor_start = 0;    
    my_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
    my_tty_driver->subtype = SERIAL_TYPE_NORMAL;
    my_tty_driver->flags = TTY_DRIVER_REAL_RAW;
    my_tty_driver->init_termios = tty_std_termios;
    my_tty_driver->init_termios.c_cflag = B115200 | CS8 | CREAD | HUPCL | CLOCAL;
    tty_set_operations(my_tty_driver, &my_tty_ops);

    retval = tty_register_driver(my_tty_driver);
    if(retval){
        printk(KERN_ERR"Failed to register my_tty_driver!\n");
        put_tty_driver(my_tty_driver);
        return retval;
    }
	pr_info("tty_register_driver ok!\n");
	/*
    for(i = 0; i < TTY_LAN_MINORS_NUM; i++){
		dev_ptr = tty_register_device(my_tty_driver, i, NULL);
		if(dev_ptr == NULL){
			pr_info("tty_register_device %d error!\n", i);
			
		}
		else{
			pr_info("tty_register_device %d ok!\n", i);
			dev_ptr = NULL;
		
		}
			
	}
       */ 
    return 0;
	
	
}


static void __exit my_tty_exit(void)
{
	
	int i;
    for(i = 0; i < TTY_LAN_MINORS_NUM; i++)
        tty_unregister_device(my_tty_driver, i);
    tty_unregister_driver(my_tty_driver);
	
}





module_init(my_tty_init);
module_exit(my_tty_exit);





























