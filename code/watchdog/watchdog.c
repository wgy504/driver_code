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
  


  
  
  

module_init(my_wdt_init);  
module_exit(my_wdt_exit);  
  
MODULE_LICENSE("GPL");  
MODULE_AUTHOR("yshisx");  




