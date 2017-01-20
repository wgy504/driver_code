/*		
 *	name:	platform_test.c
 *	auth:	wuw
 *	date:	2016Äê3ÔÂ7ÈÕ 
 *	
 *
 */



#include <linux/device.h>  
#include <linux/module.h>  
#include <linux/kernel.h>  
#include <linux/init.h>  
#include <linux/string.h>  
#include <linux/sysfs.h>  
#include <linux/stat.h> 
#include <linux/slab.h> 












module_init(platform_test_init);  
module_exit(platform_test_exit);  


MODULE_AUTHOR("yshisx");  
MODULE_LICENSE("Dual BSD/GPL");

