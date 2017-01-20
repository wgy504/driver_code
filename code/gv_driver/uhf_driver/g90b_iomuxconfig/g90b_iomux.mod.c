#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
 .name = KBUILD_MODNAME,
 .init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
 .exit = cleanup_module,
#endif
 .arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x819eaac0, "module_layout" },
	{ 0x14ebfe7f, "cdev_del" },
	{ 0x11d8ea9b, "cdev_init" },
	{ 0xc8b57c27, "autoremove_wake_function" },
	{ 0xa9fbabc4, "no_llseek" },
	{ 0x3248be82, "omap_register_i2c_bus" },
	{ 0xfc1dce80, "device_destroy" },
	{ 0x403f9529, "gpio_request_one" },
	{ 0x7485e15e, "unregister_chrdev_region" },
	{ 0x891a1b10, "gpmc_cs_write_reg" },
	{ 0x729a72b2, "omap_mux_init_signal" },
	{ 0xf6288e02, "__init_waitqueue_head" },
	{ 0xfa2a45e, "__memzero" },
	{ 0xa964dd13, "gpmc_cs_request" },
	{ 0x27e1a049, "printk" },
	{ 0x82f776b7, "gpio_export" },
	{ 0x31c67579, "device_create" },
	{ 0xd6b8e852, "request_threaded_irq" },
	{ 0x43b0c9c3, "preempt_schedule" },
	{ 0xc6db89de, "cdev_add" },
	{ 0xbc477a2, "irq_set_irq_type" },
	{ 0x11f447ce, "__gpio_to_irq" },
	{ 0xc63280f2, "platform_device_register" },
	{ 0x3bd1b1f6, "msecs_to_jiffies" },
	{ 0xd62c833f, "schedule_timeout" },
	{ 0xb9e52429, "__wake_up" },
	{ 0xfe990052, "gpio_free" },
	{ 0x75a17bed, "prepare_to_wait" },
	{ 0xfb3f6b7e, "class_destroy" },
	{ 0x8893fa5d, "finish_wait" },
	{ 0x52ab4b26, "spi_register_board_info" },
	{ 0xc91a87e0, "__class_create" },
	{ 0x29537c9e, "alloc_chrdev_region" },
	{ 0xf20dabd8, "free_irq" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "E550B96D8FAC00F3C47174D");
