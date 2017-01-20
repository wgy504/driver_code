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
	{ 0x6bc3fbc0, "__unregister_chrdev" },
	{ 0x12da5bb2, "__kmalloc" },
	{ 0xfbc74f64, "__copy_from_user" },
	{ 0x67c2fa54, "__copy_to_user" },
	{ 0xaca89101, "dev_set_drvdata" },
	{ 0xd3dbfbc4, "_find_first_zero_bit_le" },
	{ 0xc5ae0182, "malloc_sizes" },
	{ 0xa9fbabc4, "no_llseek" },
	{ 0xa21176b, "__dynamic_pr_debug" },
	{ 0xfc1dce80, "device_destroy" },
	{ 0x432fd7f6, "__gpio_set_value" },
	{ 0x9be5f04f, "__register_chrdev" },
	{ 0x62b72b0d, "mutex_unlock" },
	{ 0xedafe2da, "spi_setup" },
	{ 0x91715312, "sprintf" },
	{ 0xf9179787, "nonseekable_open" },
	{ 0xf6288e02, "__init_waitqueue_head" },
	{ 0x5baaba0, "wait_for_completion" },
	{ 0xfa2a45e, "__memzero" },
	{ 0x19040566, "spi_async" },
	{ 0xdc798d37, "__mutex_init" },
	{ 0x27e1a049, "printk" },
	{ 0xbe7bd137, "driver_unregister" },
	{ 0xe16b893b, "mutex_lock" },
	{ 0x31c67579, "device_create" },
	{ 0xd6b8e852, "request_threaded_irq" },
	{ 0x43b0c9c3, "preempt_schedule" },
	{ 0x8836fca7, "kmem_cache_alloc" },
	{ 0x11f447ce, "__gpio_to_irq" },
	{ 0x24f67dee, "put_device" },
	{ 0x66bc7145, "__dynamic_dev_dbg" },
	{ 0xcb37b0dd, "get_device" },
	{ 0xb9e52429, "__wake_up" },
	{ 0x37a0cba, "kfree" },
	{ 0xfb3f6b7e, "class_destroy" },
	{ 0x676bbc0f, "_set_bit" },
	{ 0x60f71cfa, "complete" },
	{ 0x77bf2184, "spi_register_driver" },
	{ 0x49ebacbd, "_clear_bit" },
	{ 0xc3fe87c8, "param_ops_uint" },
	{ 0xc91a87e0, "__class_create" },
	{ 0xbd07ed98, "dev_get_drvdata" },
	{ 0xf20dabd8, "free_irq" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "4DA119FBF0598789F7C7449");
