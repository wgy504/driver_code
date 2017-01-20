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
	{ 0xfc1dce80, "device_destroy" },
	{ 0xc5ae0182, "malloc_sizes" },
	{ 0x37a0cba, "kfree" },
	{ 0x7485e15e, "unregister_chrdev_region" },
	{ 0x14ebfe7f, "cdev_del" },
	{ 0xfb3f6b7e, "class_destroy" },
	{ 0x31c67579, "device_create" },
	{ 0xc91a87e0, "__class_create" },
	{ 0xc6db89de, "cdev_add" },
	{ 0x11d8ea9b, "cdev_init" },
	{ 0x29537c9e, "alloc_chrdev_region" },
	{ 0xd8e484f0, "register_chrdev_region" },
	{ 0xfa2a45e, "__memzero" },
	{ 0x27e1a049, "printk" },
	{ 0x8836fca7, "kmem_cache_alloc" },
	{ 0x9798c3c3, "kill_fasync" },
	{ 0xbdc0db89, "fasync_helper" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "31DFC526BAA659E870C4EDB");
